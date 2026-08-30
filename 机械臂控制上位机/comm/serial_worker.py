"""
serial_worker.py
--------------------------------------------------------------------------
四轴机械臂串口通信调度器与离线仿真引擎
对应固件: MotorBus.c/.h 串行化安全调度机制

特点:
1. 双优先级指令队列 (急停/失能/清零 优先于 位置目标与轮询)
2. 四轴多机自动轮询 (位置/速度/状态标志/电压)
3. 真实物理响应离线仿真器 (无硬件时可无缝体验 UI 交互与数字孪生)
4. 串口热插拔检测与断线自动恢复
"""

import time
import queue
import threading
import serial
import serial.tools.list_ports
from dataclasses import dataclass
from typing import Optional, Callable, List, Dict, Tuple, Any

from protocol.emm_v5_protocol import (
    EmmV5Protocol,
    MotorProfile,
    MotorState,
    BusQueryType,
    MotorBusResult,
    NoticeType,
    SysParams,
    MOTORBUS_AXIS_COUNT,
    MOTORBUS_MIN_ANGLE_TENTHS,
    MOTORBUS_MAX_ANGLE_TENTHS,
    CHECK_BYTE
)
from protocol.emm_v5_protocol import MOTOR_FLAG_STALL, MOTOR_FLAG_STALL_PROTECT
from protocol.host_protocol import (
    Parser as HostParser, CMD_TARGET, CMD_ENABLE, CMD_ZERO, CMD_STOP,
    RESP_ACK, RESP_STATUS, CMD_SERVO, target as host_target, enable as host_enable,
    zero as host_zero, stop as host_stop, reset_clog as host_reset_clog,
    velocity as host_velocity, servo as host_servo, status_request, decode_status
)


@dataclass
class CommLogItem:
    """通信数据日志条目"""
    timestamp: float
    direction: str   # "TX" or "RX" or "SYS"
    content: str
    raw_hex: str
    level: str = "INFO"  # "INFO", "WARN", "ERROR", "DEBUG"


class SerialWorker:
    """机械臂串口通信与状态机调度工作线程"""

    PREPARE_TIMEOUT_SECONDS = 8.0

    def __init__(self):
        self.serial_port: Optional[serial.Serial] = None
        self.port_name: str = ""
        self.baud_rate: int = 115200
        self.is_running: bool = False
        self.is_connected: bool = False
        self.simulation_mode: bool = True  # 默认若无连接可直接切仿真

        # 四轴参数与状态
        self.profiles: List[MotorProfile] = [MotorProfile() for _ in range(MOTORBUS_AXIS_COUNT)]
        self.states: List[MotorState] = [MotorState() for _ in range(MOTORBUS_AXIS_COUNT)]
        
        # 仿真物理状态 (用于无硬件仿真模式下的平滑运动计算)
        self._sim_angles = [0.0, 0.0, 0.0, 0.0]        # 当前实时角度 (度)
        self._sim_target_angles = [0.0, 0.0, 0.0, 0.0] # 目标角度 (度)
        self._sim_speeds = [30.0, 30.0, 30.0, 30.0]    # 仿真速度 (RPM)
        self._sim_enabled = [False, False, False, False]
        self._sim_zeroed = [False, False, False, False]
        self._sim_stopped = [True, True, True, True]
        self.servo_angle = 90

        # 调度队列 (高优先级: 停止/失能/清零; 低优先级: 目标位置/设置)
        self.urgent_tx_queue: queue.Queue[Tuple[bytes, str]] = queue.Queue()
        self.normal_tx_queue: queue.Queue[Tuple[bytes, str]] = queue.Queue()

        # 轮询状态机
        self.query_axis = 0
        self.query_type_idx = 0
        self.query_sequence = [SysParams.S_CPOS, SysParams.S_VEL, SysParams.S_FLAG, SysParams.S_VBUS]
        self.last_query_time = 0.0
        self.query_interval = 0.5  # F407主动以100ms周期回传，这里仅作链路保活查询
        self.host_parser = HostParser()
        self._prepare_all_pending = False
        self._prepare_all_started = 0.0

        # 线程与锁
        self.worker_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # 线程安全 UI 消费队列
        self.log_queue: queue.Queue[CommLogItem] = queue.Queue()
        self.notice_queue: queue.Queue[Tuple[NoticeType, str]] = queue.Queue()
        self.conn_queue: queue.Queue[Tuple[bool, str, bool]] = queue.Queue()

        # 回调函数 (可选)
        self.on_state_updated: Optional[Callable[[int, MotorState, MotorProfile], None]] = None
        self.on_log_message: Optional[Callable[[CommLogItem], None]] = None
        self.on_connection_changed: Optional[Callable[[bool, str, bool], None]] = None
        self.on_notice: Optional[Callable[[NoticeType, str], None]] = None

    @staticmethod
    def get_available_ports() -> List[Dict[str, str]]:
        """获取系统当前可用串口列表"""
        ports = []
        for p in serial.tools.list_ports.comports():
            desc = p.description if p.description else "未知设备"
            ports.append({"port": p.device, "desc": f"{p.device} ({desc})"})
        return ports

    def connect(self, port: str, baud: int = 115200) -> bool:
        """连接物理串口"""
        self.disconnect()
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
                write_timeout=0.1
            )
            self.port_name = port
            self.baud_rate = baud
            self.is_connected = True
            self.simulation_mode = False
            self._prepare_all_pending = False
            with self.lock:
                for state in self.states:
                    state.online = False
                    state.enabled = False
                    state.zero_valid = False
                    state.stopped = True
                    state.actual_rpm = 0
                    state.last_rx_tick = 0.0
            self._log("SYS", f"成功连接串口: {port} (波特率: {baud})", "", "INFO")
            self._emit_notice(NoticeType.NONE, f"串口已连接: {port}")
            self.conn_queue.put((True, port, False))
            self._start_worker()
            if self.on_connection_changed:
                self.on_connection_changed(True, port, False)
            return True
        except Exception as e:
            self._log("SYS", f"串口连接失败: {str(e)}，已保持仿真模式", "", "WARN")
            self.is_connected = False
            self.conn_queue.put((False, port, self.simulation_mode))
            if self.on_connection_changed:
                self.on_connection_changed(False, port, self.simulation_mode)
            return False

    def start_simulation(self):
        """开启离线仿真模式"""
        self.disconnect()
        self.simulation_mode = True
        self.is_connected = True
        self.port_name = "SIMULATOR (离线仿真)"
        self._log("SYS", "已进入离线仿真模式 (可测试全套控制与数字孪生)", "", "INFO")
        self._emit_notice(NoticeType.NONE, "已进入离线仿真模式")
        self.conn_queue.put((True, self.port_name, True))
        
        # 初始化仿真状态
        with self.lock:
            for i in range(MOTORBUS_AXIS_COUNT):
                self.states[i].online = True
                self.states[i].enabled = False
                self.states[i].zero_valid = False
                self.states[i].stopped = True
                self.states[i].actual_angle_tenths = 0
                self.states[i].actual_rpm = 0
                self.states[i].bus_voltage = 24.1
                self.states[i].temperature = 31.5
                self._sim_angles[i] = 0.0
                self._sim_target_angles[i] = 0.0
                self._sim_enabled[i] = False
                self._sim_zeroed[i] = False
                self._sim_stopped[i] = True

        self._start_worker()
        if self.on_connection_changed:
            self.on_connection_changed(True, self.port_name, True)

    def disconnect(self):
        """断开连接或停止仿真"""
        self.is_running = False
        self._prepare_all_pending = False
        if self.worker_thread and self.worker_thread.is_alive():
            self.worker_thread.join(timeout=0.5)
        
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.is_connected = False
        self._log("SYS", "通信链路已断开", "", "INFO")
        self.conn_queue.put((False, "", self.simulation_mode))
        if self.on_connection_changed:
            self.on_connection_changed(False, "", self.simulation_mode)

    def _start_worker(self):
        """启动后台服务线程"""
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()

    # ==================== 安全控制业务 API ====================

    def request_angle(self, axis: int, target_angle_tenths: int) -> MotorBusResult:
        """提交单轴角度目标 (单位 0.1°) - 遵循固件安全检查"""
        if axis < 0 or axis >= MOTORBUS_AXIS_COUNT:
            return MotorBusResult.REJECT_AXIS
        
        if target_angle_tenths < MOTORBUS_MIN_ANGLE_TENTHS or target_angle_tenths > MOTORBUS_MAX_ANGLE_TENTHS:
            self._emit_notice(NoticeType.LIMIT, f"轴{axis+1} 目标角度超限: {target_angle_tenths/10.0:.1f}°")
            return MotorBusResult.REJECT_LIMIT

        state = self.states[axis]
        profile = self.profiles[axis]

        if not state.online:
            self._emit_notice(NoticeType.OFFLINE, f"轴{axis+1} 电机离线")
            return MotorBusResult.REJECT_OFFLINE

        if not state.zero_valid:
            self._emit_notice(NoticeType.ZERO, f"轴{axis+1} 零点未校准，请先归零")
            return MotorBusResult.REJECT_ZERO

        if not state.enabled:
            self._emit_notice(NoticeType.DISABLED, f"轴{axis+1} 未使能，无法执行运动")
            return MotorBusResult.REJECT_DISABLED

        if state.stopped:
            self._emit_notice(NoticeType.STOPPED, f"轴{axis+1} 处于急停锁定态")
            return MotorBusResult.REJECT_STOPPED

        profile.target_angle_tenths = target_angle_tenths

        if self.simulation_mode:
            self._sim_target_angles[axis] = target_angle_tenths / 10.0
            self._log("SIM", f"轴{axis+1} 设定目标角度: {target_angle_tenths/10.0:.1f}°", "", "DEBUG")
        else:
            cmd = host_target(axis, target_angle_tenths, profile.speed_rpm, profile.accel)
            self.normal_tx_queue.put((cmd, f"轴{axis+1} 绝对位置 -> {target_angle_tenths/10.0:.1f}°"))

        return MotorBusResult.OK

    def request_velocity(self, axis: int, direction: int, pressed: bool = True) -> MotorBusResult:
        """按住连续运行；松开立即走现有急停通道。"""
        if axis < 0 or axis >= MOTORBUS_AXIS_COUNT:
            return MotorBusResult.REJECT_AXIS
        if not pressed:
            state = self.states[axis]
            if not state.online:
                return MotorBusResult.REJECT_OFFLINE
            # Release must stop ahead of normal targets/queries, but must not
            # call request_stop(), which intentionally enters the lock state.
            self.urgent_tx_queue.put((
                host_velocity(axis, direction, 0, self.profiles[axis].accel),
                f"轴{axis+1} 松开按钮，立即停止（不锁定）"
            ))
            return MotorBusResult.OK
        state = self.states[axis]
        if not state.online:
            self._emit_notice(NoticeType.OFFLINE, f"轴{axis+1} 电机离线")
            return MotorBusResult.REJECT_OFFLINE
        if not state.zero_valid:
            self._emit_notice(NoticeType.ZERO, f"轴{axis+1} 零点未校准，请先归零")
            return MotorBusResult.REJECT_ZERO
        if not state.enabled:
            self._emit_notice(NoticeType.DISABLED, f"轴{axis+1} 未使能，请先使能")
            return MotorBusResult.REJECT_DISABLED
        if state.stopped:
            self._emit_notice(NoticeType.STOPPED, f"轴{axis+1} 处于急停锁定态")
            return MotorBusResult.REJECT_STOPPED
        self.normal_tx_queue.put((host_velocity(axis, direction, self.profiles[axis].speed_rpm,
                                                self.profiles[axis].accel), f"轴{axis+1} 连续运行"))
        return MotorBusResult.OK

    def set_speed(self, axis: int, speed_rpm: int):
        """设置单轴运行速度 (RPM)"""
        if 0 <= axis < MOTORBUS_AXIS_COUNT:
            self.profiles[axis].speed_rpm = min(max(speed_rpm, 0), 3000)
            if self.simulation_mode:
                self._sim_speeds[axis] = float(self.profiles[axis].speed_rpm)

    def set_accel(self, axis: int, accel: int):
        """设置单轴运行加速度 (0~255)"""
        if 0 <= axis < MOTORBUS_AXIS_COUNT:
            self.profiles[axis].accel = min(max(accel, 0), 255)

    def request_servo(self, angle: int):
        """Set MG90S gripper angle (0..180 degrees)."""
        self.servo_angle = min(max(int(angle), 0), 180)
        cmd = host_servo(self.servo_angle)
        if self.simulation_mode:
            self._log("SIM", f"夹爪舵机 -> {self.servo_angle}°", "", "DEBUG")
        else:
            self.normal_tx_queue.put((cmd, f"夹爪舵机 -> {self.servo_angle}°"))

    def request_enable(self, axis: int, enable: bool):
        """单轴使能/失能控制"""
        if axis < 0 or axis >= MOTORBUS_AXIS_COUNT:
            return
        
        cmd = host_enable(axis, enable)
        desc = f"轴{axis+1} {'使能' if enable else '失能'}"
        
        if enable:
            self.normal_tx_queue.put((cmd, desc))
        else:
            self.urgent_tx_queue.put((cmd, desc))
            
        if self.simulation_mode:
            self._sim_enabled[axis] = enable
            self.states[axis].enabled = enable
            if enable:
                self.states[axis].stopped = False
                self._sim_stopped[axis] = False

    def request_enable_all(self, enable: bool):
        """全部轴一键使能/失能"""
        for i in range(MOTORBUS_AXIS_COUNT):
            self.request_enable(i, enable)
        notice = NoticeType.ENABLED if enable else NoticeType.DISABLED_ALL
        self._emit_notice(notice, "四轴已全部使能" if enable else "四轴已全部失能")

    def request_stop(self, axis: int):
        """单轴紧急停止"""
        if axis < 0 or axis >= MOTORBUS_AXIS_COUNT:
            return
        cmd = host_stop(axis)
        self.urgent_tx_queue.put((cmd, f"轴{axis+1} 紧急停止"))
        self.states[axis].stopped = True
        if self.simulation_mode:
            self._sim_stopped[axis] = True
            self._sim_target_angles[axis] = self._sim_angles[axis]
            self.profiles[axis].target_angle_tenths = int(self._sim_angles[axis] * 10)
        self._emit_notice(NoticeType.STOPPED, f"轴{axis+1} 已急停锁定")

    def request_stop_all(self):
        """四轴一键急停"""
        for i in range(MOTORBUS_AXIS_COUNT):
            self.request_stop(i)
        self._emit_notice(NoticeType.STOPPED, "四轴已全部紧急停止！")

    def request_zero(self, axis: int):
        """单轴当前位置设为零点 (K6)，不触发机械运动。"""
        if axis < 0 or axis >= MOTORBUS_AXIS_COUNT:
            return
        cmd = host_zero(axis)
        self.urgent_tx_queue.put((cmd, f"轴{axis+1} 当前位置置零"))
        if self.simulation_mode:
            self._sim_angles[axis] = 0.0
            self._sim_target_angles[axis] = 0.0
            self.states[axis].actual_angle_tenths = 0
            self.profiles[axis].target_angle_tenths = 0
            self.states[axis].zero_valid = True
            self._sim_zeroed[axis] = True
        if self.simulation_mode:
            self._emit_notice(NoticeType.NONE, f"轴{axis+1} 当前位置已设为零点")
        else:
            self._emit_notice(NoticeType.NONE, f"轴{axis+1} 设零指令已发送，等待驱动器确认")

    def request_zero_all(self):
        """四轴一键准备：当前位置设零，全部确认后再使能。"""
        if not self.simulation_mode and not all(state.online for state in self.states):
            self._emit_notice(NoticeType.OFFLINE, "一键准备已取消：请先确认四轴均在线")
            return

        self._prepare_all_pending = True
        self._prepare_all_started = time.time()
        for i in range(MOTORBUS_AXIS_COUNT):
            self.request_zero(i)
        self._emit_notice(NoticeType.ZERO_ALL, "正在停机并将四轴当前位置设零，确认后自动使能")

    def _process_prepare_workflow(self):
        """Only enable after every motor has acknowledged the zero command."""
        if not self._prepare_all_pending:
            return

        if all(state.online and state.zero_valid for state in self.states):
            self._prepare_all_pending = False
            self.request_enable_all(True)
            self._emit_notice(NoticeType.ENABLED, "四轴设零确认完成，已自动发送全部使能")
            return

        if time.time() - self._prepare_all_started >= self.PREPARE_TIMEOUT_SECONDS:
            self._prepare_all_pending = False
            self._emit_notice(NoticeType.ZERO, "一键准备超时：未收到全部设零确认，电机保持失能")

    def request_reset_clog(self, axis: Optional[int] = None):
        """清除电机堵转报警"""
        axes = range(MOTORBUS_AXIS_COUNT) if axis is None else [axis]
        for ax in axes:
            self.urgent_tx_queue.put((host_reset_clog(ax), f"轴{ax+1} 解除堵转保护"))
            self.states[ax].stall_warning = False
        self._emit_notice(NoticeType.NONE, "堵转保护已解除")

    # ==================== 后台调度主循环 ====================

    def _worker_loop(self):
        """后台轮询与数据收发主循环"""
        rx_buffer = bytearray()

        while self.is_running:
            loop_start = time.time()

            if self.simulation_mode:
                self._update_simulation_physics()
            else:
                self._handle_hardware_io(rx_buffer)
            self._process_prepare_workflow()

            # 保持固定步长
            elapsed = time.time() - loop_start
            sleep_time = max(0.01 - elapsed, 0.002)
            time.sleep(sleep_time)

    def _handle_hardware_io(self, rx_buffer: bytearray):
        """物理串口通信收发与多轴轮询"""
        if not self.serial_port or not self.serial_port.is_open:
            return

        now = time.time()

        # 1. 优先发送紧急队列中的命令
        try:
            while not self.urgent_tx_queue.empty():
                cmd, desc = self.urgent_tx_queue.get_nowait()
                self.serial_port.write(cmd)
                self._log("TX", desc, cmd.hex().upper(), "WARN")
                time.sleep(0.005)  # 5ms 物理间隔，防止驱动器粘包

            # 2. 发送普通队列中的命令 (位置/使能)
            if not self.normal_tx_queue.empty():
                cmd, desc = self.normal_tx_queue.get_nowait()
                self.serial_port.write(cmd)
                self._log("TX", desc, cmd.hex().upper(), "INFO")
                time.sleep(0.005)

            # 3. F407 owns the motor polling; request its aggregate status.
            elif now - self.last_query_time >= self.query_interval:
                self.last_query_time = now
                self.serial_port.write(status_request())

            # 4. 接收并流式解析串口字节
            if self.serial_port.in_waiting > 0:
                chunk = self.serial_port.read(self.serial_port.in_waiting)
                rx_buffer.extend(chunk)

                # 解析缓冲区完整帧
                for frame_type, payload in self.host_parser.feed(bytes(rx_buffer)):
                    self._apply_host_frame(frame_type, payload)
                rx_buffer.clear()

        except Exception as e:
            self._log("SYS", f"串口通信异常: {str(e)}", "", "ERROR")
            self._emit_notice(NoticeType.BUS, "串口通信异常中断")

    def _apply_host_frame(self, frame_type: int, payload: bytes):
        if frame_type == RESP_STATUS:
            decoded = decode_status(payload)
            if decoded is None:
                return
            online, enabled, zeroed, stopped = decoded[0]
            with self.lock:
                for axis, (angle, rpm, flags, errors) in enumerate(decoded[1]):
                    state = self.states[axis]
                    bit = 1 << axis
                    state.online = bool(online & bit)
                    state.enabled = bool(enabled & bit)
                    state.zero_valid = bool(zeroed & bit)
                    state.stopped = bool(stopped & bit)
                    state.actual_angle_tenths = angle
                    state.actual_rpm = rpm
                    state.flags = flags
                    state.comm_errors = errors
                    state.last_rx_tick = time.time()
                    state.stall_warning = bool(flags & (MOTOR_FLAG_STALL | MOTOR_FLAG_STALL_PROTECT))
            return
        if frame_type == RESP_ACK and len(payload) == 3:
            command, axis, result = payload
            try:
                result_enum = MotorBusResult(result)
            except ValueError:
                result_enum = MotorBusResult.REJECT_BUS_FAULT
            if result_enum != MotorBusResult.OK:
                if command == CMD_ZERO:
                    self._prepare_all_pending = False
                self._emit_notice(NoticeType.BUS, f"F407拒绝命令: 轴{axis+1}，结果码{result}")

    def _parse_rx_buffer(self, rx_buffer: bytearray):
        """流式状态机解析反馈帧"""
        while len(rx_buffer) >= 4:
            # 帧头必须是合法地址 (1~4)
            addr = rx_buffer[0]
            if addr < 1 or addr > MOTORBUS_AXIS_COUNT:
                rx_buffer.pop(0)
                continue

            code = rx_buffer[1]
            expected_len = EmmV5Protocol.get_expected_frame_length(code)
            if expected_len == 0:
                rx_buffer.pop(0)
                continue

            if len(rx_buffer) < expected_len:
                break  # 等待更多字节

            frame = bytes(rx_buffer[:expected_len])
            if frame[-1] == CHECK_BYTE:
                # 提取完整合法帧
                del rx_buffer[:expected_len]
                parsed = EmmV5Protocol.parse_frame(frame)
                if parsed:
                    self._apply_parsed_frame(parsed, frame)
            else:
                # 校验字节不对，滑窗前进 1 字节
                rx_buffer.pop(0)

    def _apply_parsed_frame(self, parsed: Dict[str, Any], raw_frame: bytes):
        """将解析到的遥测数据更新到对应轴的状态中"""
        axis = parsed["axis"]
        p_type = parsed.get("type", "")
        state = self.states[axis]

        with self.lock:
            state.online = True
            state.last_rx_tick = time.time()

            if p_type == "position":
                state.actual_angle_tenths = parsed["angle_tenths"]
                self._log("RX", f"轴{axis+1} 实时角度: {parsed['angle_tenths']/10.0:.1f}°", raw_frame.hex().upper(), "DEBUG")

            elif p_type == "speed":
                state.actual_rpm = parsed["actual_rpm"]

            elif p_type == "flags":
                state.flags = parsed["flags"]
                state.enabled = parsed["enabled"]
                if parsed["stalled"]:
                    state.stopped = True
                    state.stall_warning = True
                    self._emit_notice(NoticeType.STALL_ALERT, f"轴{axis+1} 发生堵转！")

            elif p_type == "vbus":
                state.bus_voltage = parsed["bus_voltage"]

            elif p_type == "temp":
                state.temperature = parsed["temperature"]

            elif p_type == "zero_ack":
                state.zero_valid = True
                state.actual_angle_tenths = 0
                self.profiles[axis].target_angle_tenths = 0

            elif p_type == "enable_ack":
                state.enabled = True
                state.stopped = False

            elif p_type == "stop_ack":
                state.stopped = True

    def _update_simulation_physics(self):
        """仿真模式下的机械动力学平滑插补计算"""
        dt = 0.02  # 20ms 仿真步长
        with self.lock:
            for i in range(MOTORBUS_AXIS_COUNT):
                state = self.states[i]
                state.online = True

                if not self._sim_enabled[i] or self._sim_stopped[i]:
                    state.actual_rpm = 0
                    continue

                curr = self._sim_angles[i]
                target = self._sim_target_angles[i]
                diff = target - curr

                if abs(diff) > 0.05:
                    # 速度换算: RPM -> 度/秒 (1 RPM = 360°/60s = 6°/s)
                    max_d_deg = self._sim_speeds[i] * 6.0 * dt
                    step = min(abs(diff), max_d_deg)
                    sign = 1.0 if diff > 0 else -1.0
                    self._sim_angles[i] += sign * step
                    state.actual_rpm = int(sign * self._sim_speeds[i])
                else:
                    self._sim_angles[i] = target
                    state.actual_rpm = 0

                state.actual_angle_tenths = int(round(self._sim_angles[i] * 10))

    def _log(self, direction: str, content: str, raw_hex: str = "", level: str = "INFO"):
        """记录通信日志"""
        item = CommLogItem(
            timestamp=time.time(),
            direction=direction,
            content=content,
            raw_hex=raw_hex,
            level=level
        )
        self.log_queue.put(item)
        if self.on_log_message:
            try:
                self.on_log_message(item)
            except Exception:
                pass

    def _emit_notice(self, n_type: NoticeType, msg: str):
        """派发系统通知"""
        self.notice_queue.put((n_type, msg))
        if self.on_notice:
            try:
                self.on_notice(n_type, msg)
            except Exception:
                pass

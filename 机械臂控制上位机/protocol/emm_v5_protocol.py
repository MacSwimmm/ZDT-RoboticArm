"""
emm_v5_protocol.py
--------------------------------------------------------------------------
四轴机械臂通信协议与状态机编解码模块 (Emm_V5 闭环步进驱动器协议)
对应固件: MotorBus.c/.h, ArmControl.c/.h, Emm_V5.c/.h

轴定义:
  Axis 0 (ID 1): SCREW   - 丝杆滑台
  Axis 1 (ID 2): BASE    - 整体底座
  Axis 2 (ID 3): UPPER   - 大臂关节
  Axis 3 (ID 4): FOREARM - 小臂关节
"""

import enum
import struct
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any


# 协议常量
MOTORBUS_AXIS_COUNT = 4
MOTORBUS_MIN_ANGLE_TENTHS = -1800   # -180.0°
MOTORBUS_MAX_ANGLE_TENTHS = 1800    # +180.0°
MOTORBUS_DEFAULT_SPEED_RPM = 30
MOTORBUS_DEFAULT_ACCEL = 20
MOTORBUS_PULSES_PER_REV = 3200
CHECK_BYTE = 0x6B

# 驱动器标志位掩码 (0x3A 寄存器)
MOTOR_FLAG_ENABLED = 0x01
MOTOR_FLAG_ARRIVED = 0x02
MOTOR_FLAG_STALL = 0x04
MOTOR_FLAG_STALL_PROTECT = 0x08


class SysParams(enum.IntEnum):
    """系统参数读取功能码"""
    S_VBUS  = 0x24  # 读取总线电压
    S_CBUS  = 0x26  # 读取总线电流
    S_CPHA  = 0x27  # 读取相电流
    S_ENCO  = 0x29  # 读取编码器原始值
    S_CLKC  = 0x30  # 读取实时脉冲数
    S_ENCL  = 0x31  # 读取经过线性化校准后的编码器值
    S_CLKI  = 0x32  # 读取输入脉冲数
    S_TPOS  = 0x33  # 读取电机目标位置
    S_SPOS  = 0x34  # 读取电机实时设定的目标位置
    S_VEL   = 0x35  # 读取电机实时转速
    S_CPOS  = 0x36  # 读取电机实时位置
    S_PERR  = 0x37  # 读取电机位置误差
    S_VBAT  = 0x38  # 读取多圈编码器电池电压
    S_TEMP  = 0x39  # 读取电机实时温度
    S_FLAG  = 0x3A  # 读取电机状态标志位
    S_OFLAG = 0x3B  # 读取回零状态标志位
    S_OAF   = 0x3C  # 读取电机状态标志位 + 回零状态标志位
    S_PIN   = 0x3D  # 读取引脚状态


class BusQueryType(enum.IntEnum):
    """主循环周期性轮询查询项"""
    POSITION = 0  # S_CPOS (0x36)
    SPEED    = 1  # S_VEL  (0x35)
    FLAGS    = 2  # S_FLAG (0x3A)
    VBUS     = 3  # S_VBUS (0x24)


class MotorBusResult(enum.IntEnum):
    """命令提交结果代码 (与固件 MotorBusResult_t 一致)"""
    OK = 0
    REJECT_AXIS = 1
    REJECT_LIMIT = 2
    REJECT_OFFLINE = 3
    REJECT_DISABLED = 4
    REJECT_ZERO = 5
    REJECT_STOPPED = 6
    REJECT_BUS_FAULT = 7


class NoticeType(enum.IntEnum):
    """系统提示与报警信息 (与固件 OledNotice_t 一致)"""
    NONE = 0
    LIMIT = 1           # "REJECT ANGLE LIMIT" - 角度超限
    OFFLINE = 2         # "REJECT MOTOR OFF" - 电机离线
    DISABLED = 3        # "REJECT DISABLED" - 电机未使能
    ZERO = 4            # "ZERO REQUIRED" - 需先校准零点
    STOPPED = 5         # "STOP LOCK ACTIVE" - 处于急停锁定态
    BUS = 6             # "MOTOR BUS FAULT" - 总线通信故障
    ZERO_ALL = 7        # "ZERO ALL QUEUED" - 已排队全部归零
    ENABLED = 8         # "ENABLE ALL QUEUED" - 已排队全部使能
    DISABLED_ALL = 9    # "DISABLE ALL QUEUED" - 已排队全部失能
    STALL_ALERT = 10    # "MOTOR STALL DETECTED" - 发生堵转报警


NOTICE_MESSAGES = {
    NoticeType.NONE: ("正常运行", "#00F5A0"),
    NoticeType.LIMIT: ("拒绝动作: 目标角度超出软限位 (-180.0° ~ +180.0°)", "#FF416C"),
    NoticeType.OFFLINE: ("拒绝动作: 目标电机离线无响应", "#FF416C"),
    NoticeType.DISABLED: ("拒绝动作: 电机处于失能状态，请先使能", "#FF9F43"),
    NoticeType.ZERO: ("拒绝动作: 本次上电尚未确认零点，请先归零", "#FF9F43"),
    NoticeType.STOPPED: ("拒绝动作: 电机处于急停锁定状态", "#FF416C"),
    NoticeType.BUS: ("总线故障: 串口通信异常或断开", "#FF416C"),
    NoticeType.ZERO_ALL: ("一键准备中: 四轴当前位置设零，确认后自动使能", "#00F2FE"),
    NoticeType.ENABLED: ("指令已发送: 四轴已全部使能锁定", "#00F5A0"),
    NoticeType.DISABLED_ALL: ("指令已发送: 四轴已全部释放失能", "#8A2387"),
    NoticeType.STALL_ALERT: ("严重警告: 检测到电机堵转！请检查机械结构", "#FF0033"),
}

AXIS_NAMES_EN = ["SCREW", "BASE", "UPPER", "FOREARM"]
AXIS_NAMES_CN = ["丝杆滑台 (Z/X)", "底座旋转 (Yaw)", "大臂关节 (Pitch1)", "小臂关节 (Pitch2)"]


@dataclass
class MotorProfile:
    """期望运动曲线参数"""
    target_angle_tenths: int = 0       # 单位 0.1° (-1800 ~ +1800)
    speed_rpm: int = MOTORBUS_DEFAULT_SPEED_RPM  # 0 ~ 3000 RPM
    accel: int = MOTORBUS_DEFAULT_ACCEL          # 0 ~ 255 (0为直接启动)


@dataclass
class MotorState:
    """驱动器实时遥测状态"""
    online: bool = False               # 在线指示
    enabled: bool = False              # 使能状态
    zero_valid: bool = False           # 零点已校准有效
    stopped: bool = True               # 停止/锁定
    flags: int = 0                     # 0x3A 原始标志位
    actual_angle_tenths: int = 0       # 0x36 实时角度，单位 0.1°
    actual_rpm: int = 0                # 0x35 实时转速，带方向
    bus_voltage: float = 24.0          # 0x24 实时总线电压 (V)
    temperature: float = 25.0          # 0x39 实时驱动器温度 (°C)
    last_rx_tick: float = 0.0          # 最近收到反馈的时间戳
    comm_errors: int = 0               # 累计超时与丢包次数
    stall_warning: bool = False        # 是否处于堵转警告中


class EmmV5Protocol:
    """张大头 Emm_V5 协议编解码器"""

    @staticmethod
    def angle_to_pulses(angle_tenths: int) -> int:
        """
        根据 0.1° 单位角度计算脉冲数 (四舍五入)
        3200 脉冲 / 360° = 8.8888 脉冲/度 = 0.8888 脉冲/0.1°
        """
        magnitude = abs(angle_tenths)
        return (magnitude * MOTORBUS_PULSES_PER_REV + 1800) // 3600

    @staticmethod
    def position_raw_to_angle_tenths(sign: int, raw: int) -> int:
        """
        驱动器单圈 0~65535 编码器值转换到 -180.0° ~ +180.0° (单位 0.1°)
        """
        angle = ((raw & 0xFFFF) * 3600 + 32768) // 65536
        if sign != 0:
            angle = -angle
        if angle > MOTORBUS_MAX_ANGLE_TENTHS:
            angle -= 3600
        elif angle < MOTORBUS_MIN_ANGLE_TENTHS:
            angle += 3600
        return int(angle)

    @staticmethod
    def format_angle(tenths: int) -> str:
        """格式化角度字符串，例如 +120.5°"""
        sign = "-" if tenths < 0 else "+"
        mag = abs(tenths)
        return f"{sign}{mag // 10}.{mag % 10:01d}°"

    # ==================== 打包发送指令 ====================

    @classmethod
    def pack_pos_control(
        cls,
        addr: int,
        target_angle_tenths: int,
        speed_rpm: int,
        accel: int,
        is_abs: bool = True,
        is_sync: bool = False
    ) -> bytes:
        """
        生成位置控制指令 (0xFD)
        帧长 13 字节: [addr, 0xFD, dir, vel_H, vel_L, acc, clk3, clk2, clk1, clk0, raF, snF, 0x6B]
        """
        direction = 1 if target_angle_tenths < 0 else 0
        pulses = cls.angle_to_pulses(target_angle_tenths)
        speed = min(max(speed_rpm, 0), 5000)
        acc = min(max(accel, 0), 255)
        
        return struct.pack(
            ">BBBHHBIBBB",
            addr,
            0xFD,
            direction,
            speed,
            acc,
            pulses,
            1 if is_abs else 0,
            1 if is_sync else 0,
            CHECK_BYTE
        )

    @classmethod
    def pack_vel_control(
        cls,
        addr: int,
        direction: int,
        speed_rpm: int,
        accel: int,
        is_sync: bool = False
    ) -> bytes:
        """
        生成速度控制指令 (0xF6)
        帧长 8 字节: [addr, 0xF6, dir, vel_H, vel_L, acc, snF, 0x6B]
        """
        speed = min(max(speed_rpm, 0), 5000)
        acc = min(max(accel, 0), 255)
        return struct.pack(
            ">BBBHBBB",
            addr,
            0xF6,
            direction & 0x01,
            speed,
            acc,
            1 if is_sync else 0,
            CHECK_BYTE
        )

    @classmethod
    def pack_enable(cls, addr: int, state: bool, is_sync: bool = False) -> bytes:
        """
        生成使能控制指令 (0xF3)
        帧长 6 字节: [addr, 0xF3, 0xAB, state, snF, 0x6B]
        """
        return bytes([addr, 0xF3, 0xAB, 1 if state else 0, 1 if is_sync else 0, CHECK_BYTE])

    @classmethod
    def pack_stop(cls, addr: int, is_sync: bool = False) -> bytes:
        """
        生成立即停止指令 (0xFE)
        帧长 5 字节: [addr, 0xFE, 0x98, snF, 0x6B]
        """
        return bytes([addr, 0xFE, 0x98, 1 if is_sync else 0, CHECK_BYTE])

    @classmethod
    def pack_reset_curpos_to_zero(cls, addr: int) -> bytes:
        """
        生成将当前位置清零指令 (0x0A)
        帧长 4 字节: [addr, 0x0A, 0x6D, 0x6B]
        """
        return bytes([addr, 0x0A, 0x6D, CHECK_BYTE])

    @classmethod
    def pack_reset_clog_pro(cls, addr: int) -> bytes:
        """
        生成解除堵转保护指令 (0x0E)
        帧长 4 字节: [addr, 0x0E, 0x52, 0x6B]
        """
        return bytes([addr, 0x0E, 0x52, CHECK_BYTE])

    @classmethod
    def pack_trig_encoder_cal(cls, addr: int) -> bytes:
        """
        生成触发编码器校准指令 (0x06)
        帧长 4 字节: [addr, 0x06, 0x45, 0x6B]
        """
        return bytes([addr, 0x06, 0x45, CHECK_BYTE])

    @classmethod
    def pack_reset_motor(cls, addr: int) -> bytes:
        """
        生成重启电机指令 (0x08)
        帧长 4 字节: [addr, 0x08, 0x97, 0x6B]
        """
        return bytes([addr, 0x08, 0x97, CHECK_BYTE])

    @classmethod
    def pack_read_sys_params(cls, addr: int, param: SysParams) -> bytes:
        """
        生成读取系统参数指令 (0x1F / Read_Sys_Params)
        帧长 3 字节: [addr, param_code, 0x6B]
        """
        return bytes([addr, int(param), CHECK_BYTE])

    @classmethod
    def pack_origin_trigger(cls, addr: int, o_mode: int = 0, is_sync: bool = False) -> bytes:
        """
        生成触发回零指令 (0x9A)
        帧长 5 字节: [addr, 0x9A, o_mode, snF, 0x6B]
        """
        return bytes([addr, 0x9A, o_mode & 0x03, 1 if is_sync else 0, CHECK_BYTE])

    @classmethod
    def pack_origin_interrupt(cls, addr: int) -> bytes:
        """
        生成强制中断并退出回零指令 (0x9C)
        帧长 4 字节: [addr, 0x9C, 0x48, 0x6B]
        """
        return bytes([addr, 0x9C, 0x48, CHECK_BYTE])

    @classmethod
    def pack_modify_motor_id(cls, addr: int, new_id: int, save: bool = True) -> bytes:
        """
        修改电机 ID 地址指令 (0xAE)
        帧长 5 字节: [addr, 0xAE, 0x4B, 1 if save else 0, new_id, 0x6B]
        """
        return bytes([addr, 0xAE, 0x4B, 1 if save else 0, new_id & 0xFF, CHECK_BYTE])

    # ==================== 解包接收反馈 ====================

    @classmethod
    def get_expected_frame_length(cls, code: int) -> int:
        """根据返回功能码获取预期的完整帧长"""
        if code == 0x36:       # S_CPOS 实时位置
            return 8
        elif code == 0x35:     # S_VEL 实时转速
            return 6
        elif code == 0x24:     # S_VBUS 总线电压
            return 5
        elif code == 0x39:     # S_TEMP 实时温度
            return 5
        elif code in (0x3A, 0x0A, 0xF3, 0xFD, 0xFE, 0x0E, 0x06, 0x08, 0x9A, 0x9C):
            return 4
        return 0

    @classmethod
    def parse_frame(cls, frame: bytes) -> Optional[Dict[str, Any]]:
        """
        解析单包合法协议帧
        frame 格式: [addr, func_code, payload..., 0x6B]
        """
        length = len(frame)
        if length < 4 or frame[-1] != CHECK_BYTE:
            return None

        addr = frame[0]
        if addr < 1 or addr > MOTORBUS_AXIS_COUNT:
            return None

        axis = addr - 1
        code = frame[1]
        result = {"axis": axis, "addr": addr, "code": code}

        if code == 0x36 and length == 8:
            # 实时位置: [addr, 0x36, sign, pos_3, pos_2, pos_1, pos_0, 0x6B]
            sign = frame[2]
            raw = (frame[3] << 24) | (frame[4] << 16) | (frame[5] << 8) | frame[6]
            result["type"] = "position"
            result["angle_tenths"] = cls.position_raw_to_angle_tenths(sign, raw)
            return result

        elif code == 0x35 and length == 6:
            # 实时转速: [addr, 0x35, sign, vel_H, vel_L, 0x6B]
            sign = frame[2]
            speed = (frame[3] << 8) | frame[4]
            rpm = -speed if sign != 0 else speed
            result["type"] = "speed"
            result["actual_rpm"] = rpm
            return result

        elif code == 0x3A and length == 4:
            # 状态标志: [addr, 0x3A, flags, 0x6B]
            flags = frame[2]
            result["type"] = "flags"
            result["flags"] = flags
            result["enabled"] = bool(flags & MOTOR_FLAG_ENABLED)
            result["stalled"] = bool(flags & (MOTOR_FLAG_STALL | MOTOR_FLAG_STALL_PROTECT))
            return result

        elif code == 0x24 and length == 5:
            # 总线电压: [addr, 0x24, v_H, v_L, 0x6B]
            v_raw = (frame[2] << 8) | frame[3]
            result["type"] = "vbus"
            result["bus_voltage"] = v_raw / 100.0  # 假设单位为 0.01V 或 0.1V
            return result

        elif code == 0x39 and length == 5:
            # 温度: [addr, 0x39, sign, temp, 0x6B]
            sign = frame[2]
            temp = frame[3]
            result["type"] = "temp"
            result["temperature"] = -temp if sign != 0 else temp
            return result

        elif code == 0x0A and length == 4:
            # 清零确认: [addr, 0x0A, 0x02, 0x6B]
            result["type"] = "zero_ack"
            result["success"] = (frame[2] == 0x02)
            return result

        elif code == 0xF3 and length == 4:
            # 使能确认: [addr, 0xF3, 0x02, 0x6B]
            result["type"] = "enable_ack"
            result["success"] = (frame[2] == 0x02)
            return result

        elif code == 0xFE and length == 4:
            # 停止确认: [addr, 0xFE, 0x02, 0x6B]
            result["type"] = "stop_ack"
            result["success"] = (frame[2] == 0x02)
            return result

        elif code == 0x0E and length == 4:
            # 解除堵转确认: [addr, 0x0E, 0x02, 0x6B]
            result["type"] = "clog_ack"
            result["success"] = (frame[2] == 0x02)
            return result

        return result

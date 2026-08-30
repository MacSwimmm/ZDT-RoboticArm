"""
main.py
--------------------------------------------------------------------------
四轴机械臂控制上位机 (Robotic Arm Host Controller)
STM32F407 固件与张大头 Emm_V5 闭环步进驱动器配套全屏控制系统
"""

import os
import sys
import customtkinter as ctk
import tkinter as tk
from typing import List, Dict, Any, Tuple

from ui.theme import Theme
from comm.serial_worker import SerialWorker, CommLogItem
from protocol.emm_v5_protocol import (
    MotorState,
    MotorProfile,
    NoticeType,
    MOTORBUS_AXIS_COUNT,
    MOTORBUS_MIN_ANGLE_TENTHS,
    MOTORBUS_MAX_ANGLE_TENTHS
)
from ui.components.header_bar import HeaderBar
from ui.components.axis_card import AxisCard
from ui.components.trajectory_panel import TrajectoryPanel
from ui.components.monitor_panel import MonitorPanel
from ui.components.servo_card import ServoCard


class RoboticArmApp(ctk.CTk):
    """四轴机械臂全屏控制上位机主窗体"""

    def __init__(self):
        super().__init__()

        # 1. 基础窗口与全屏配置
        self.title("FOUR-2 四轴机械臂全功能控制系统 - Host Controller")
        self.geometry("1680x950")
        self.minsize(1280, 800)
        self.configure(fg_color=Theme.BG_MAIN)

        # 设置 CustomTkinter 深色主题
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self.is_fullscreen = False

        # 2. 实例化通信与后台调度引擎
        self.worker = SerialWorker()

        # 3. 构建用户界面
        self._build_layout()

        # 4. 绑定全局热键
        self.bind("<F11>", lambda e: self._toggle_fullscreen())
        self.bind("<Escape>", lambda e: self._exit_fullscreen())
        self.bind("<space>", lambda e: self._on_emergency_stop())

        # 6. 启动 UI 定时轮询器 (40Hz)
        self.after(25, self._poll_ui_loop)

        # 7. 默认开启离线仿真模式以保证界面直接可用
        self.after(200, self.worker.start_simulation)

        # 窗口关闭事件处理
        self.protocol("WM_DELETE_WINDOW", self._on_closing)

    def _build_layout(self):
        """构建自适应大屏的多列无翻页仪表盘布局"""
        self.grid_rowconfigure(0, weight=0)  # 顶部 Header 导航栏
        self.grid_rowconfigure(1, weight=1)  # 主工作区
        self.grid_columnconfigure(0, weight=1)

        # ----------------- 1. 顶部状态与通信控制栏 -----------------
        self.header_bar = HeaderBar(
            self,
            on_connect=self._on_connect_serial,
            on_disconnect=self._on_disconnect_serial,
            on_toggle_sim=self._on_toggle_simulation,
            on_emergency_stop=self._on_emergency_stop,
            on_toggle_fullscreen=self._toggle_fullscreen,
            on_refresh_ports=self.worker.get_available_ports
        )
        self.header_bar.grid(row=0, column=0, padx=12, pady=(10, 6), sticky="ew")

        # ----------------- 2. 主工作区分栏容器 (2 列控制网格) -----------------
        main_container = ctk.CTkFrame(self, fg_color="transparent")
        main_container.grid(row=1, column=0, padx=12, pady=(0, 10), sticky="nsew")

        # 移除中间仿真区后，四轴和辅助控制区扩大填充空间。
        main_container.grid_columnconfigure(0, weight=60)
        main_container.grid_columnconfigure(1, weight=40)
        main_container.grid_rowconfigure(0, weight=1)

        # ==================== 第 1 列: 四轴独立控制卡片 (2x2 阵列) ====================
        col_axes = ctk.CTkFrame(main_container, fg_color="transparent")
        col_axes.grid(row=0, column=0, padx=(0, 6), pady=0, sticky="nsew")
        col_axes.grid_rowconfigure(0, weight=1)
        col_axes.grid_rowconfigure(1, weight=1)
        col_axes.grid_columnconfigure(0, weight=1)
        col_axes.grid_columnconfigure(1, weight=1)

        self.axis_cards: List[AxisCard] = []
        for i in range(MOTORBUS_AXIS_COUNT):
            row = i // 2
            col = i % 2
            card = AxisCard(
                col_axes,
                axis_id=i,
                on_angle_changed=self._on_axis_angle_changed,
                on_speed_changed=self._on_axis_speed_changed,
                on_accel_changed=self._on_axis_accel_changed,
                on_enable_toggle=self._on_axis_enable_toggle,
                on_stop_axis=self._on_axis_stop,
                on_zero_axis=self._on_axis_zero,
                on_jog=self._on_axis_jog,
                on_reset_clog=self._on_axis_reset_clog
            )
            card.grid(row=row, column=col, padx=4, pady=4, sticky="nsew")
            self.axis_cards.append(card)

        # ==================== 第 2 列: 夹爪舵机 + 示教轨迹 + 遥测 ====================
        col_right = ctk.CTkFrame(main_container, fg_color="transparent")
        col_right.grid(row=0, column=1, padx=(6, 0), pady=0, sticky="nsew")
        col_right.grid_rowconfigure(0, weight=25)
        col_right.grid_rowconfigure(1, weight=37)
        col_right.grid_rowconfigure(2, weight=38)
        col_right.grid_columnconfigure(0, weight=1)

        self.servo_card = ServoCard(col_right, on_angle_changed=self._on_servo_angle_changed)
        self.servo_card.grid(row=0, column=0, padx=0, pady=(0, 4), sticky="nsew")

        # 示教再现面板
        self.trajectory_panel = TrajectoryPanel(
            col_right,
            get_current_angles=self._get_all_current_angles,
            execute_pose=self._execute_multi_axis_pose
        )
        self.trajectory_panel.grid(row=1, column=0, padx=0, pady=4, sticky="nsew")

        # 遥测监视与通讯日志面板
        self.monitor_panel = MonitorPanel(
            col_right,
            on_zero_all=self.worker.request_zero_all,
            on_enable_all=self.worker.request_enable_all,
            on_reset_all_clog=self.worker.request_reset_clog
        )
        self.monitor_panel.grid(row=2, column=0, padx=0, pady=(4, 0), sticky="nsew")

    # ==================== 业务事件中继处理 ====================

    def _on_axis_angle_changed(self, axis_id: int, target_tenths: int):
        self.worker.request_angle(axis_id, target_tenths)

    def _on_axis_speed_changed(self, axis_id: int, speed_rpm: int):
        self.worker.set_speed(axis_id, speed_rpm)

    def _on_axis_accel_changed(self, axis_id: int, accel: int):
        self.worker.set_accel(axis_id, accel)

    def _on_axis_enable_toggle(self, axis_id: int, enable: bool):
        self.worker.request_enable(axis_id, enable)

    def _on_axis_stop(self, axis_id: int):
        self.worker.request_stop(axis_id)

    def _on_axis_zero(self, axis_id: int):
        self.worker.request_zero(axis_id)

    def _on_axis_jog(self, axis_id: int, direction: int, pressed: bool):
        self.worker.request_velocity(axis_id, direction, pressed)

    def _on_axis_reset_clog(self, axis_id: int):
        self.worker.request_reset_clog(axis_id)

    def _on_servo_angle_changed(self, angle: int):
        self.worker.request_servo(angle)

    def _on_emergency_stop(self):
        """一键全局急停"""
        self.worker.request_stop_all()

    def _on_virtual_encoder_turn(self, axis_id: int, delta: int, mode: str):
        """虚拟 EC11 旋钮旋转触发"""
        profile = self.worker.profiles[axis_id]
        if mode == "ANGLE":
            # 角度步进: 1 单位 = 1.0° (10 个 tenths)
            new_tenths = profile.target_angle_tenths + delta * 10
            new_tenths = min(max(new_tenths, MOTORBUS_MIN_ANGLE_TENTHS), MOTORBUS_MAX_ANGLE_TENTHS)
            self.axis_cards[axis_id].slider_angle.set(new_tenths / 10.0)
            self.axis_cards[axis_id].lbl_target_val.configure(text=f"{new_tenths/10.0:+.1f}°")
            self.worker.request_angle(axis_id, new_tenths)
        elif mode == "SPEED":
            new_spd = min(max(profile.speed_rpm + delta * 10, 0), 3000)
            self.axis_cards[axis_id].slider_speed.set(new_spd)
            self.axis_cards[axis_id].lbl_speed_val.configure(text=f"{new_spd} RPM")
            self.worker.set_speed(axis_id, new_spd)
        else:
            new_acc = min(max(profile.accel + delta, 0), 255)
            self.axis_cards[axis_id].slider_accel.set(new_acc)
            self.axis_cards[axis_id].lbl_accel_val.configure(text=str(new_acc))
            self.worker.set_accel(axis_id, new_acc)

    def _on_key_shortcut_trigger(self, key_idx: int):
        """板载 K1~K6 按键动作映射"""
        if key_idx == 2:  # K3: 全局归零
            self.worker.request_zero_all()
        elif key_idx == 3:  # K4: 全局使能
            self.worker.request_enable_all(True)
        elif key_idx == 4:  # K5: 停止首个选中的轴
            self.worker.request_stop(0)
        elif key_idx == 5:  # K6: 单轴置零
            self.worker.request_zero(0)

    def _get_all_current_angles(self) -> Tuple[float, float, float, float]:
        """获取当前 4 轴角度 (度)"""
        return (
            self.worker.states[0].actual_angle_tenths / 10.0,
            self.worker.states[1].actual_angle_tenths / 10.0,
            self.worker.states[2].actual_angle_tenths / 10.0,
            self.worker.states[3].actual_angle_tenths / 10.0,
        )

    def _execute_multi_axis_pose(self, a1: float, a2: float, a3: float, a4: float, speed_rpm: int):
        """执行多轴同步目标姿态"""
        angles = [a1, a2, a3, a4]
        for i in range(MOTORBUS_AXIS_COUNT):
            self.worker.set_speed(i, speed_rpm)
            t_tenths = int(round(angles[i] * 10))
            self.axis_cards[i].slider_angle.set(angles[i])
            self.axis_cards[i].lbl_target_val.configure(text=f"{angles[i]:+.1f}°")
            self.worker.request_angle(i, t_tenths)

    # ==================== UI 主循环状态同步与队列排空 ====================

    def _poll_ui_loop(self):
        """UI 主线程定时轮询器 (40Hz): 安全同步状态、更新 3D 姿态并排空日志/通知队列"""
        try:
            # 1. 刷新四轴卡片与遥测矩阵
            with self.worker.lock:
                for i in range(MOTORBUS_AXIS_COUNT):
                    if i < len(self.axis_cards):
                        self.axis_cards[i].update_state(self.worker.states[i], self.worker.profiles[i])

                self.monitor_panel.update_matrix(self.worker.states, self.worker.profiles)

            # 2. 排空日志队列
            while not self.worker.log_queue.empty():
                log_item = self.worker.log_queue.get_nowait()
                self.monitor_panel.append_log(log_item)

            # 3. 排空通知队列
            while not self.worker.notice_queue.empty():
                n_type, n_msg = self.worker.notice_queue.get_nowait()
                self.header_bar.update_notice(n_type, n_msg)

            # 4. 排空连接状态队列
            while not self.worker.conn_queue.empty():
                conn_state, port, is_sim = self.worker.conn_queue.get_nowait()
                self.header_bar.set_connection_state(conn_state, port, is_sim)

        except Exception:
            pass

        # 调度下一帧
        self.after(25, self._poll_ui_loop)

    def _on_connect_serial(self, port: str, baud: int):
        self.worker.connect(port, baud)

    def _on_disconnect_serial(self):
        self.worker.disconnect()

    def _on_toggle_simulation(self):
        self.worker.start_simulation()

    def _toggle_fullscreen(self):
        self.is_fullscreen = not self.is_fullscreen
        self.attributes("-fullscreen", self.is_fullscreen)

    def _exit_fullscreen(self):
        if self.is_fullscreen:
            self.is_fullscreen = False
            self.attributes("-fullscreen", False)

    def _on_closing(self):
        """退出程序时安全释放串口与线程"""
        self.worker.disconnect()
        self.destroy()


def main():
    app = RoboticArmApp()
    app.mainloop()


if __name__ == "__main__":
    main()

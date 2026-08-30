"""
axis_card.py
--------------------------------------------------------------------------
单轴独立控制卡片组件 (包含仪表盘、多级步进微调、速度/加速度配置及安全动作)
对应固件: OLED.c 单轴显示与控制页 (AXIS 1~4)
"""

import math
import customtkinter as ctk
import tkinter as tk
from typing import Callable, Optional
from ui.theme import Theme
from protocol.emm_v5_protocol import (
    MotorState,
    MotorProfile,
    AXIS_NAMES_EN,
    AXIS_NAMES_CN,
    MOTORBUS_MIN_ANGLE_TENTHS,
    MOTORBUS_MAX_ANGLE_TENTHS
)


class AxisCard(ctk.CTkFrame):
    """四轴机械臂单个电机的全功能独立仪表与控制卡片"""

    def __init__(
        self,
        master,
        axis_id: int,  # 0~3
        on_angle_changed: Callable[[int, int], None],   # (axis, tenths)
        on_speed_changed: Callable[[int, int], None],   # (axis, rpm)
        on_accel_changed: Callable[[int, int], None],   # (axis, acc)
        on_enable_toggle: Callable[[int, bool], None],  # (axis, enable)
        on_stop_axis: Callable[[int], None],            # (axis)
        on_zero_axis: Callable[[int], None],            # (axis)
        on_jog: Callable[[int, int, bool], None],       # (axis, direction, pressed)
        on_reset_clog: Callable[[int], None],           # (axis)
        **kwargs
    ):
        super().__init__(
            master,
            fg_color=Theme.BG_CARD,
            corner_radius=Theme.RADIUS_CARD,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            **kwargs
        )

        self.axis_id = axis_id
        self.on_angle_changed = on_angle_changed
        self.on_speed_changed = on_speed_changed
        self.on_accel_changed = on_accel_changed
        self.on_enable_toggle = on_enable_toggle
        self.on_stop_axis = on_stop_axis
        self.on_zero_axis = on_zero_axis
        self.on_jog = on_jog
        self.on_reset_clog = on_reset_clog

        self.name_en = AXIS_NAMES_EN[axis_id]
        self.name_cn = AXIS_NAMES_CN[axis_id]

        self._curr_target_tenths = 0
        self._curr_speed_rpm = 30
        self._curr_accel = 20

        self._build_ui()

    def _build_ui(self):
        # 垂直布局，紧凑高信息密度
        self.pack_propagate(True)

        # ----------------- 1. 顶部标题与状态标签栏 -----------------
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.pack(fill="x", padx=12, pady=(10, 4))

        # 轴编号与名称
        title_box = ctk.CTkFrame(header, fg_color="transparent")
        title_box.pack(side="left")

        lbl_id = ctk.CTkLabel(
            title_box,
            text=f"AXIS {self.axis_id + 1}",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=14, weight="bold"),
            text_color=Theme.CYAN_ACCENT
        )
        lbl_id.pack(side="left", padx=(0, 6))

        lbl_name = ctk.CTkLabel(
            title_box,
            text=f"{self.name_cn}",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=13, weight="bold"),
            text_color=Theme.TEXT_TITLE
        )
        lbl_name.pack(side="left")

        # 右侧状态微徽章 (ON/OFF, Z/NZ, EN/DS, STALL)
        badges_box = ctk.CTkFrame(header, fg_color="transparent")
        badges_box.pack(side="right")

        self.badge_online = self._create_badge(badges_box, "OFF", Theme.TEXT_MUTED, Theme.BG_INPUT)
        self.badge_zero = self._create_badge(badges_box, "NZ", Theme.ORANGE_WARN, Theme.BG_INPUT)
        self.badge_enabled = self._create_badge(badges_box, "DS", Theme.TEXT_MUTED, Theme.BG_INPUT)
        self.badge_stall = self._create_badge(badges_box, "OK", Theme.GREEN_SUCCESS, Theme.BG_INPUT)

        # 分割线
        sep = ctk.CTkFrame(self, height=1, fg_color=Theme.BORDER_DEFAULT)
        sep.pack(fill="x", padx=12, pady=4)

        # ----------------- 2. 中间：圆形表盘与实时角度数字 -----------------
        meter_frame = ctk.CTkFrame(self, fg_color="transparent")
        meter_frame.pack(fill="x", padx=12, pady=4)

        # 左侧绘制迷你等轴测指针刻度仪表
        self.canvas_meter = tk.Canvas(
            meter_frame,
            width=80,
            height=80,
            bg=Theme.BG_CARD,
            highlightthickness=0
        )
        self.canvas_meter.pack(side="left", padx=(4, 10))

        # 右侧巨大数字显示 (实时角度 + 目标角度 + 实时转速)
        digits_box = ctk.CTkFrame(meter_frame, fg_color="transparent")
        digits_box.pack(side="left", fill="both", expand=True)

        # 实时角度
        row_act = ctk.CTkFrame(digits_box, fg_color="transparent")
        row_act.pack(anchor="w")
        ctk.CTkLabel(
            row_act,
            text="实时角度:",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            text_color=Theme.TEXT_SECONDARY
        ).pack(side="left", padx=(0, 6))

        self.lbl_actual_angle = ctk.CTkLabel(
            row_act,
            text="+0.0°",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=20, weight="bold"),
            text_color=Theme.CYAN_ACCENT
        )
        self.lbl_actual_angle.pack(side="left")

        # 目标角度与实时转速
        row_sub = ctk.CTkFrame(digits_box, fg_color="transparent")
        row_sub.pack(anchor="w", pady=(2, 0))

        ctk.CTkLabel(
            row_sub,
            text="目标:",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10),
            text_color=Theme.TEXT_MUTED
        ).pack(side="left", padx=(0, 2))

        self.lbl_target_val = ctk.CTkLabel(
            row_sub,
            text="+0.0°",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=11, weight="bold"),
            text_color=Theme.TEXT_TITLE
        )
        self.lbl_target_val.pack(side="left", padx=(0, 10))

        ctk.CTkLabel(
            row_sub,
            text="转速:",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10),
            text_color=Theme.TEXT_MUTED
        ).pack(side="left", padx=(0, 2))

        self.lbl_actual_rpm = ctk.CTkLabel(
            row_sub,
            text="0 RPM",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=11, weight="bold"),
            text_color=Theme.GREEN_SUCCESS
        )
        self.lbl_actual_rpm.pack(side="left")

        self._draw_meter(0.0)

        # ----------------- 3. 目标角度滑块与直接输入 -----------------
        slider_box = ctk.CTkFrame(self, fg_color="transparent")
        slider_box.pack(fill="x", padx=12, pady=(4, 2))

        # 滑块
        self.slider_angle = ctk.CTkSlider(
            slider_box,
            from_=-180.0,
            to=180.0,
            number_of_steps=3600,
            height=16,
            progress_color=Theme.CYAN_DARK,
            button_color=Theme.CYAN_ACCENT,
            button_hover_color="#38BDF8",
            corner_radius=Theme.RADIUS_SLIDER,
            command=self._on_slider_drag
        )
        self.slider_angle.set(0.0)
        self.slider_angle.pack(fill="x", pady=(0, 4))

        # ----------------- 4. 按住连续运行按键 -----------------
        steps_frame = ctk.CTkFrame(self, fg_color="transparent")
        steps_frame.pack(fill="x", padx=10, pady=2)
        self.btn_jog_neg = ctk.CTkButton(steps_frame, text="◀ 按住反向", height=26,
                                         font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10))
        self.btn_jog_neg.pack(side="left", padx=2, expand=True)
        self.btn_jog_neg.bind("<ButtonPress-1>", lambda _e: self.on_jog(self.axis_id, 1, True))
        self.btn_jog_neg.bind("<ButtonRelease-1>", lambda _e: self.on_jog(self.axis_id, 1, False))
        self.btn_jog_pos = ctk.CTkButton(steps_frame, text="按住正向 ▶", height=26,
                                         font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10))
        self.btn_jog_pos.pack(side="left", padx=2, expand=True)
        self.btn_jog_pos.bind("<ButtonPress-1>", lambda _e: self.on_jog(self.axis_id, 0, True))
        self.btn_jog_pos.bind("<ButtonRelease-1>", lambda _e: self.on_jog(self.axis_id, 0, False))

        # ----------------- 5. 速度与加速度设定参数栏 -----------------
        param_frame = ctk.CTkFrame(
            self,
            fg_color=Theme.BG_INPUT,
            corner_radius=Theme.RADIUS_SUB_CARD,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT
        )
        param_frame.pack(fill="x", padx=12, pady=6)

        # 速度行
        row_spd = ctk.CTkFrame(param_frame, fg_color="transparent")
        row_spd.pack(fill="x", padx=8, pady=(4, 2))

        ctk.CTkLabel(
            row_spd,
            text="运行速度 (RPM):",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10),
            text_color=Theme.TEXT_SECONDARY
        ).pack(side="left")

        self.lbl_speed_val = ctk.CTkLabel(
            row_spd,
            text="30 RPM",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10, weight="bold"),
            text_color=Theme.CYAN_ACCENT
        )
        self.lbl_speed_val.pack(side="right")

        self.slider_speed = ctk.CTkSlider(
            param_frame,
            from_=1,
            to=3000,
            number_of_steps=300,
            height=12,
            progress_color=Theme.CYAN_DARK,
            button_color=Theme.CYAN_ACCENT,
            corner_radius=Theme.RADIUS_SLIDER,
            command=self._on_speed_slider
        )
        self.slider_speed.set(30)
        self.slider_speed.pack(fill="x", padx=8, pady=(0, 4))

        # 加速度行
        row_acc = ctk.CTkFrame(param_frame, fg_color="transparent")
        row_acc.pack(fill="x", padx=8, pady=(2, 2))

        ctk.CTkLabel(
            row_acc,
            text="加速度 (0~255):",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10),
            text_color=Theme.TEXT_SECONDARY
        ).pack(side="left")

        self.lbl_accel_val = ctk.CTkLabel(
            row_acc,
            text="20",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10, weight="bold"),
            text_color=Theme.CYAN_ACCENT
        )
        self.lbl_accel_val.pack(side="right")

        self.slider_accel = ctk.CTkSlider(
            param_frame,
            from_=0,
            to=255,
            number_of_steps=255,
            height=12,
            progress_color=Theme.PURPLE_ACCENT,
            button_color="#A855F7",
            corner_radius=Theme.RADIUS_SLIDER,
            command=self._on_accel_slider
        )
        self.slider_accel.set(20)
        self.slider_accel.pack(fill="x", padx=8, pady=(0, 4))

        # ----------------- 6. 底部控制动作按键 (使能开关, 急停, 置零) -----------------
        actions_frame = ctk.CTkFrame(self, fg_color="transparent")
        actions_frame.pack(fill="x", padx=12, pady=(4, 10))

        self.switch_enable = ctk.CTkSwitch(
            actions_frame,
            text="使能",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            progress_color=Theme.GREEN_SUCCESS,
            button_color="#FFFFFF",
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_switch_enable
        )
        self.switch_enable.pack(side="left", padx=(0, 4))

        self.btn_zero = ctk.CTkButton(
            actions_frame,
            text="🎯 当前位设零 (K6)",
            width=70,
            height=26,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BADGE,
            command=lambda: self.on_zero_axis(self.axis_id)
        )
        self.btn_zero.pack(side="left", padx=2, expand=True)

        self.btn_stop = ctk.CTkButton(
            actions_frame,
            text="🛑 停止",
            width=60,
            height=26,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10, weight="bold"),
            fg_color=Theme.RED_CRIMSON,
            hover_color=Theme.RED_DANGER,
            corner_radius=Theme.RADIUS_BADGE,
            command=lambda: self.on_stop_axis(self.axis_id)
        )
        self.btn_stop.pack(side="right", padx=(2, 0))

    def _create_badge(self, master, text: str, text_color: str, bg_color: str) -> ctk.CTkLabel:
        badge = ctk.CTkLabel(
            master,
            text=text,
            width=28,
            height=18,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=9, weight="bold"),
            text_color=text_color,
            fg_color=bg_color,
            corner_radius=Theme.RADIUS_BADGE
        )
        badge.pack(side="left", padx=2)
        return badge

    def _draw_meter(self, angle_deg: float):
        """在小 Canvas 上绘制科技感弧形表盘"""
        self.canvas_meter.delete("all")
        cx, cy, r = 40, 40, 32

        # 绘制背景暗色圆弧 (-135° 到 135°)
        self.canvas_meter.create_arc(
            cx - r, cy - r, cx + r, cy + r,
            start=-45, extent=270,
            style="arc",
            outline=Theme.BORDER_DEFAULT,
            width=5
        )

        # 映射角度 (-180°~180°) 到表盘弧度 (-135°~135°)
        ratio = (angle_deg + 180.0) / 360.0
        extent = 270.0 * ratio

        # 绘制发光青色活动圆弧
        self.canvas_meter.create_arc(
            cx - r, cy - r, cx + r, cy + r,
            start=225, extent=-extent,
            style="arc",
            outline=Theme.CYAN_ACCENT,
            width=5
        )

        # 绘制中心指针与圆心
        needle_angle_rad = math.radians(225 - extent)
        nx = cx + (r - 10) * math.cos(needle_angle_rad)
        ny = cy - (r - 10) * math.sin(needle_angle_rad)
        self.canvas_meter.create_line(cx, cy, nx, ny, fill=Theme.CYAN_ACCENT, width=2)
        self.canvas_meter.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=Theme.TEXT_TITLE, outline=Theme.CYAN_ACCENT)

    def _on_slider_drag(self, val: float):
        tenths = int(round(val * 10))
        self._curr_target_tenths = tenths
        self.lbl_target_val.configure(text=f"{val:+.1f}°")
        self.on_angle_changed(self.axis_id, tenths)

    def _on_step_click(self, delta_tenths: int, is_zero: bool):
        if is_zero:
            new_tenths = 0
        else:
            new_tenths = self._curr_target_tenths + delta_tenths

        new_tenths = min(max(new_tenths, MOTORBUS_MIN_ANGLE_TENTHS), MOTORBUS_MAX_ANGLE_TENTHS)
        self._curr_target_tenths = new_tenths
        deg = new_tenths / 10.0
        self.slider_angle.set(deg)
        self.lbl_target_val.configure(text=f"{deg:+.1f}°")
        self.on_angle_changed(self.axis_id, new_tenths)

    def _on_speed_slider(self, val: float):
        rpm = int(val)
        self._curr_speed_rpm = rpm
        self.lbl_speed_val.configure(text=f"{rpm} RPM")
        self.on_speed_changed(self.axis_id, rpm)

    def _on_accel_slider(self, val: float):
        acc = int(val)
        self._curr_accel = acc
        self.lbl_accel_val.configure(text=str(acc) if acc > 0 else "直接启动(0)")
        self.on_accel_changed(self.axis_id, acc)

    def _on_switch_enable(self):
        state = bool(self.switch_enable.get())
        self.on_enable_toggle(self.axis_id, state)

    def update_state(self, state: MotorState, profile: MotorProfile):
        """实时更新电机遥测数据到控件上"""
        act_deg = state.actual_angle_tenths / 10.0
        tgt_deg = profile.target_angle_tenths / 10.0

        # 更新数字与表盘
        self.lbl_actual_angle.configure(text=f"{act_deg:+.1f}°")
        self.lbl_target_val.configure(text=f"{tgt_deg:+.1f}°")
        self.lbl_actual_rpm.configure(text=f"{state.actual_rpm:+d} RPM")
        self._draw_meter(act_deg)

        # 徽章状态更新
        if state.online:
            self.badge_online.configure(text="ON", text_color=Theme.GREEN_SUCCESS)
        else:
            self.badge_online.configure(text="OFF", text_color=Theme.TEXT_MUTED)

        if state.zero_valid:
            self.badge_zero.configure(text="Z", text_color=Theme.GREEN_SUCCESS)
        else:
            self.badge_zero.configure(text="NZ", text_color=Theme.ORANGE_WARN)

        if state.enabled:
            self.badge_enabled.configure(text="EN", text_color=Theme.GREEN_SUCCESS)
            if self.switch_enable.get() == 0:
                self.switch_enable.select()
        else:
            self.badge_enabled.configure(text="DS", text_color=Theme.TEXT_MUTED)
            if self.switch_enable.get() == 1:
                self.switch_enable.deselect()

        if state.stall_warning:
            self.badge_stall.configure(text="STALL", text_color=Theme.RED_DANGER)
            self.configure(border_color=Theme.BORDER_DANGER)
        else:
            self.badge_stall.configure(text="OK", text_color=Theme.GREEN_SUCCESS)
            self.configure(border_color=Theme.BORDER_DEFAULT)

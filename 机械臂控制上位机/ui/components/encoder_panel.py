"""
encoder_panel.py
--------------------------------------------------------------------------
EC11 虚拟旋转编码器与多模式快捷调节面板 (复刻并增强固件 OLED Mode Page 与物理按键)
对应固件: ArmControl.c, Key.c, Encoder.c
"""

import customtkinter as ctk
import tkinter as tk
from typing import Callable, Optional
from ui.theme import Theme
from protocol.emm_v5_protocol import AXIS_NAMES_EN, AXIS_NAMES_CN


class VirtualEncoderPanel(ctk.CTkFrame):
    """虚拟 EC11 编码器旋钮与 OLED 调节模式面板"""

    def __init__(
        self,
        master,
        on_encoder_turn: Callable[[int, int, str], None],  # (axis, delta, mode)
        on_key_trigger: Callable[[int], None],            # (key_index 0~5)
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

        self.on_encoder_turn = on_encoder_turn
        self.on_key_trigger = on_key_trigger

        self.current_mode = "ANGLE"  # "ANGLE", "SPEED", "ACCEL"

        self._build_ui()

    def _build_ui(self):
        # 布局
        self.grid_rowconfigure(0, weight=0)  # 模式切换栏
        self.grid_rowconfigure(1, weight=1)  # 四路虚拟旋钮卡片
        self.grid_rowconfigure(2, weight=0)  # 物理按键 K1~K6 仿真
        self.grid_columnconfigure(0, weight=1)

        # ----------------- 1. 顶部：调节目标模式选择 (复刻 OLED_PAGE_MODE) -----------------
        mode_header = ctk.CTkFrame(self, fg_color="transparent")
        mode_header.grid(row=0, column=0, padx=12, pady=(10, 6), sticky="ew")

        ctk.CTkLabel(
            mode_header,
            text="🎛️ EC11 旋钮调节对象:",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=12, weight="bold"),
            text_color=Theme.TEXT_TITLE
        ).pack(side="left", padx=(0, 10))

        self.seg_mode = ctk.CTkSegmentedButton(
            mode_header,
            values=["📐 目标角度 (Angle)", "⚡ 运行转速 (Speed)", "🚀 加速度 (Accel)"],
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            selected_color=Theme.CYAN_DARK,
            selected_hover_color=Theme.CYAN_ACCENT,
            unselected_color=Theme.BG_INPUT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_mode_selected
        )
        self.seg_mode.set("📐 目标角度 (Angle)")
        self.seg_mode.pack(side="left", fill="x", expand=True)

        # ----------------- 2. 中间：四轴虚拟 EC11 旋钮控制盘 -----------------
        knobs_container = ctk.CTkFrame(self, fg_color="transparent")
        knobs_container.grid(row=1, column=0, padx=10, pady=4, sticky="nsew")
        for i in range(4):
            knobs_container.grid_columnconfigure(i, weight=1)
        knobs_container.grid_rowconfigure(0, weight=1)

        for axis_id in range(4):
            self._create_knob_box(knobs_container, axis_id)

        # ----------------- 3. 底部：固件六个独立按键 (K1~K6) 映射栏 -----------------
        keys_frame = ctk.CTkFrame(
            self,
            fg_color=Theme.BG_INPUT,
            corner_radius=Theme.RADIUS_SUB_CARD,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT
        )
        keys_frame.grid(row=2, column=0, padx=12, pady=(4, 10), sticky="ew")

        ctk.CTkLabel(
            keys_frame,
            text="板载按键快捷响应:",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            text_color=Theme.TEXT_MUTED
        ).pack(side="left", padx=(10, 6), pady=6)

        key_configs = [
            ("K1 减/上翻", 0, Theme.BG_CARD),
            ("K2 加/下翻", 1, Theme.BG_CARD),
            ("K3 全局归零", 2, Theme.CYAN_DARK),
            ("K4 全局使能", 3, Theme.GREEN_DARK),
            ("K5 停止选轴", 4, Theme.RED_CRIMSON),
            ("K6 单轴置零", 5, Theme.PURPLE_ACCENT),
        ]

        for text, key_idx, color in key_configs:
            btn = ctk.CTkButton(
                keys_frame,
                text=text,
                height=26,
                font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10),
                fg_color=color,
                hover_color=Theme.CYAN_ACCENT,
                corner_radius=Theme.RADIUS_BADGE,
                command=lambda k=key_idx: self.on_key_trigger(k)
            )
            btn.pack(side="left", padx=3, expand=True, pady=4)

    def _create_knob_box(self, master, axis_id: int):
        """创建单路旋钮微调卡片"""
        box = ctk.CTkFrame(
            master,
            fg_color=Theme.BG_INPUT,
            corner_radius=Theme.RADIUS_SUB_CARD,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT
        )
        box.grid(row=0, column=axis_id, padx=3, pady=2, sticky="nsew")

        # 轴名称
        ctk.CTkLabel(
            box,
            text=f"EC11-{axis_id+1}\n{AXIS_NAMES_EN[axis_id]}",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10, weight="bold"),
            text_color=Theme.CYAN_ACCENT
        ).pack(pady=(6, 2))

        # 微调按键组
        btn_grid = ctk.CTkFrame(box, fg_color="transparent")
        btn_grid.pack(pady=4)

        # 快速步进
        btn_m10 = ctk.CTkButton(
            btn_grid,
            text="◀◀ -10",
            width=52,
            height=24,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=9),
            fg_color=Theme.BG_CARD,
            hover_color=Theme.BG_CARD_HOVER,
            corner_radius=Theme.RADIUS_BADGE,
            command=lambda: self.on_encoder_turn(axis_id, -10, self.current_mode)
        )
        btn_m10.grid(row=0, column=0, padx=2, pady=2)

        btn_p10 = ctk.CTkButton(
            btn_grid,
            text="+10 ▶▶",
            width=52,
            height=24,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=9),
            fg_color=Theme.BG_CARD,
            hover_color=Theme.BG_CARD_HOVER,
            corner_radius=Theme.RADIUS_BADGE,
            command=lambda: self.on_encoder_turn(axis_id, 10, self.current_mode)
        )
        btn_p10.grid(row=0, column=1, padx=2, pady=2)

        # 细分微步
        btn_m1 = ctk.CTkButton(
            btn_grid,
            text="◀ -1",
            width=52,
            height=24,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=9),
            fg_color=Theme.BG_CARD,
            hover_color=Theme.BG_CARD_HOVER,
            corner_radius=Theme.RADIUS_BADGE,
            command=lambda: self.on_encoder_turn(axis_id, -1, self.current_mode)
        )
        btn_m1.grid(row=1, column=0, padx=2, pady=2)

        btn_p1 = ctk.CTkButton(
            btn_grid,
            text="+1 ▶",
            width=52,
            height=24,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=9),
            fg_color=Theme.BG_CARD,
            hover_color=Theme.BG_CARD_HOVER,
            corner_radius=Theme.RADIUS_BADGE,
            command=lambda: self.on_encoder_turn(axis_id, 1, self.current_mode)
        )
        btn_p1.grid(row=1, column=1, padx=2, pady=2)

    def _on_mode_selected(self, val: str):
        if "角度" in val:
            self.current_mode = "ANGLE"
        elif "转速" in val:
            self.current_mode = "SPEED"
        else:
            self.current_mode = "ACCEL"

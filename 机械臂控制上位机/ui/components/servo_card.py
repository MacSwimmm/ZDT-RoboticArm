import customtkinter as ctk
from typing import Callable
from ui.theme import Theme


class ServoCard(ctk.CTkFrame):
    """MG90S 夹爪舵机角度控制。滑块改变本地值，点击应用才发送。"""
    def __init__(self, master, on_angle_changed: Callable[[int], None], **kwargs):
        super().__init__(master, fg_color=Theme.BG_CARD, corner_radius=Theme.RADIUS_CARD,
                         border_width=1, border_color=Theme.BORDER_DEFAULT, **kwargs)
        self.on_angle_changed = on_angle_changed
        self.angle = 90
        self._build_ui()

    def _build_ui(self):
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.pack(fill="x", padx=14, pady=(12, 4))
        ctk.CTkLabel(header, text="GRIPPER SERVO", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=14, weight="bold"), text_color=Theme.CYAN_ACCENT).pack(side="left")
        ctk.CTkLabel(header, text="MG90S / 180°", font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11), text_color=Theme.TEXT_SECONDARY).pack(side="right")
        self.value_label = ctk.CTkLabel(self, text="90°", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=28, weight="bold"), text_color=Theme.TEXT_TITLE)
        self.value_label.pack(pady=(2, 6))
        self.slider = ctk.CTkSlider(self, from_=0, to=180, number_of_steps=180, command=self._on_slider,
                                    progress_color=Theme.CYAN_DARK, button_color=Theme.CYAN_ACCENT)
        self.slider.set(90)
        self.slider.pack(fill="x", padx=16, pady=(0, 10))
        presets = ctk.CTkFrame(self, fg_color="transparent")
        presets.pack(fill="x", padx=12, pady=(0, 10))
        for label, angle in (("闭合", 0), ("中位", 90), ("打开", 180)):
            ctk.CTkButton(presets, text=label, height=28, command=lambda a=angle: self.set_angle(a)).pack(side="left", expand=True, padx=3)
        ctk.CTkButton(self, text="应用舵机角度", height=30, fg_color=Theme.GREEN_DARK, hover_color=Theme.GREEN_SUCCESS,
                      command=self._apply).pack(fill="x", padx=16, pady=(0, 14))

    def _on_slider(self, value):
        self.angle = int(round(float(value)))
        self.value_label.configure(text=f"{self.angle}°")

    def set_angle(self, angle: int):
        self.slider.set(angle)
        self._on_slider(angle)
        self._apply()

    def _apply(self):
        self.on_angle_changed(self.angle)

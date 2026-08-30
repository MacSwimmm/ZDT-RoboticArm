"""
header_bar.py
--------------------------------------------------------------------------
上位机顶部导航、串口连接管理与全局安全提示栏
"""

import customtkinter as ctk
import tkinter as tk
from typing import Callable, Optional, List, Dict
from ui.theme import Theme
from protocol.emm_v5_protocol import NoticeType, NOTICE_MESSAGES


class HeaderBar(ctk.CTkFrame):
    """顶部标题栏、通信配置与全局安全报警组件"""

    def __init__(
        self,
        master,
        on_connect: Callable[[str, int], None],
        on_disconnect: Callable[[], None],
        on_toggle_sim: Callable[[], None],
        on_emergency_stop: Callable[[], None],
        on_toggle_fullscreen: Callable[[], None],
        on_refresh_ports: Callable[[], List[Dict[str, str]]],
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

        self.on_connect = on_connect
        self.on_disconnect = on_disconnect
        self.on_toggle_sim = on_toggle_sim
        self.on_emergency_stop = on_emergency_stop
        self.on_toggle_fullscreen = on_toggle_fullscreen
        self.on_refresh_ports = on_refresh_ports

        self._is_connected = False
        self._is_sim = True
        self._is_fullscreen = False

        self._build_ui()

    def _build_ui(self):
        # 栅格布局 (3 列: 左LOGO与标题, 中间串口连接与系统状态, 右侧全屏与急停)
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure(2, weight=0)
        self.grid_rowconfigure(0, weight=1)

        # ----------------- 1. 左侧 LOGO 与 标题 -----------------
        left_frame = ctk.CTkFrame(self, fg_color="transparent")
        left_frame.grid(row=0, column=0, padx=(16, 12), pady=10, sticky="w")

        # 发光图标指示器
        self.logo_icon = ctk.CTkLabel(
            left_frame,
            text="⚡",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=22, weight="bold"),
            text_color=Theme.CYAN_ACCENT
        )
        self.logo_icon.pack(side="left", padx=(0, 8))

        title_box = ctk.CTkFrame(left_frame, fg_color="transparent")
        title_box.pack(side="left")

        title_label = ctk.CTkLabel(
            title_box,
            text="FOUR-2 四轴机械臂控制中心",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=17, weight="bold"),
            text_color=Theme.TEXT_TITLE
        )
        title_label.pack(anchor="w")

        sub_title = ctk.CTkLabel(
            title_box,
            text="F407 MASTER CONTROLLER • EMM_V5 BUS PROTOCOL",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10),
            text_color=Theme.TEXT_MUTED
        )
        sub_title.pack(anchor="w")

        # ----------------- 2. 中间：串口与系统提示横幅 -----------------
        center_frame = ctk.CTkFrame(self, fg_color="transparent")
        center_frame.grid(row=0, column=1, padx=10, pady=8, sticky="ew")

        # 上排：串口控制控件
        comm_box = ctk.CTkFrame(center_frame, fg_color="transparent")
        comm_box.pack(anchor="center", pady=(0, 4))

        ctk.CTkLabel(
            comm_box,
            text="通信端口:",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=12),
            text_color=Theme.TEXT_SECONDARY
        ).pack(side="left", padx=(0, 4))

        self.port_combo = ctk.CTkComboBox(
            comm_box,
            width=180,
            height=30,
            values=["自动检测中..."],
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=12),
            dropdown_font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=11),
            fg_color=Theme.BG_INPUT,
            border_color=Theme.BORDER_DEFAULT,
            button_color=Theme.CYAN_DARK,
            corner_radius=Theme.RADIUS_BTN
        )
        self.port_combo.pack(side="left", padx=4)

        self.btn_refresh = ctk.CTkButton(
            comm_box,
            text="🔄 刷新",
            width=65,
            height=30,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=12),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BTN,
            command=self._on_refresh_click
        )
        self.btn_refresh.pack(side="left", padx=4)

        self.baud_combo = ctk.CTkComboBox(
            comm_box,
            width=95,
            height=30,
            values=["115200", "57600", "9600"],
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=12),
            fg_color=Theme.BG_INPUT,
            border_color=Theme.BORDER_DEFAULT,
            button_color=Theme.CYAN_DARK,
            corner_radius=Theme.RADIUS_BTN
        )
        self.baud_combo.set("115200")
        self.baud_combo.pack(side="left", padx=4)

        self.btn_connect = ctk.CTkButton(
            comm_box,
            text="⚡ 连接串口",
            width=90,
            height=30,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=12, weight="bold"),
            fg_color=Theme.CYAN_DARK,
            hover_color=Theme.CYAN_ACCENT,
            text_color="#FFFFFF",
            corner_radius=Theme.RADIUS_BTN,
            command=self._on_connect_toggle
        )
        self.btn_connect.pack(side="left", padx=4)

        self.btn_sim = ctk.CTkButton(
            comm_box,
            text="🌐 离线仿真",
            width=85,
            height=30,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=12),
            fg_color=Theme.PURPLE_ACCENT,
            hover_color="#A855F7",
            corner_radius=Theme.RADIUS_BTN,
            command=self.on_toggle_sim
        )
        self.btn_sim.pack(side="left", padx=4)

        # 下排：实时安全状态/警告横幅 (Notice Banner)
        self.banner_frame = ctk.CTkFrame(
            center_frame,
            fg_color=Theme.BG_INPUT,
            corner_radius=Theme.RADIUS_BADGE,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            height=26
        )
        self.banner_frame.pack(fill="x", padx=10)

        self.lbl_notice_icon = ctk.CTkLabel(
            self.banner_frame,
            text="●",
            font=ctk.CTkFont(size=14),
            text_color=Theme.GREEN_SUCCESS
        )
        self.lbl_notice_icon.pack(side="left", padx=(10, 6))

        self.lbl_notice_text = ctk.CTkLabel(
            self.banner_frame,
            text="系统就绪 (就绪状态，已就绪可执行指令)",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            text_color=Theme.TEXT_SECONDARY
        )
        self.lbl_notice_text.pack(side="left", fill="x", expand=True)

        # ----------------- 3. 右侧：全屏切换 与 一键急停 -----------------
        right_frame = ctk.CTkFrame(self, fg_color="transparent")
        right_frame.grid(row=0, column=2, padx=(10, 16), pady=10, sticky="e")

        self.btn_fullscreen = ctk.CTkButton(
            right_frame,
            text="⛶ 全屏",
            width=70,
            height=34,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=12),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BTN,
            command=self._on_fullscreen_click
        )
        self.btn_fullscreen.pack(side="left", padx=6)

        self.btn_stop_all = ctk.CTkButton(
            right_frame,
            text="🛑 全局急停",
            width=115,
            height=34,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=13, weight="bold"),
            fg_color=Theme.RED_CRIMSON,
            hover_color=Theme.RED_DANGER,
            text_color="#FFFFFF",
            corner_radius=Theme.RADIUS_BTN,
            command=self.on_emergency_stop
        )
        self.btn_stop_all.pack(side="left", padx=4)

        # 初始刷新一次串口
        self._on_refresh_click()

    def _on_refresh_click(self):
        """刷新串口列表"""
        ports = self.on_refresh_ports()
        if ports:
            port_values = [p["desc"] for p in ports]
            self.port_combo.configure(values=port_values)
            self.port_combo.set(port_values[0])
        else:
            self.port_combo.configure(values=["未发现串口"])
            self.port_combo.set("未发现串口")

    def _on_connect_toggle(self):
        """点击连接/断开按钮"""
        if self._is_connected and not self._is_sim:
            self.on_disconnect()
        else:
            sel = self.port_combo.get()
            if not sel or "未发现串口" in sel:
                return
            port_name = sel.split(" ")[0]
            try:
                baud = int(self.baud_combo.get())
            except ValueError:
                baud = 115200
            self.on_connect(port_name, baud)

    def _on_fullscreen_click(self):
        """点击全屏按钮"""
        self._is_fullscreen = not self._is_fullscreen
        self.btn_fullscreen.configure(text="🗗 窗口" if self._is_fullscreen else "⛶ 全屏")
        self.on_toggle_fullscreen()

    def set_connection_state(self, connected: bool, port: str, is_sim: bool):
        """更新通信状态显示"""
        self._is_connected = connected
        self._is_sim = is_sim

        if connected:
            if is_sim:
                self.btn_connect.configure(text="⚡ 连接串口", fg_color=Theme.CYAN_DARK)
                self.btn_sim.configure(text="✓ 仿真中", fg_color=Theme.GREEN_DARK)
                self.logo_icon.configure(text_color=Theme.PURPLE_ACCENT)
            else:
                self.btn_connect.configure(text="🔌 断开连接", fg_color=Theme.RED_CRIMSON)
                self.btn_sim.configure(text="🌐 离线仿真", fg_color=Theme.PURPLE_ACCENT)
                self.logo_icon.configure(text_color=Theme.GREEN_SUCCESS)
        else:
            self.btn_connect.configure(text="⚡ 连接串口", fg_color=Theme.CYAN_DARK)
            self.btn_sim.configure(text="🌐 离线仿真", fg_color=Theme.PURPLE_ACCENT)
            self.logo_icon.configure(text_color=Theme.TEXT_MUTED)

    def update_notice(self, notice: NoticeType, custom_msg: Optional[str] = None):
        """更新顶部状态通知横幅"""
        default_msg, color = NOTICE_MESSAGES.get(notice, ("未知状态", Theme.TEXT_MUTED))
        msg = custom_msg if custom_msg else default_msg
        
        self.lbl_notice_icon.configure(text_color=color)
        self.lbl_notice_text.configure(text=msg, text_color=color if notice != NoticeType.NONE else Theme.TEXT_SECONDARY)
        self.banner_frame.configure(border_color=color if notice in (NoticeType.STALL_ALERT, NoticeType.LIMIT, NoticeType.BUS) else Theme.BORDER_DEFAULT)

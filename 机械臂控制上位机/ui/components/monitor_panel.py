"""
monitor_panel.py
--------------------------------------------------------------------------
四轴机械臂状态总览矩阵、全局控制与实时通讯协议日志监视器
对应固件: OLED_PAGE_HOME 页面总览与 MotorBus 遥测
"""

import time
import customtkinter as ctk
import tkinter as tk
from typing import Callable, List, Optional
from ui.theme import Theme
from comm.serial_worker import CommLogItem
from protocol.emm_v5_protocol import MotorState, MotorProfile, AXIS_NAMES_EN, AXIS_NAMES_CN


class MonitorPanel(ctk.CTkFrame):
    """状态遥测矩阵、全局安全动作与通讯日志面板"""

    def __init__(
        self,
        master,
        on_zero_all: Callable[[], None],
        on_enable_all: Callable[[bool], None],
        on_reset_all_clog: Callable[[], None],
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

        self.on_zero_all = on_zero_all
        self.on_enable_all = on_enable_all
        self.on_reset_all_clog = on_reset_all_clog

        self.log_items: List[CommLogItem] = []
        self.max_log_lines = 150

        self._build_ui()

    def _build_ui(self):
        self.grid_rowconfigure(0, weight=0)  # 顶部标题与全局动作栏
        self.grid_rowconfigure(1, weight=0)  # 遥测矩阵卡片
        self.grid_rowconfigure(2, weight=1)  # 实时通信原始日志
        self.grid_columnconfigure(0, weight=1)

        # ----------------- 1. 顶部：全局安全动作控制按键 -----------------
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.grid(row=0, column=0, padx=12, pady=(10, 4), sticky="ew")

        ctk.CTkLabel(
            header,
            text="📊 状态遥测与全局指令",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=13, weight="bold"),
            text_color=Theme.TEXT_TITLE
        ).pack(side="left")

        actions_box = ctk.CTkFrame(header, fg_color="transparent")
        actions_box.pack(side="right")

        btn_z_all = ctk.CTkButton(
            actions_box,
            text="🎯 一键设零并使能 (K3)",
            width=135,
            height=26,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.CYAN_DARK,
            hover_color=Theme.CYAN_ACCENT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self.on_zero_all
        )
        btn_z_all.pack(side="left", padx=2)

        btn_en_all = ctk.CTkButton(
            actions_box,
            text="⚡ 全部使能",
            width=80,
            height=26,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.GREEN_DARK,
            hover_color=Theme.GREEN_SUCCESS,
            corner_radius=Theme.RADIUS_BADGE,
            command=lambda: self.on_enable_all(True)
        )
        btn_en_all.pack(side="left", padx=2)

        btn_ds_all = ctk.CTkButton(
            actions_box,
            text="🔒 全部失能",
            width=80,
            height=26,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.PURPLE_ACCENT,
            hover_color="#A855F7",
            corner_radius=Theme.RADIUS_BADGE,
            command=lambda: self.on_enable_all(False)
        )
        btn_ds_all.pack(side="left", padx=2)

        btn_clog = ctk.CTkButton(
            actions_box,
            text="🔄 解除堵转",
            width=80,
            height=26,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self.on_reset_all_clog
        )
        btn_clog.pack(side="left", padx=2)

        # ----------------- 2. 中间：四轴状态矩阵表格 -----------------
        matrix_frame = ctk.CTkFrame(
            self,
            fg_color=Theme.BG_INPUT,
            corner_radius=Theme.RADIUS_SUB_CARD,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT
        )
        matrix_frame.grid(row=1, column=0, padx=12, pady=4, sticky="ew")

        # 表头
        headers = ["轴编号", "名称", "在线", "零点", "使能", "实时角度", "目标角度", "转速", "总线电压", "温度", "状态"]
        for c, h in enumerate(headers):
            matrix_frame.grid_columnconfigure(c, weight=1)
            lbl = ctk.CTkLabel(
                matrix_frame,
                text=h,
                font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10, weight="bold"),
                text_color=Theme.CYAN_ACCENT
            )
            lbl.grid(row=0, column=c, padx=4, pady=(6, 2))

        # 4行数据标签
        self.matrix_rows = []
        for r in range(4):
            row_widgets = []
            # 轴ID
            w_id = ctk.CTkLabel(matrix_frame, text=f"ID {r+1}", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10), text_color=Theme.TEXT_MUTED)
            w_id.grid(row=r+1, column=0, padx=2, pady=2)
            row_widgets.append(w_id)

            # 名称
            w_name = ctk.CTkLabel(matrix_frame, text=AXIS_NAMES_EN[r], font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10), text_color=Theme.TEXT_BODY)
            w_name.grid(row=r+1, column=1, padx=2, pady=2)
            row_widgets.append(w_name)

            # 在线
            w_on = ctk.CTkLabel(matrix_frame, text="OFF", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10, weight="bold"), text_color=Theme.TEXT_MUTED)
            w_on.grid(row=r+1, column=2, padx=2, pady=2)
            row_widgets.append(w_on)

            # 零点
            w_z = ctk.CTkLabel(matrix_frame, text="NZ", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10, weight="bold"), text_color=Theme.ORANGE_WARN)
            w_z.grid(row=r+1, column=3, padx=2, pady=2)
            row_widgets.append(w_z)

            # 使能
            w_en = ctk.CTkLabel(matrix_frame, text="DS", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10, weight="bold"), text_color=Theme.TEXT_MUTED)
            w_en.grid(row=r+1, column=4, padx=2, pady=2)
            row_widgets.append(w_en)

            # 实时角度
            w_act = ctk.CTkLabel(matrix_frame, text="+0.0°", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10, weight="bold"), text_color=Theme.CYAN_ACCENT)
            w_act.grid(row=r+1, column=5, padx=2, pady=2)
            row_widgets.append(w_act)

            # 目标角度
            w_tgt = ctk.CTkLabel(matrix_frame, text="+0.0°", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10), text_color=Theme.TEXT_SECONDARY)
            w_tgt.grid(row=r+1, column=6, padx=2, pady=2)
            row_widgets.append(w_tgt)

            # 转速
            w_rpm = ctk.CTkLabel(matrix_frame, text="0 RPM", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10), text_color=Theme.GREEN_SUCCESS)
            w_rpm.grid(row=r+1, column=7, padx=2, pady=2)
            row_widgets.append(w_rpm)

            # 总线电压
            w_v = ctk.CTkLabel(matrix_frame, text="24.0V", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10), text_color=Theme.TEXT_SECONDARY)
            w_v.grid(row=r+1, column=8, padx=2, pady=2)
            row_widgets.append(w_v)

            # 温度
            w_t = ctk.CTkLabel(matrix_frame, text="25°C", font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10), text_color=Theme.TEXT_SECONDARY)
            w_t.grid(row=r+1, column=9, padx=2, pady=2)
            row_widgets.append(w_t)

            # 状态
            w_st = ctk.CTkLabel(matrix_frame, text="正常", font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10), text_color=Theme.GREEN_SUCCESS)
            w_st.grid(row=r+1, column=10, padx=2, pady=(2, 6))
            row_widgets.append(w_st)

            self.matrix_rows.append(row_widgets)

        # ----------------- 3. 底部：通讯协议日志监控框 -----------------
        log_header = ctk.CTkFrame(self, fg_color="transparent")
        log_header.grid(row=2, column=0, padx=12, pady=(6, 2), sticky="ew")

        ctk.CTkLabel(
            log_header,
            text="📡 实时通信帧与遥测日志:",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11, weight="bold"),
            text_color=Theme.TEXT_SECONDARY
        ).pack(side="left")

        btn_clear_log = ctk.CTkButton(
            log_header,
            text="清空日志",
            width=55,
            height=20,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=9),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_clear_log
        )
        btn_clear_log.pack(side="right")

        # 文本框
        self.txt_log = ctk.CTkTextbox(
            self,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=10),
            text_color=Theme.TEXT_BODY,
            fg_color=Theme.BG_INPUT,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_SUB_CARD,
            wrap="none"
        )
        self.txt_log.grid(row=3, column=0, padx=12, pady=(0, 10), sticky="nsew")
        self.grid_rowconfigure(3, weight=1)

    def append_log(self, item: CommLogItem):
        """向日志窗口追加通信帧记录"""
        t_str = time.strftime("%H:%M:%S", time.localtime(item.timestamp))
        hex_str = f"[{item.raw_hex}]" if item.raw_hex else ""
        line = f"[{t_str}] [{item.direction:3s}] {item.content} {hex_str}\n"

        self.txt_log.configure(state="normal")
        self.txt_log.insert("end", line)
        self.txt_log.see("end")
        self.txt_log.configure(state="disabled")

    def _on_clear_log(self):
        self.txt_log.configure(state="normal")
        self.txt_log.delete("1.0", "end")
        self.txt_log.configure(state="disabled")

    def update_matrix(self, states: List[MotorState], profiles: List[MotorProfile]):
        """刷新四轴遥测矩阵数据"""
        for i in range(min(4, len(states))):
            st = states[i]
            pf = profiles[i]
            widgets = self.matrix_rows[i]

            # 在线
            widgets[2].configure(
                text="ON" if st.online else "OFF",
                text_color=Theme.GREEN_SUCCESS if st.online else Theme.TEXT_MUTED
            )

            # 零点
            widgets[3].configure(
                text="Z" if st.zero_valid else "NZ",
                text_color=Theme.GREEN_SUCCESS if st.zero_valid else Theme.ORANGE_WARN
            )

            # 使能
            widgets[4].configure(
                text="EN" if st.enabled else "DS",
                text_color=Theme.GREEN_SUCCESS if st.enabled else Theme.TEXT_MUTED
            )

            # 实时/目标角度
            widgets[5].configure(text=f"{st.actual_angle_tenths / 10.0:+.1f}°")
            widgets[6].configure(text=f"{pf.target_angle_tenths / 10.0:+.1f}°")

            # 转速
            widgets[7].configure(text=f"{st.actual_rpm:+d} RPM")

            # 电压/温度
            widgets[8].configure(text=f"{st.bus_voltage:.1f}V")
            widgets[9].configure(text=f"{st.temperature:.0f}°C")

            # 状态
            if st.stall_warning:
                widgets[10].configure(text="堵转警告", text_color=Theme.RED_DANGER)
            elif st.stopped:
                widgets[10].configure(text="急停锁定", text_color=Theme.ORANGE_WARN)
            elif not st.zero_valid:
                widgets[10].configure(text="待归零", text_color=Theme.ORANGE_WARN)
            else:
                widgets[10].configure(text="正常就绪", text_color=Theme.GREEN_SUCCESS)

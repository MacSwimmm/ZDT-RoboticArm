"""
trajectory_panel.py
--------------------------------------------------------------------------
四轴机械臂示教再现 (Teach & Repeat) 与轨迹规划执行面板
支持多路点实时记录、单步/全自动循迹、循环播放与轨迹文件导入导出 (JSON)
"""

import json
import time
import threading
import customtkinter as ctk
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from dataclasses import dataclass, asdict
from typing import List, Dict, Callable, Optional, Tuple
from ui.theme import Theme


@dataclass
class Waypoint:
    """机械臂空间路点数据结构"""
    name: str
    axis1_deg: float  # 丝杆
    axis2_deg: float  # 底座
    axis3_deg: float  # 大臂
    axis4_deg: float  # 小臂
    speed_rpm: int = 30
    delay_ms: int = 500


class TrajectoryPanel(ctk.CTkFrame):
    """机械臂示教再现与轨迹序列规划面板"""

    def __init__(
        self,
        master,
        get_current_angles: Callable[[], Tuple[float, float, float, float]],
        execute_pose: Callable[[float, float, float, float, int], None],
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

        self.get_current_angles = get_current_angles
        self.execute_pose = execute_pose

        self.waypoints: List[Waypoint] = []
        self.is_playing = False
        self.is_paused = False
        self.loop_mode = False
        self.current_step_idx = -1
        self.play_thread: Optional[threading.Thread] = None

        self._build_ui()
        self._load_demo_trajectory()

    def _build_ui(self):
        self.grid_rowconfigure(0, weight=0)  # 顶部标题栏
        self.grid_rowconfigure(1, weight=1)  # 中间路点表格
        self.grid_rowconfigure(2, weight=0)  # 底部控制栏
        self.grid_columnconfigure(0, weight=1)

        # ----------------- 1. 顶部标题与路点编辑操作 -----------------
        header = ctk.CTkFrame(self, fg_color="transparent")
        header.grid(row=0, column=0, padx=12, pady=(10, 4), sticky="ew")

        ctk.CTkLabel(
            header,
            text="🎬 示教再现与轨迹规划",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=13, weight="bold"),
            text_color=Theme.TEXT_TITLE
        ).pack(side="left")

        # 快捷按钮
        btn_box = ctk.CTkFrame(header, fg_color="transparent")
        btn_box.pack(side="right")

        self.btn_record = ctk.CTkButton(
            btn_box,
            text="➕ 记录当前姿态",
            width=95,
            height=26,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11, weight="bold"),
            fg_color=Theme.CYAN_DARK,
            hover_color=Theme.CYAN_ACCENT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_record_current
        )
        self.btn_record.pack(side="left", padx=2)

        btn_del = ctk.CTkButton(
            btn_box,
            text="🗑️ 删除",
            width=50,
            height=26,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_delete_selected
        )
        btn_del.pack(side="left", padx=2)

        btn_clear = ctk.CTkButton(
            btn_box,
            text="🧹 清空",
            width=50,
            height=26,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_clear_all
        )
        btn_clear.pack(side="left", padx=2)

        # ----------------- 2. 中间：路点数据表格 (Treeview) -----------------
        table_container = ctk.CTkFrame(self, fg_color=Theme.BG_INPUT, corner_radius=Theme.RADIUS_SUB_CARD)
        table_container.grid(row=1, column=0, padx=12, pady=4, sticky="nsew")
        table_container.grid_rowconfigure(0, weight=1)
        table_container.grid_columnconfigure(0, weight=1)

        # 配置 Treeview 样式
        style = ttk.Style()
        style.theme_use("default")
        style.configure(
            "Custom.Treeview",
            background=Theme.BG_INPUT,
            foreground=Theme.TEXT_BODY,
            fieldbackground=Theme.BG_INPUT,
            rowheight=24,
            font=(Theme.FONT_FAMILY_MONO, 10),
            borderwidth=0
        )
        style.configure(
            "Custom.Treeview.Heading",
            background=Theme.BG_CARD,
            foreground=Theme.CYAN_ACCENT,
            font=(Theme.FONT_FAMILY_CN, 10, "bold"),
            borderwidth=1,
            relief="flat"
        )
        style.map("Custom.Treeview", background=[("selected", Theme.CYAN_DARK)])

        columns = ("idx", "name", "a1", "a2", "a3", "a4", "speed", "delay")
        self.tree = ttk.Treeview(
            table_container,
            columns=columns,
            show="headings",
            style="Custom.Treeview",
            selectmode="browse"
        )

        col_widths = {
            "idx": 36, "name": 90, "a1": 65, "a2": 65, "a3": 65, "a4": 65, "speed": 60, "delay": 60
        }
        col_titles = {
            "idx": "序号", "name": "路点名称", "a1": "J1 丝杆", "a2": "J2 底座",
            "a3": "J3 大臂", "a4": "J4 小臂", "speed": "速度(RPM)", "delay": "停顿(ms)"
        }

        for col in columns:
            self.tree.heading(col, text=col_titles[col])
            self.tree.column(col, width=col_widths[col], anchor="center")

        # 垂直滚动条
        scrollbar = ttk.Scrollbar(table_container, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)

        self.tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        # 双击直接跳转执行该路点
        self.tree.bind("<Double-1>", lambda e: self._on_double_click_row())

        # ----------------- 3. 底部：轨迹播放控制与文件操作 -----------------
        footer = ctk.CTkFrame(self, fg_color="transparent")
        footer.grid(row=2, column=0, padx=12, pady=(4, 10), sticky="ew")

        # 左侧控制按钮 (播放, 暂停, 停止, 循环)
        play_box = ctk.CTkFrame(footer, fg_color="transparent")
        play_box.pack(side="left")

        self.btn_run = ctk.CTkButton(
            play_box,
            text="▶️ 执行轨迹",
            width=85,
            height=28,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11, weight="bold"),
            fg_color=Theme.GREEN_DARK,
            hover_color=Theme.GREEN_SUCCESS,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_run_trajectory
        )
        self.btn_run.pack(side="left", padx=2)

        self.btn_step = ctk.CTkButton(
            play_box,
            text="⏭️ 单步",
            width=55,
            height=28,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_step_next
        )
        self.btn_step.pack(side="left", padx=2)

        self.btn_stop_play = ctk.CTkButton(
            play_box,
            text="⏹️ 停止",
            width=55,
            height=28,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.RED_CRIMSON,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_stop_trajectory
        )
        self.btn_stop_play.pack(side="left", padx=2)

        self.switch_loop = ctk.CTkSwitch(
            play_box,
            text="循环",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            progress_color=Theme.CYAN_DARK,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_toggle_loop
        )
        self.switch_loop.pack(side="left", padx=6)

        # 右侧导入导出文件按钮
        file_box = ctk.CTkFrame(footer, fg_color="transparent")
        file_box.pack(side="right")

        btn_import = ctk.CTkButton(
            file_box,
            text="📂 导入",
            width=55,
            height=28,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_import_file
        )
        btn_import.pack(side="left", padx=2)

        btn_export = ctk.CTkButton(
            file_box,
            text="💾 导出",
            width=55,
            height=28,
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=11),
            fg_color=Theme.BG_INPUT,
            hover_color=Theme.BG_CARD_HOVER,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            corner_radius=Theme.RADIUS_BADGE,
            command=self._on_export_file
        )
        btn_export.pack(side="left", padx=2)

    def _refresh_table(self):
        """刷新路点列表表格"""
        for item in self.tree.get_children():
            self.tree.delete(item)

        for i, wp in enumerate(self.waypoints):
            tag = "current" if i == self.current_step_idx else "normal"
            self.tree.insert(
                "",
                "end",
                values=(
                    f"P{i+1:02d}",
                    wp.name,
                    f"{wp.axis1_deg:+.1f}°",
                    f"{wp.axis2_deg:+.1f}°",
                    f"{wp.axis3_deg:+.1f}°",
                    f"{wp.axis4_deg:+.1f}°",
                    f"{wp.speed_rpm}",
                    f"{wp.delay_ms}"
                ),
                tags=(tag,)
            )

    def _on_record_current(self):
        """记录当前机械臂姿态为新路点"""
        a1, a2, a3, a4 = self.get_current_angles()
        wp_name = f"路点_{len(self.waypoints) + 1}"
        wp = Waypoint(
            name=wp_name,
            axis1_deg=a1,
            axis2_deg=a2,
            axis3_deg=a3,
            axis4_deg=a4,
            speed_rpm=30,
            delay_ms=600
        )
        self.waypoints.append(wp)
        self._refresh_table()

    def _on_delete_selected(self):
        selected = self.tree.selection()
        if not selected:
            return
        item_idx = self.tree.index(selected[0])
        if 0 <= item_idx < len(self.waypoints):
            self.waypoints.pop(item_idx)
            self._refresh_table()

    def _on_clear_all(self):
        self.waypoints.clear()
        self._refresh_table()

    def _on_double_click_row(self):
        selected = self.tree.selection()
        if not selected:
            return
        idx = self.tree.index(selected[0])
        if 0 <= idx < len(self.waypoints):
            wp = self.waypoints[idx]
            self.execute_pose(wp.axis1_deg, wp.axis2_deg, wp.axis3_deg, wp.axis4_deg, wp.speed_rpm)

    def _on_run_trajectory(self):
        """启动轨迹自动循迹线程"""
        if not self.waypoints:
            return
        if self.is_playing:
            return

        self.is_playing = True
        self.is_paused = False
        self.btn_run.configure(text="⏸️ 暂停", fg_color=Theme.ORANGE_WARN)
        
        self.play_thread = threading.Thread(target=self._trajectory_runner_loop, daemon=True)
        self.play_thread.start()

    def _on_stop_trajectory(self):
        """停止轨迹运行"""
        self.is_playing = False
        self.is_paused = False
        self.current_step_idx = -1
        self.btn_run.configure(text="▶️ 执行轨迹", fg_color=Theme.GREEN_DARK)
        self._refresh_table()

    def _on_step_next(self):
        """单步执行下一个路点"""
        if not self.waypoints:
            return
        self.current_step_idx = (self.current_step_idx + 1) % len(self.waypoints)
        wp = self.waypoints[self.current_step_idx]
        self.execute_pose(wp.axis1_deg, wp.axis2_deg, wp.axis3_deg, wp.axis4_deg, wp.speed_rpm)
        self._refresh_table()

    def _on_toggle_loop(self):
        self.loop_mode = bool(self.switch_loop.get())

    def _trajectory_runner_loop(self):
        """轨迹循迹后台工作线程"""
        while self.is_playing:
            for idx, wp in enumerate(self.waypoints):
                if not self.is_playing:
                    break

                self.current_step_idx = idx
                self.after(0, self._refresh_table)

                # 下发指令到机械臂
                self.execute_pose(wp.axis1_deg, wp.axis2_deg, wp.axis3_deg, wp.axis4_deg, wp.speed_rpm)

                # 估算运动到达时间 + 停留延时
                time.sleep(max(wp.delay_ms / 1000.0, 0.8))

            if not self.loop_mode:
                break

        self.is_playing = False
        self.after(0, lambda: self.btn_run.configure(text="▶️ 执行轨迹", fg_color=Theme.GREEN_DARK))
        self.after(0, self._refresh_table)

    def _load_demo_trajectory(self):
        """加载出厂预设示教动作"""
        self.waypoints = [
            Waypoint(name="原点待机", axis1_deg=0.0, axis2_deg=0.0, axis3_deg=0.0, axis4_deg=0.0, speed_rpm=30, delay_ms=500),
            Waypoint(name="前倾抓取位", axis1_deg=50.0, axis2_deg=0.0, axis3_deg=35.0, axis4_deg=-45.0, speed_rpm=40, delay_ms=800),
            Waypoint(name="抬起过渡位", axis1_deg=50.0, axis2_deg=45.0, axis3_deg=-15.0, axis4_deg=20.0, speed_rpm=40, delay_ms=600),
            Waypoint(name="侧方放置位", axis1_deg=10.0, axis2_deg=90.0, axis3_deg=30.0, axis4_deg=-40.0, speed_rpm=35, delay_ms=800),
            Waypoint(name="返回待机", axis1_deg=0.0, axis2_deg=0.0, axis3_deg=0.0, axis4_deg=0.0, speed_rpm=30, delay_ms=500),
        ]
        self._refresh_table()

    def _on_export_file(self):
        """导出轨迹到 JSON 文件"""
        path = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("Robotic Arm Trajectory", "*.json"), ("All Files", "*.*")]
        )
        if not path:
            return
        data = [asdict(wp) for wp in self.waypoints]
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def _on_import_file(self):
        """从 JSON 文件导入轨迹"""
        path = filedialog.askopenfilename(
            filetypes=[("Robotic Arm Trajectory", "*.json"), ("All Files", "*.*")]
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.waypoints = [Waypoint(**item) for item in data]
            self._refresh_table()
        except Exception as e:
            messagebox.showerror("导入错误", f"解析轨迹文件失败: {str(e)}")

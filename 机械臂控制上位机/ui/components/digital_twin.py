"""
digital_twin.py
--------------------------------------------------------------------------
四轴机械臂 3D/2D 实时数字孪生与正向运动学 (FK) 可视化画布
基于 Tkinter Canvas 高性能矢量渲染，具备发光连杆、自由视角旋转与空间位姿 HUD
"""

import math
import customtkinter as ctk
import tkinter as tk
from typing import List, Tuple, Optional
from ui.theme import Theme
from kinematics.arm_model import ArmKinematics, JointPose3D, EndEffectorState


class DigitalTwinView(ctk.CTkFrame):
    """三维机械臂数字孪生实时姿态交互可视化面板"""

    def __init__(self, master, kinematics: Optional[ArmKinematics] = None, **kwargs):
        super().__init__(
            master,
            fg_color=Theme.BG_CARD,
            corner_radius=Theme.RADIUS_CARD,
            border_width=1,
            border_color=Theme.BORDER_DEFAULT,
            **kwargs
        )

        self.kinematics = kinematics if kinematics else ArmKinematics()

        # 视角与投影控制参数
        self.view_yaw = -40.0       # 水平视角角 (度)
        self.view_pitch = 25.0      # 俯仰视角角 (度)
        self.view_scale = 1.0       # 缩放比例
        self.offset_x = 0.0         # 视图平移
        self.offset_y = 30.0

        # 鼠标拖拽交互状态
        self._last_mouse_x = 0
        self._last_mouse_y = 0
        self._is_dragging = False

        # 机械臂当前关节角度
        self.angles = [0.0, 0.0, 0.0, 0.0]  # [丝杆, 底座, 大臂, 小臂]
        self.end_effector = EndEffectorState(0, 0, 0, 0, 0, 0)

        # 轨迹尾迹历史点
        self.trajectory_trail: List[Tuple[float, float, float]] = []
        self.max_trail_length = 80

        self._build_ui()

    def _build_ui(self):
        # 栅格配置
        self.grid_rowconfigure(0, weight=0)  # 顶部工具栏
        self.grid_rowconfigure(1, weight=1)  # Canvas 主画布
        self.grid_columnconfigure(0, weight=1)

        # ----------------- 1. 顶部 HUD 状态栏与视角快捷键 -----------------
        top_bar = ctk.CTkFrame(self, fg_color="transparent")
        top_bar.grid(row=0, column=0, padx=12, pady=(10, 4), sticky="ew")

        # 标题与坐标 HUD
        title_box = ctk.CTkFrame(top_bar, fg_color="transparent")
        title_box.pack(side="left")

        ctk.CTkLabel(
            title_box,
            text="🧊 3D 数字孪生姿态",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=13, weight="bold"),
            text_color=Theme.TEXT_TITLE
        ).pack(side="left", padx=(0, 10))

        self.lbl_hud_coords = ctk.CTkLabel(
            title_box,
            text="X: 0.0 mm | Y: 0.0 mm | Z: 0.0 mm | R: 0.0 mm",
            font=ctk.CTkFont(family=Theme.FONT_FAMILY_MONO, size=11),
            text_color=Theme.CYAN_ACCENT
        )
        self.lbl_hud_coords.pack(side="left")

        # 右侧视角按钮
        view_btns_box = ctk.CTkFrame(top_bar, fg_color="transparent")
        view_btns_box.pack(side="right")

        preset_views = [
            ("3D 视角", -40.0, 25.0),
            ("俯视", 0.0, 90.0),
            ("主视", 0.0, 0.0),
            ("侧视", -90.0, 0.0),
            ("复位", -40.0, 25.0),
        ]

        for text, y_ang, p_ang in preset_views:
            btn = ctk.CTkButton(
                view_btns_box,
                text=text,
                width=50,
                height=24,
                font=ctk.CTkFont(family=Theme.FONT_FAMILY_CN, size=10),
                fg_color=Theme.BG_INPUT,
                hover_color=Theme.BG_CARD_HOVER,
                border_width=1,
                border_color=Theme.BORDER_DEFAULT,
                corner_radius=Theme.RADIUS_BADGE,
                command=lambda y=y_ang, p=p_ang, t=text: self._set_preset_view(y, p, t == "复位")
            )
            btn.pack(side="left", padx=2)

        # ----------------- 2. 主 3D 渲染 Canvas 画布 -----------------
        canvas_container = ctk.CTkFrame(self, fg_color=Theme.BG_MAIN, corner_radius=Theme.RADIUS_SUB_CARD)
        canvas_container.grid(row=1, column=0, padx=12, pady=(4, 12), sticky="nsew")
        canvas_container.grid_rowconfigure(0, weight=1)
        canvas_container.grid_columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(
            canvas_container,
            bg=Theme.BG_MAIN,
            highlightthickness=0,
            cursor="crosshair"
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")

        # 绑定鼠标交互
        self.canvas.bind("<ButtonPress-1>", self._on_mouse_down)
        self.canvas.bind("<B1-Motion>", self._on_mouse_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_mouse_up)
        self.canvas.bind("<MouseWheel>", self._on_mouse_wheel)
        self.canvas.bind("<Configure>", lambda e: self.render())

    def _set_preset_view(self, yaw: float, pitch: float, reset_zoom: bool = False):
        self.view_yaw = yaw
        self.view_pitch = pitch
        if reset_zoom:
            self.view_scale = 1.0
            self.offset_x = 0.0
            self.offset_y = 30.0
        self.render()

    def _on_mouse_down(self, event):
        self._last_mouse_x = event.x
        self._last_mouse_y = event.y
        self._is_dragging = True

    def _on_mouse_drag(self, event):
        if not self._is_dragging:
            return
        dx = event.x - self._last_mouse_x
        dy = event.y - self._last_mouse_y
        self._last_mouse_x = event.x
        self._last_mouse_y = event.y

        # 转换拖拽增量为视角旋转
        self.view_yaw += dx * 0.6
        self.view_pitch = min(max(self.view_pitch - dy * 0.6, -89.0), 89.0)
        self.render()

    def _on_mouse_up(self, event):
        self._is_dragging = False

    def _on_mouse_wheel(self, event):
        # 滚轮缩放
        if event.delta > 0:
            self.view_scale = min(self.view_scale * 1.1, 3.0)
        else:
            self.view_scale = max(self.view_scale * 0.9, 0.3)
        self.render()

    def update_angles(self, a1_deg: float, a2_deg: float, a3_deg: float, a4_deg: float):
        """更新四轴角度并重新渲染"""
        self.angles = [a1_deg, a2_deg, a3_deg, a4_deg]
        self.render()

    def clear_trail(self):
        """清空轨迹尾迹"""
        self.trajectory_trail.clear()
        self.render()

    def render(self):
        """执行完整 3D 场景矢量投影与渲染"""
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()
        if width <= 10 or height <= 10:
            return

        self.canvas.delete("all")

        # 1. 计算正向运动学关节位置
        joints_3d, end_state = self.kinematics.forward_kinematics(
            self.angles[0], self.angles[1], self.angles[2], self.angles[3]
        )
        self.end_effector = end_state

        # 更新顶部 HUD 坐标
        self.lbl_hud_coords.configure(
            text=f"末端笛卡尔坐标:  X: {end_state.x:+6.1f} mm  |  Y: {end_state.y:+6.1f} mm  |  Z: {end_state.z:+6.1f} mm  |  半径 R: {end_state.reach_radius:5.1f} mm"
        )

        # 记录轨迹点
        self.trajectory_trail.append((end_state.x, end_state.y, end_state.z))
        if len(self.trajectory_trail) > self.max_trail_length:
            self.trajectory_trail.pop(0)

        # 2. 渲染地面参考网格与滑台导轨
        self._render_ground_grid(width, height)

        # 3. 渲染运动轨迹尾迹
        self._render_trajectory_trail(width, height)

        # 4. 投影关节 3D 点到 2D Canvas
        pts_2d = ArmKinematics.project_3d_to_2d(
            joints_3d, width, height,
            self.view_yaw, self.view_pitch,
            self.view_scale, self.offset_x, self.offset_y
        )

        # 5. 绘制连杆与关节 (自底向上)
        p0, p1, p2, p3, p4 = pts_2d[0], pts_2d[1], pts_2d[2], pts_2d[3], pts_2d[4]

        # 连杆 0: 滑台到肩部 (基座柱)
        self._draw_cylinder_link(p0, p1, color="#334155", width_px=14, border_color=Theme.CYAN_DARK)

        # 连杆 1: 大臂 (肩到肘)
        self._draw_glowing_arm_link(p1, p2, color="#0284C7", glow_color=Theme.CYAN_ACCENT, width_px=10)

        # 连杆 2: 小臂 (肘到腕)
        self._draw_glowing_arm_link(p2, p3, color="#7C3AED", glow_color="#C084FC", width_px=8)

        # 连杆 3: 手爪连杆 (腕到指尖)
        self._draw_glowing_arm_link(p3, p4, color="#059669", glow_color=Theme.GREEN_SUCCESS, width_px=5)

        # 绘制各个关节发光转轴圆盘
        self._draw_joint_node(p0, r=7, color="#475569", label="滑块")
        self._draw_joint_node(p1, r=9, color=Theme.CYAN_ACCENT, label="基座(J1)")
        self._draw_joint_node(p2, r=8, color="#A855F7", label="大臂(J2)")
        self._draw_joint_node(p3, r=7, color=Theme.GREEN_SUCCESS, label="小臂(J3)")
        self._draw_joint_node(p4, r=6, color="#F43F5E", label="末端")

        # 绘制手爪夹爪示意
        self._draw_gripper_jaws(p3, p4)

    def _render_ground_grid(self, w: int, h: int):
        """渲染地面透视网格与导轨"""
        grid_pts = []
        grid_size = 200
        step = 50

        # 网格横线与竖线
        for x in range(-grid_size, grid_size + 1, step):
            p_start = JointPose3D(x=float(x), y=float(-grid_size), z=0.0)
            p_end = JointPose3D(x=float(x), y=float(grid_size), z=0.0)
            proj = ArmKinematics.project_3d_to_2d(
                [p_start, p_end], w, h, self.view_yaw, self.view_pitch,
                self.view_scale, self.offset_x, self.offset_y
            )
            self.canvas.create_line(proj[0][0], proj[0][1], proj[1][0], proj[1][1], fill="#1E293B", width=1)

        for y in range(-grid_size, grid_size + 1, step):
            p_start = JointPose3D(x=float(-grid_size), y=float(y), z=0.0)
            p_end = JointPose3D(x=float(grid_size), y=float(y), z=0.0)
            proj = ArmKinematics.project_3d_to_2d(
                [p_start, p_end], w, h, self.view_yaw, self.view_pitch,
                self.view_scale, self.offset_x, self.offset_y
            )
            self.canvas.create_line(proj[0][0], proj[0][1], proj[1][0], proj[1][1], fill="#1E293B", width=1)

        # 丝杆导轨主体 (加粗高亮线)
        rail_start = JointPose3D(x=-120.0, y=0.0, z=0.0)
        rail_end = JointPose3D(x=120.0, y=0.0, z=0.0)
        rail_proj = ArmKinematics.project_3d_to_2d(
            [rail_start, rail_end], w, h, self.view_yaw, self.view_pitch,
            self.view_scale, self.offset_x, self.offset_y
        )
        self.canvas.create_line(
            rail_proj[0][0], rail_proj[0][1], rail_proj[1][0], rail_proj[1][1],
            fill="#334155", width=4
        )

    def _render_trajectory_trail(self, w: int, h: int):
        """渲染历史末端运动轨迹虚线"""
        if len(self.trajectory_trail) < 2:
            return

        trail_3d = [JointPose3D(x=p[0], y=p[1], z=p[2]) for p in self.trajectory_trail]
        trail_2d = ArmKinematics.project_3d_to_2d(
            trail_3d, w, h, self.view_yaw, self.view_pitch,
            self.view_scale, self.offset_x, self.offset_y
        )

        for i in range(len(trail_2d) - 1):
            p_a = trail_2d[i]
            p_b = trail_2d[i + 1]
            self.canvas.create_line(
                p_a[0], p_a[1], p_b[0], p_b[1],
                fill=Theme.CYAN_ACCENT, width=1, dash=(2, 2)
            )

    def _draw_cylinder_link(self, p1, p2, color: str, width_px: int, border_color: str):
        """绘制圆柱连杆基座"""
        self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=border_color, width=width_px + 2, capstyle="round")
        self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=color, width=width_px, capstyle="round")

    def _draw_glowing_arm_link(self, p1, p2, color: str, glow_color: str, width_px: int):
        """绘制发光机械臂连杆"""
        # 外发光辉光
        self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=glow_color, width=width_px + 4, capstyle="round")
        # 核心主干
        self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=color, width=width_px, capstyle="round")
        # 中心高光亮线
        self.canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill="#FFFFFF", width=max(width_px // 4, 1), capstyle="round")

    def _draw_joint_node(self, p, r: int, color: str, label: str = ""):
        """绘制发光关节节点"""
        x, y = p[0], p[1]
        self.canvas.create_oval(x - r - 2, y - r - 2, x + r + 2, y + r + 2, fill=color, outline="")
        self.canvas.create_oval(x - r, y - r, x + r, y + r, fill="#0F172A", outline=color, width=2)
        self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="#FFFFFF", outline="")

    def _draw_gripper_jaws(self, p_wrist, p_tip):
        """绘制末端手爪开合形态"""
        x1, y1 = p_wrist[0], p_wrist[1]
        x2, y2 = p_tip[0], p_tip[1]

        # 计算垂直于连杆的法向量
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length < 1.0:
            return

        nx = -dy / length * 6.0
        ny = dx / length * 6.0

        # 两片夹爪
        self.canvas.create_line(x2, y2, x2 + nx + dx * 0.2, y2 + ny + dy * 0.2, fill=Theme.GREEN_SUCCESS, width=2)
        self.canvas.create_line(x2, y2, x2 - nx + dx * 0.2, y2 - ny + dy * 0.2, fill=Theme.GREEN_SUCCESS, width=2)

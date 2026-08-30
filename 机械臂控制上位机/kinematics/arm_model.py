"""
arm_model.py
--------------------------------------------------------------------------
四轴机械臂正向运动学 (Forward Kinematics) 与三维几何空间数字孪生计算
对应结构: Structure/【FOUR-2】sw文件

结构参数:
- 丝杆移动行程: 0 ~ 200 mm (对应 Axis 1 丝杆电机角度转换)
- 底座旋转关节: Axis 2 (Yaw 角: -180° ~ +180°)
- 大臂俯仰关节: Axis 3 (Pitch1 角: -90° ~ +90°, 杆长 L1 = 140 mm)
- 小臂俯仰关节: Axis 4 (Pitch2 角: -120° ~ +120°, 杆长 L2 = 145 mm)
- 末端手爪组件: 长度 L3 = 60 mm
"""

import math
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any


@dataclass
class JointPose3D:
    """机械臂空间各关节三维笛卡尔坐标 (单位: mm)"""
    x: float
    y: float
    z: float


@dataclass
class EndEffectorState:
    """末端执行器位姿"""
    x: float
    y: float
    z: float
    reach_radius: float
    pitch_angle: float
    yaw_angle: float


class ArmKinematics:
    """四轴机械臂正向运动学与三维透视投影计算引擎"""

    def __init__(
        self,
        slide_lead_mm: float = 8.0,      # 丝杆导程: 8mm/圈 (360° -> 8mm)
        base_height_mm: float = 85.0,    # 基座固定高度
        l1_upper_mm: float = 140.0,      # 大臂连杆长度
        l2_forearm_mm: float = 145.0,    # 小臂连杆长度
        l3_gripper_mm: float = 65.0      # 末端手爪长度
    ):
        self.slide_lead = slide_lead_mm
        self.base_height = base_height_mm
        self.l1 = l1_upper_mm
        self.l2 = l2_forearm_mm
        self.l3 = l3_gripper_mm

    def forward_kinematics(
        self,
        axis1_deg: float,  # 丝杆轴
        axis2_deg: float,  # 底座偏航角 (Yaw)
        axis3_deg: float,  # 大臂俯仰角 (Pitch1)
        axis4_deg: float   # 小臂俯仰角 (Pitch2)
    ) -> Tuple[List[JointPose3D], EndEffectorState]:
        """
        根据四个关节角度计算各个关节的三维笛卡尔坐标序列及末端位置
        返回值:
          points: [滑块基座, 旋转肩部, 肘部关节, 腕部关节, 手爪末端]
          end_effector: 末端详细空间状态
        """
        # 1. 丝杆滑台位移计算 (X方向水平导轨移动)
        slide_x = (axis1_deg / 360.0) * self.slide_lead

        # 2. 角度换算为弧度
        rad_yaw = math.radians(axis2_deg)
        rad_pitch1 = math.radians(axis3_deg)
        rad_pitch2 = math.radians(axis4_deg)

        # 3. 关节 0: 滑块固定导轨点
        p0 = JointPose3D(x=slide_x, y=0.0, z=0.0)

        # 4. 关节 1: 基座旋转中心点 (肩部)
        p1 = JointPose3D(x=slide_x, y=0.0, z=self.base_height)

        # 5. 关节 2: 肘部旋转关节 (大臂末端)
        # 大臂在水平面上的投影分量 r1 与垂直分量 z1
        r1 = self.l1 * math.cos(rad_pitch1)
        p2 = JointPose3D(
            x=slide_x + r1 * math.cos(rad_yaw),
            y=r1 * math.sin(rad_yaw),
            z=p1.z + self.l1 * math.sin(rad_pitch1)
        )

        # 6. 关节 3: 腕部末端 (小臂末端)
        # 平行四边形/串联结构综合俯仰角 = pitch1 + pitch2
        total_pitch_forearm = rad_pitch1 + rad_pitch2
        r2 = r1 + self.l2 * math.cos(total_pitch_forearm)
        p3 = JointPose3D(
            x=slide_x + r2 * math.cos(rad_yaw),
            y=r2 * math.sin(rad_yaw),
            z=p2.z + self.l2 * math.sin(total_pitch_forearm)
        )

        # 7. 关节 4: 手爪末端抓取点
        total_pitch_gripper = total_pitch_forearm
        r3 = r2 + self.l3 * math.cos(total_pitch_gripper)
        p4 = JointPose3D(
            x=slide_x + r3 * math.cos(rad_yaw),
            y=r3 * math.sin(rad_yaw),
            z=p3.z + self.l3 * math.sin(total_pitch_gripper)
        )

        end_state = EndEffectorState(
            x=p4.x,
            y=p4.y,
            z=p4.z,
            reach_radius=math.hypot(p4.x - slide_x, p4.y),
            pitch_angle=math.degrees(total_pitch_gripper),
            yaw_angle=axis2_deg
        )

        return [p0, p1, p2, p3, p4], end_state

    @staticmethod
    def project_3d_to_2d(
        points: List[JointPose3D],
        width: int,
        height: int,
        view_yaw_deg: float = -45.0,    # 视角水平旋转角
        view_pitch_deg: float = 30.0,   # 视角俯仰仰角
        scale: float = 0.9,             # 画面缩放比
        offset_x: float = 0.0,          # 平移
        offset_y: float = 0.0
    ) -> List[Tuple[float, float, float]]:
        """
        三维空间点透视等轴测投影到二维 Canvas 屏幕像素坐标 (带 Z 深度值用于深度排序渲染)
        """
        yaw_rad = math.radians(view_yaw_deg)
        pitch_rad = math.radians(view_pitch_deg)

        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)

        cx = width / 2.0 + offset_x
        cy = height / 2.0 + 40.0 + offset_y  # 稍微偏下给机械臂留出上升空间

        projected = []
        for p in points:
            # 1. 围绕 Z 轴旋转 (Yaw)
            x1 = p.x * cos_y - p.y * sin_y
            y1 = p.x * sin_y + p.y * cos_y
            z1 = p.z

            # 2. 围绕 X 轴旋转 (Pitch)
            x2 = x1
            y2 = y1 * cos_p - z1 * sin_p
            z2 = y1 * sin_p + z1 * cos_p

            # 3. 屏幕映射 (Y轴向下反转)
            sx = cx + x2 * scale
            sy = cy - z2 * scale
            depth = y2  # 深度用于渲染遮挡

            projected.append((sx, sy, depth))

        return projected

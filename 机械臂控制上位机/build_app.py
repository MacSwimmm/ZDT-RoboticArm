"""
build_app.py
--------------------------------------------------------------------------
四轴机械臂控制上位机 PyInstaller 一键打包与发布脚本
目标产物目录: C:\\Users\\han_z\\Desktop\\MY_PROJECT\\Robotic Arm\\App
"""

import os
import sys
import shutil
import subprocess
import customtkinter

APP_NAME = "机械臂控制上位机"
PROJECT_ROOT = os.path.abspath(os.path.dirname(__file__))
OUTPUT_DIR = os.path.abspath(os.path.join(PROJECT_ROOT, "..", "..", "App"))
BUILD_DIR = os.path.join(PROJECT_ROOT, "build")
CUSTOMTKINTER_DIR = os.path.dirname(customtkinter.__file__)
CUSTOMTKINTER_ASSETS_DIR = os.path.join(CUSTOMTKINTER_DIR, "assets")


def clean_previous():
    """清理历史构建缓存"""
    print(">>> 正在清理历史构建缓存...")
    if os.path.exists(BUILD_DIR):
        try:
            shutil.rmtree(BUILD_DIR)
        except Exception:
            pass

    # 确保目标 App 目录存在
    os.makedirs(OUTPUT_DIR, exist_ok=True)


def build_executable():
    """执行 PyInstaller 打包流程"""
    clean_previous()

    print(f">>> 开始打包 [{APP_NAME}] ...")
    print(f">>> CustomTkinter 资源路径: {CUSTOMTKINTER_DIR}")
    print(f">>> 目标输出目录: {OUTPUT_DIR}")

    # 构造 PyInstaller 参数
    cmd = [
        sys.executable,
        "-m",
        "PyInstaller",
        "--noconfirm",
        "--onedir",  # 目录型绿色版，启动速度快且资源完整
        "--windowed", # 无黑框控制台窗口
        f"--name={APP_NAME}",
        f"--distpath={OUTPUT_DIR}",
        f"--workpath={BUILD_DIR}",
        f"--specpath={BUILD_DIR}",
        f"--add-data={CUSTOMTKINTER_DIR};customtkinter/",
        # CustomTkinter loads theme JSON files at runtime; collect them
        # explicitly because some PyInstaller hook versions omit package data.
        f"--add-data={CUSTOMTKINTER_ASSETS_DIR};customtkinter/assets/",
        "--collect-all=customtkinter",
        "--collect-all=serial",
        "--hidden-import=serial",
        "--hidden-import=serial.tools.list_ports",
        "--hidden-import=customtkinter",
        "--hidden-import=PIL",
        "--hidden-import=math",
        "--hidden-import=queue",
        "--hidden-import=threading",
        "--hidden-import=dataclasses",
        "main.py"
    ]

    print(">>> 执行命令:", " ".join(cmd))
    res = subprocess.run(cmd, cwd=PROJECT_ROOT)
    if res.returncode != 0:
        print("❌ 打包失败！错误码:", res.returncode)
        return False

    print("[SUCCESS] PyInstaller build completed!")

    # 写入使用说明文档
    readme_path = os.path.join(OUTPUT_DIR, "上位机使用说明.txt")
    with open(readme_path, "w", encoding="utf-8") as f:
        f.write(
            "====================================================\n"
            "   FOUR-2 四轴机械臂全功能控制系统 - 运行说明文档   \n"
            "====================================================\n\n"
            "【软件版本】: v1.0.0 (Release)\n"
            "【适用硬件】: STM32F407 控制板 / 张大头 Emm_V5 闭环步进驱动器\n"
            "【主程序路径】: 机械臂控制上位机/机械臂控制上位机.exe\n\n"
            "【串口接线】: DAP-Link TX->F407 PA3，RX<-PA2，GND 共地；115200 8N1\n"
            "【控制链路】: 上位机->DAP-Link串口->F407 USART2->MotorBus->四台电机\n\n"
            "【核心功能特性】:\n"
            "  1. 全屏科技感仪表盘 (F11 一键切换全屏/窗口模式，无需翻页)\n"
            "  2. 四轴独立仪表控制 (丝杆滑台/底座/大臂/小臂，角度微调、速度、加速度设置)\n"
            "  3. MG90S 夹爪舵机控制 (0~180°，闭合/中位/打开快捷位置)\n"
            "  4. 示教再现与轨迹规划 (支持多路点记录、单步/全自动循迹、JSON 导入导出)\n"
            "  5. 四轴遥测矩阵、通讯日志与离线仿真模式\n"
            "  6. 串口自动识别与热插拔重连机制\n\n"
            "【快捷键说明】:\n"
            "  - [F11]: 切换全屏 / 窗口化\n"
            "  - [ESC]: 退出全屏\n"
            "  - [空格键 (Space)]: 一键全局急停\n\n"
            "【首次上电准备】: 将机械臂人工放到确认过的机械零位，确认四轴在线后，\n"
            "  点击“一键设零并使能 (K3)”。程序会先停机/失能，逐轴将当前位置设零，\n"
            "  仅在四轴全部返回成功后自动使能；超时会保持失能。\n\n"
            "祝您使用愉快！\n"
        )
    print(f"[SUCCESS] Readme generated at: {readme_path}")
    return True


if __name__ == "__main__":
    success = build_executable()
    if not success:
        sys.exit(1)

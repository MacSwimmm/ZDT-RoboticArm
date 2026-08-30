"""
theme.py
--------------------------------------------------------------------------
四轴机械臂全屏科技感 UI 主题配置与配色系统
风格: 高对比度赛博朋克深色工控仪表盘 (Cyberpunk High-Tech Dark)
"""

class Theme:
    # 背景与面板基色
    BG_MAIN = "#0A0E17"          # 极深深空黑蓝背景
    BG_CARD = "#111827"          # 一级卡片面板背景
    BG_CARD_HOVER = "#1F2937"    # 悬浮微亮背景
    BG_INPUT = "#0D1320"         # 输入框与凹槽背景
    BG_BANNER = "#1E293B"        # 顶部/通知栏背景

    # 边框与分隔线
    BORDER_DEFAULT = "#1F293D"   # 默认边框
    BORDER_GLOW = "#00F2FE"      # 荧光高亮激活边框
    BORDER_SUCCESS = "#00F5A0"   # 成功状态边框
    BORDER_WARN = "#FF9F43"      # 警告边框
    BORDER_DANGER = "#FF416C"    # 危险边框

    # 品牌核心点缀色 (Neon Accents)
    CYAN_ACCENT = "#00F2FE"      # 科技青 (主交互/目标/高亮)
    CYAN_DARK = "#0284C7"
    PURPLE_ACCENT = "#8A2387"    # 电光紫 (高级功能/模式)
    GREEN_SUCCESS = "#00F5A0"    # 翡翠绿 (使能/在线/就绪)
    GREEN_DARK = "#059669"
    ORANGE_WARN = "#FF9F43"      # 警告橙 (离线/未归零)
    RED_DANGER = "#FF416C"       # 珊瑚红 (急停/堵转/错误)
    RED_CRIMSON = "#E11D48"

    # 文字排版色彩
    TEXT_TITLE = "#FFFFFF"       # 标题白色
    TEXT_BODY = "#E2E8F0"        # 正文浅灰白
    TEXT_SECONDARY = "#94A3B8"   # 次级信息中灰
    TEXT_MUTED = "#64748B"       # 暗灰提示
    TEXT_CYAN = "#38BDF8"        # 科技青文字
    TEXT_GREEN = "#34D399"       # 翡翠绿文字
    TEXT_ORANGE = "#FBBF24"      # 警告文字
    TEXT_RED = "#F87171"         # 危险文字

    # 控件圆角标准
    RADIUS_WINDOW = 0
    RADIUS_CARD = 14
    RADIUS_SUB_CARD = 10
    RADIUS_BTN = 8
    RADIUS_BADGE = 6
    RADIUS_SLIDER = 6

    # 字体规范
    FONT_FAMILY_CN = "Microsoft YaHei UI"
    FONT_FAMILY_EN = "Segoe UI"
    FONT_FAMILY_MONO = "Consolas"

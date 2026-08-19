from maix import camera, display, image, nn, uart, app, time, pinmap

"""
MaixCAM -> STM32F103C8T6 二维云台追踪脚本

用途：
1. MaixCAM 负责手掌/人脸识别。
2. MaixCAM 通过 UART 把目标中心点发送给 STM32。
3. STM32 工程中的 Camera.c 接收 "x,y\n"，Gimbal_Control.c 做 PID 和步进电机控制。

串口协议：
- 识别到目标：发送 "x,y\n"，例如 "153,118\n"。
- 没识别到目标：发送 "N\n"。

为什么丢失目标发 "N\n"：
- 你当前 STM32 的 Camera.c 只解析 "x,y"。
- "N\n" 会解析失败，然后 Camera.c 会调用 Gimbal_UpdateTarget(0, 0, 0)。
- 这样只改 MaixCAM 脚本，也能让 STM32 立刻刹车，不必把 STM32 协议改成 "found,x,y"。

硬件接线：
- MaixCAM TX  -> STM32 PA10 / USART1_RX
- MaixCAM GND -> STM32 GND
- 两边 UART 波特率都设置为 115200
- 不要把 5V 串口电平直接接到 MaixCAM 或 STM32 的 3.3V IO

MaixVision 操作：
1. 打开 MaixVision，连接 MaixCAM。
2. 右侧 Device File Manager 进入 /root/models。
3. 确认手部模型存在：/root/models/hand_landmarks.mud。
   - 你截图里有 hand_landmarks.mud，可以直接用。
   - hand_detector.cvimodel 是实际模型文件之一，不建议直接填进本脚本。
   - .mud 是模型描述文件，MaixPy 通常加载 .mud，它会指向对应的 .cvimodel。
4. 如果要人脸追踪，确认 /root/models/face_detector.mud 存在。
5. 在 MaixVision 中运行本文件。
6. MaixCAM 屏幕能看到框/中心点后，再给 STM32 上电或复位。
7. 如果云台方向反了，改 STM32 的 Gimbal_Control.c 误差方向，或调换电机方向。

模式选择：
- TRACK_MODE = "hand"：手掌/手部关键点追踪，默认推荐。
- TRACK_MODE = "face"：人脸追踪。
"""


# ===================== 用户配置区 =====================

# 可选："hand" 或 "face"
TRACK_MODE = "hand"

# MaixCAM 的 UART 设备。
#
# 注意：
# - MaixPy 的 uart.UART() 不能直接写 "UART1"，要写 Linux 设备路径。
# - UART1 通常是 /dev/ttyS1。
# - UART0 通常是 /dev/ttyS0 或 /dev/serial0，但它经常被系统日志/Maix Comm Protocol 占用。
# - 所以这里推荐用 UART1，避开日志串口。
UART_DEVICE = "/dev/ttyS1"
SERIAL_BAUD = 115200

# MaixCAM / MaixCAM Pro 常见 UART1 引脚映射。
# 接线时：MaixCAM A19(UART1_TX) -> STM32 PA10(USART1_RX)
# 如果你后续需要 STM32 回传数据，再接：MaixCAM A18(UART1_RX) <- STM32 PA9(USART1_TX)
UART1_TX_PIN = "A19"
UART1_RX_PIN = "A18"

# STM32 工程 Gimbal_Control.h 当前按 320x240 画面中心计算。
# 如果后续你改了 STM32 的 CAM_CENTER_X/Y，这里也要同步。
STM32_FRAME_W = 320
STM32_FRAME_H = 240

# 发送周期。0.05s = 20Hz，和 STM32 20ms 控制周期比较匹配。
SEND_INTERVAL_S = 0.05

# 目标丢失时是否发送 "N\n"。
# True：丢失后 STM32 立即刹车。
# False：依赖 STM32 的 100ms 超时保护。
SEND_LOST_FRAME = True

# 模型路径。按你截图里的 /root/models 文件列表设置。
HAND_MODEL_PATH = "/root/models/hand_landmarks.mud"
FACE_MODEL_PATH = "/root/models/face_detector.mud"

# 检测阈值。越高越严格，越低越容易误检。
HAND_CONF_TH = 0.7
HAND_IOU_TH = 0.45
HAND_CONF_TH2 = 0.8
FACE_CONF_TH = 0.4
FACE_IOU_TH = 0.45


# ===================== 工具函数 =====================

def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def map_to_stm32_frame(x, y, cam_w, cam_h):
    """
    把 MaixCAM 当前模型输入坐标映射到 STM32 使用的 320x240 坐标系。

    手部关键点官方例程通常使用 320x224，而你的 STM32 用 320x240。
    如果不映射，Y 轴中心会从 112 变成 STM32 认为的 120，导致云台有固定偏差。
    """
    send_x = int(x * STM32_FRAME_W / cam_w)
    send_y = int(y * STM32_FRAME_H / cam_h)
    send_x = clamp(send_x, 0, STM32_FRAME_W - 1)
    send_y = clamp(send_y, 0, STM32_FRAME_H - 1)
    return send_x, send_y


def select_largest_object(objs):
    """
    多个目标同时出现时，选择画面中面积最大的那个。
    这样比直接取 objs[0] 稳一点，通常会跟随离镜头最近/最明显的目标。
    """
    if len(objs) == 0:
        return None
    best = objs[0]
    best_area = best.w * best.h
    for i in range(1, len(objs)):
        obj = objs[i]
        area = obj.w * obj.h
        if area > best_area:
            best = obj
            best_area = area
    return best


def send_text(ser, text):
    if ser:
        try:
            # 使用 write_str 直接发送字符串，write() 需要 maix.Bytes 类型
            ser.write_str(text)
        except Exception as e:
            print("UART Write Error:", e)

# ===================== 初始化 UART =====================

try:
    # 先把 A18/A19 复用成 UART1，再打开 /dev/ttyS1。
    pinmap.set_pin_function(UART1_RX_PIN, "UART1_RX")
    pinmap.set_pin_function(UART1_TX_PIN, "UART1_TX")
    ser = uart.UART(UART_DEVICE, SERIAL_BAUD)
    ser.open()  # 必须显式打开串口，构造函数不会自动打开！
    print("UART init ok:", UART_DEVICE, SERIAL_BAUD)
except Exception as e:
    print("UART init failed:", e)
    ser = None


# ===================== 初始化模型和摄像头 =====================

if TRACK_MODE == "hand":
    detector = nn.HandLandmarks(model=HAND_MODEL_PATH)
    cam_w = 320
    cam_h = 224
    cam = camera.Camera(cam_w, cam_h, detector.input_format())
    print("tracking mode: hand, model:", HAND_MODEL_PATH)

elif TRACK_MODE == "face":
    detector = nn.FaceDetector(model=FACE_MODEL_PATH)
    cam_w = detector.input_width()
    cam_h = detector.input_height()
    cam = camera.Camera(cam_w, cam_h, detector.input_format())
    print("tracking mode: face, model:", FACE_MODEL_PATH)

else:
    raise ValueError("TRACK_MODE must be 'hand' or 'face'")

disp = display.Display()
last_send_time = 0
last_found = False


# ===================== 姿态算法 =====================
def is_fist(points):
    import math
    if not points: return False
    
    try:
        length = len(points)
        # 兼容不同固件和模型版本的 points 格式
        if length == 42:
            def pt(i): return points[i*2], points[i*2+1]
        elif length == 63: 
            def pt(i): return points[i*3], points[i*3+1]
        elif length == 71:
            # 71 = 8(检测框信息) + 21*3(21个点的x,y,z)
            def pt(i): return points[8 + i*3], points[8 + i*3 + 1]
        elif length == 84: 
            def pt(i): return points[i*4], points[i*4+1]
        elif length == 21:
            if hasattr(points[0], 'x'):
                def pt(i): return points[i].x, points[i].y
            else:
                def pt(i): return points[i][0], points[i][1]
        else:
            print(f"Error: Unknown points length: {length}. Points: {points[:10]}...")
            return False
            
        # 0: 手腕, 9: 中指指根(MCP), 12: 中指指尖(TIP)
        wrist = pt(0)
        mcp = pt(9)
        tip = pt(12)
        
        # 算出 手腕->中指根 的距离 (手掌固定长度)
        palm_len = math.sqrt((mcp[0]-wrist[0])**2 + (mcp[1]-wrist[1])**2)
        # 算出 手腕->中指尖 的距离
        tip_len = math.sqrt((tip[0]-wrist[0])**2 + (tip[1]-wrist[1])**2)
        
        if palm_len < 1: return False
        
        # 计算比例
        ratio = tip_len / palm_len
        
        # 张手时，指尖伸长，比例通常在 1.8 ~ 2.2 左右
        # 握拳时，指尖缩回手心，比例通常在 0.8 ~ 1.2 左右
        # 打印出来方便在终端里看！
        print(f"Hand ratio: {ratio:.2f}")
        
        # 小于 1.3 判定为握拳
        return ratio < 1.3
        
    except Exception as e:
        print("is_fist error:", e)
        return False


# ===================== 主循环 =====================

while not app.need_exit():
    img = cam.read()
    found = False
    target_x = cam_w // 2
    target_y = cam_h // 2

    if TRACK_MODE == "hand":
        objs = detector.detect(
            img,
            conf_th=HAND_CONF_TH,
            iou_th=HAND_IOU_TH,
            conf_th2=HAND_CONF_TH2,
            landmarks_rel=False
        )
        obj = select_largest_object(objs)
        if obj:
            # 使用关键点算法判断是否握拳
            if is_fist(obj.points):
                # 如果是握拳，设为 False 让云台停止追踪
                img.draw_string(10, 70, "Gesture: FIST (Stop)", image.COLOR_YELLOW, scale=1.5)
                found = False
            else:
                # 其它手势（如张手），正常追踪
                img.draw_string(10, 70, "Gesture: OPEN (Track)", image.COLOR_GREEN, scale=1.5)
                found = True
                target_x = obj.x + obj.w // 2
                target_y = obj.y + obj.h // 2

            detector.draw_hand(img, obj.class_id, obj.points, 4, 10, box=True)
            if found:
                # 追踪中，画红色十字
                img.draw_cross(target_x, target_y, image.COLOR_RED, 5)
            else:
                # 握拳不追踪，画黄色十字作为提示
                img.draw_cross(obj.x + obj.w // 2, obj.y + obj.h // 2, image.COLOR_YELLOW, 5)

    else:
        objs = detector.detect(img, conf_th=FACE_CONF_TH, iou_th=FACE_IOU_TH)
        obj = select_largest_object(objs)
        if obj:
            found = True
            target_x = obj.x + obj.w // 2
            target_y = obj.y + obj.h // 2

            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_GREEN)
            img.draw_cross(target_x, target_y, image.COLOR_RED, 5)
            if hasattr(obj, "points"):
                img.draw_keypoints(obj.points, image.COLOR_RED, size=4)

    # 画出中心点
    center_x = cam_w // 2
    center_y = cam_h // 2
    img.draw_line(center_x, center_y - 10, center_x, center_y + 10, image.COLOR_BLUE, 1)
    img.draw_line(center_x - 10, center_y, center_x + 10, center_y, image.COLOR_BLUE, 1)

    # 【重要新增】：如果串口初始化失败，直接在屏幕上打出巨大的红色警告！
    if ser is None:
        img.draw_string(10, 10, "UART FAILED! Check Pins!", image.COLOR_RED, scale=2.0)
        img.draw_string(10, 40, "No data is being sent.", image.COLOR_RED, scale=1.5)
    else:
        # 如果初始化成功，屏幕左上角显示绿色的 OK
        img.draw_string(10, 10, "UART OK: " + UART_DEVICE, image.COLOR_GREEN, scale=1.5)

    now = time.time()
    if now - last_send_time >= SEND_INTERVAL_S:
        if found:
            send_x, send_y = map_to_stm32_frame(target_x, target_y, cam_w, cam_h)
            packet = "%d,%d\n" % (send_x, send_y)
            send_text(ser, packet)
            print("send target:", packet.strip())
            last_send_time = now
            last_found = True

        elif SEND_LOST_FRAME:
            send_text(ser, "N\n")
            if last_found:
                print("send lost: N")
            last_send_time = now
            last_found = False

    disp.show(img)

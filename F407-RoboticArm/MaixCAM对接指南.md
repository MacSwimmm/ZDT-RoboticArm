# MaixCAM 视觉端与 STM32 二维云台对接指南

本指南供负责 **MaixCAM 视觉端** 开发的同学使用，用于将靶心/目标检测结果通过串口（UART）发送给 **STM32 主控端**，实现双轴云台自动追踪。

---

## 1. 硬件接线与串口配置

### 物理接线
请将 MaixCAM 和 STM32 按如下方式连接：

| MaixCAM 引脚 | 功能 | 连接方向 | STM32F407 开发板引脚 |
| :--- | :--- | :---: | :--- |
| **A19 (UART1_TX)** | 串口发送 | $\rightarrow$ | **PA10 (USART2_RX)** |
| **GND** | 信号地 | $\leftrightarrow$ | **GND** |

> [!WARNING]
> 1. 请务必**共地（GND 相连）**，否则串口通信会出现乱码或无法接收。
> 2. 请勿将 5V 电源线直接接到两边的 3.3V IO 引脚上，以免烧毁芯片。
> 3. STM32 接收端使用的是 **USART2** (PA10)，请勿接错串口。

### 串口参数
* **波特率 (Baud Rate)**：`115200`
* **数据位**：`8`
* **停止位**：`1`
* **校验位**：`None` (无)

---

## 2. 串口通信协议

通信采用**以换行符 `\n` 结尾的文本控制帧**。发送频率推荐为 **20Hz**（即每 50ms 发送一次），以匹配 STM32 端 20ms 的 PID 控制周期。

### 协议格式
1. **识别到目标时**：发送目标的中心点坐标。
   * **格式**：`"x,y\n"`
   * **示例**：`"165,112\n"`（以 ASCII 码字符串形式发送）
2. **目标丢失/未识别到时**：发送丢失信号。
   * **格式**：`"N\n"` （大写字母 `N` 加换行符）
   * **作用**：STM32 收到 `"N\n"` 后会判定目标丢失并**立即刹车**，防止云台惯性乱晃。

---

## 3. 画面坐标系规范（关键）

无论你在 MaixCAM 端使用何种摄像头分辨率（如 320x240、640x480 或 240x240 运行神经网络），**发送给 STM32 的坐标必须等比例映射到 `320 x 240` 画面中**。

* **STM32 期望的分辨率**：`Width = 320`, `Height = 240`
* **STM32 认定的画面中心点**：`(X = 160, Y = 120)`
* **坐标有效范围**：
  * $X \in [0, 319]$
  * $Y \in [0, 239]$

### 映射公式
假设你图像检测输出的靶心中心在原图中的坐标为 $(X_{\text{raw}}, Y_{\text{raw}})$，原图分辨率为 $W_{\text{raw}} \times H_{\text{raw}}$。
则发送给 STM32 的坐标 $(X_{\text{send}}, Y_{\text{send}})$ 计算如下：
\[ X_{\text{send}} = \text{clamp}\left( \text{int}\left(X_{\text{raw}} \times \frac{320}{W_{\text{raw}}}\right),\ 0,\ 319 \right) \]
\[ Y_{\text{send}} = \text{clamp}\left( \text{int}\left(Y_{\text{raw}} \times \frac{240}{H_{\text{raw}}}\right),\ 0,\ 239 \right) \]

---

## 4. MaixCAM 端 Python 接口模板

你可以直接基于以下 Python 代码进行二次开发，将**步骤 3** 替换为你具体的靶心识别算法（如 `find_blobs` 颜色识别或 `YOLO` 模型推理）。

```python
import time
from maix import camera, display, image, uart, app, pinmap

# ==================== 1. 配置参数 ====================
UART_DEVICE = "/dev/ttyS1"  # MaixCAM 串口 1 (对应 A18/A19)
SERIAL_BAUD = 115200

UART1_TX_PIN = "A19"
UART1_RX_PIN = "A18"

# STM32 固定的画面参考尺寸
STM32_FRAME_W = 320
STM32_FRAME_H = 240

# 发送间隔 (0.05s = 20Hz)
SEND_INTERVAL_S = 0.05

# ==================== 2. 坐标映射与限幅 ====================
def clamp(value, low, high):
    return max(low, min(value, high))

def map_to_stm32_frame(x, y, cam_w, cam_h):
    """将任意摄像头分辨率的坐标等比例映射到 320x240 空间"""
    send_x = int(x * STM32_FRAME_W / cam_w)
    send_y = int(y * STM32_FRAME_H / cam_h)
    send_x = clamp(send_x, 0, STM32_FRAME_W - 1)
    send_y = clamp(send_y, 0, STM32_FRAME_H - 1)
    return send_x, send_y

# ==================== 3. 初始化硬件 ====================
# 配置引脚复用为 UART1
pinmap.set_pin_function(UART1_RX_PIN, "UART1_RX")
pinmap.set_pin_function(UART1_TX_PIN, "UART1_TX")

# 打开串口
ser = uart.UART(UART_DEVICE, SERIAL_BAUD)
ser.open()

# 摄像头与显示器初始化 (假设摄像头分辨率为 320x240 跑颜色/找圆)
cam_w, cam_h = 320, 240
cam = camera.Camera(cam_w, cam_h, image.Format.FMT_RGB888)
disp = display.Display()

last_send_time = 0

# ==================== 4. 主循环 ====================
while not app.need_exit():
    img = cam.read()
    
    found = False
    target_x = 0
    target_y = 0
    
    # ------------------【在此处实现你的靶心识别算法】------------------
    # 示例 A (颜色检测找靶心):
    # BULLSEYE_THRESHOLD = [0, 80, 40, 80, 10, 60]  # 示例 LAB 颜色阈值
    # blobs = img.find_blobs([BULLSEYE_THRESHOLD])
    # if blobs:
    #     max_blob = max(blobs, key=lambda b: b.pixels())
    #     target_x = max_blob.cx()
    #     target_y = max_blob.cy()
    #     found = True
    #     img.draw_rect(max_blob.x(), max_blob.y(), max_blob.w(), max_blob.h(), image.COLOR_GREEN)
    
    # 示例 B (直接用官方模型检测):
    # (如果是 YOLO 模型，提取检测框中心点即可)
    # -----------------------------------------------------------------

    # 绘制辅助线：屏幕几何中心点 (用于对准参考)
    img.draw_line(cam_w//2, cam_h//2 - 10, cam_w//2, cam_h//2 + 10, image.COLOR_BLUE, 1)
    img.draw_line(cam_w//2 - 10, cam_h//2, cam_w//2 + 10, cam_h//2, image.COLOR_BLUE, 1)

    # 5. 控制频率发送串口 data
    now = time.time()
    if now - last_send_time >= SEND_INTERVAL_S:
        if found:
            # 转换坐标并发送
            send_x, send_y = map_to_stm32_frame(target_x, target_y, cam_w, cam_h)
            packet = f"{send_x},{send_y}\n"
            ser.write_str(packet)
            print(f"发送目标坐标 -> X:{send_x}, Y:{send_y}")
            # 并在画面上用红色十字标出靶心
            img.draw_cross(target_x, target_y, image.COLOR_RED, 5)
        else:
            # 目标丢失，发送 N 让云台停机
            ser.write_str("N\n")
            
        last_send_time = now

    disp.show(img)
```

---

## 5. 联调建议与测试步骤

1. **第一步：验证物理接线与波特率**
   * 用 USB 转 TTL 模块将 MaixCAM 的 A19(TX) 接到电脑，打开串口助手（115200波特率）。
   * 运行 MaixCAM 上的脚本，确认串口助手能稳定收到 `160,120\n` 或 `N\n` 格式数据。
2. **第二步：联调云台**
   * 将 MaixCAM TX 接至 STM32 PA10 (USART2_RX)，开启云台供电。
   * **观察 STM32 端调试变量**：在 Keil 中进入仿真，将结构体 `g_camera_debug` 添加到 Watch 窗口。
     * `g_camera_debug.valid_coord_count` 递增表示成功接收解析坐标。
     * `g_camera_debug.lost_frame_count` 递增表示成功接收目标丢失信号。
     * `g_camera_debug.last_x` 和 `last_y` 应该与 MaixCAM 画面上的红色十字映射点一致。
3. **第三步：确认云台控制方向**
   * 靶心在画面**偏左**（即 $X < 160$），云台应控制电机**向左转**以使靶心回到中心；
   * 靶心在画面**偏上**（即 $Y < 120$），云台应控制电机**向上仰**以使靶心回到中心。
   * 如果反了，请联系 STM32 端同学在 `Gimbal_Control.c` 里将对应轴的 PID 误差项乘以 `-1`。

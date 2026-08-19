/*
 * OLED.c
 * --------------------------------------------------------------------------
 * SSD1306 OLED 显示驱动与页面绘制模块。
 *
 * 本文件负责三件事：维护 128x64 单色显存、把 OledView_t 渲染成页面、
 * 以“每次只发送一页”的方式刷新 I2C。字模数据独立放在 OLED_Font.h，
 * 上层 ArmControl 只提交显示模型，不接触 I2C 和像素细节。
 *
 * OLED 故障时，本模块会释放并恢复 I2C 总线，再周期性尝试重新初始化。
 * 所有发送均带短超时，OLED 断线不会长期阻塞电机总线和编码器控制。
 */

#include "OLED.h"
#include "OLED_Font.h"
#include "i2c.h"

#include <string.h>

#define OLED_WIDTH              128U
#define OLED_PAGES              8U
#define OLED_ADDR_1             (0x3CU << 1)
#define OLED_ADDR_2             (0x3DU << 1)
#define OLED_REFRESH_MS         100U
#define OLED_TRANSFER_TIMEOUT   2U

static uint8_t oled_address;
static uint8_t oled_available;
static uint8_t oled_dirty;
static uint8_t oled_page_index;
static uint32_t oled_next_refresh;
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];
static OledView_t oled_view;

/* 清空软件显存，不立即访问 I2C。 */
static void Oled_ClearBuffer(void)
{
    (void)memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void Oled_DrawChar(uint8_t x, uint8_t row, char c)
{
    uint8_t glyph[5];
    uint8_t i;
    uint16_t offset;

    if ((x >= OLED_WIDTH) || (row >= OLED_PAGES)) return;
    OLED_FontGetGlyph(c, glyph);
    offset = (uint16_t)row * OLED_WIDTH + x;
    for (i = 0U; i < 5U && (x + i) < OLED_WIDTH; ++i) {
        oled_buffer[offset + i] = glyph[i];
    }
}

static void Oled_DrawText(uint8_t x, uint8_t row, const char *text)
{
    while ((*text != '\0') && (x < OLED_WIDTH)) {
        Oled_DrawChar(x, row, *text++);
        x = (uint8_t)(x + 6U);
    }
}

static void Oled_DrawProgress(uint8_t x, uint8_t row, uint8_t width, int16_t value)
{
    uint8_t i;
    uint8_t fill;
    uint16_t offset = (uint16_t)row * OLED_WIDTH + x;

    if (width < 4U) return;
    if (value < -1800) value = -1800;
    if (value > 1800) value = 1800;
    fill = (uint8_t)(((int32_t)(value + 1800) * (width - 2U)) / 3600L);
    for (i = 0U; i < width && (x + i) < OLED_WIDTH; ++i) {
        if ((i == 0U) || (i == (width - 1U))) {
            oled_buffer[offset + i] = 0x7EU;
        } else if ((i - 1U) < fill) {
            oled_buffer[offset + i] = 0x7EU;
        } else {
            oled_buffer[offset + i] = 0x42U;
        }
    }
}

static void Oled_FormatAngle(char *text, int16_t tenths)
{
    uint16_t magnitude = (uint16_t)((tenths < 0) ? -tenths : tenths);
    text[0] = (tenths < 0) ? '-' : '+';
    text[1] = (char)('0' + ((magnitude / 1000U) % 10U));
    text[2] = (char)('0' + ((magnitude / 100U) % 10U));
    text[3] = (char)('0' + ((magnitude / 10U) % 10U));
    text[4] = '.';
    text[5] = (char)('0' + (magnitude % 10U));
    text[6] = '\0';
}

static void Oled_FormatU16(char *text, uint16_t value, uint8_t digits)
{
    uint8_t index;

    text[digits] = '\0';
    for (index = 0U; index < digits; ++index) {
        text[digits - 1U - index] = (char)('0' + (value % 10U));
        value /= 10U;
    }
}

static void Oled_FormatS16(char *text, int16_t value)
{
    uint16_t magnitude = (uint16_t)((value < 0) ? -value : value);

    text[0] = (value < 0) ? '-' : '+';
    Oled_FormatU16(&text[1], magnitude, 4U);
}

static const char *Oled_NoticeText(uint8_t notice)
{
    switch (notice) {
    case OLED_NOTICE_LIMIT: return "REJECT ANGLE LIMIT";
    case OLED_NOTICE_OFFLINE: return "REJECT MOTOR OFF";
    case OLED_NOTICE_DISABLED: return "REJECT DISABLED";
    case OLED_NOTICE_ZERO: return "ZERO REQUIRED";
    case OLED_NOTICE_STOPPED: return "STOP LOCK ACTIVE";
    case OLED_NOTICE_BUS: return "MOTOR BUS FAULT";
    case OLED_NOTICE_ZERO_ALL: return "ZERO ALL QUEUED";
    case OLED_NOTICE_ENABLED: return "ENABLE ALL QUEUED";
    case OLED_NOTICE_DISABLED_ALL: return "DISABLE ALL QUEUED";
    default: return 0;
    }
}

static const char *Oled_AxisName(uint8_t axis)
{
    switch (axis) {
    case 0U: return "SCREW";
    case 1U: return "BASE";
    case 2U: return "UPPER";
    default: return "FOREARM";
    }
}

static void Oled_BuildPage(void)
{
    char angle[8];
    char number[8];
    const char *notice_text;
    uint8_t axis;
    uint8_t row;

    Oled_ClearBuffer();
    if (oled_view.page == OLED_PAGE_HOME) {
        Oled_DrawText(0U, 0U, "ROBOT ARM HOME");
        Oled_DrawText(0U, 1U, "BUS:");
        Oled_DrawText(30U, 1U, oled_view.serial_ok ? "OK" : "WAIT");
        for (axis = 0U; axis < 4U; ++axis) {
            row = (uint8_t)(2U + axis);
            number[0] = (char)('1' + axis);
            number[1] = '\0';
            Oled_DrawText(0U, row, number);
            Oled_DrawText(12U, row, (oled_view.online_mask & (1U << axis)) ? "ON" : "OFF");
            Oled_DrawText(36U, row, (oled_view.zero_mask & (1U << axis)) ? "Z" : "NZ");
            Oled_DrawText(54U, row, (oled_view.enabled_mask & (1U << axis)) ? "EN" : "DS");
            Oled_FormatAngle(angle, oled_view.actual_angle_tenths[axis]);
            Oled_DrawText(72U, row, angle);
        }
        notice_text = Oled_NoticeText(oled_view.notice);
        Oled_DrawText(0U, 7U, (notice_text != 0) ? notice_text : "K1/K2 PAGE K3 OK");
    } else if ((oled_view.page >= OLED_PAGE_AXIS1) && (oled_view.page <= OLED_PAGE_AXIS4)) {
        axis = (uint8_t)(oled_view.page - OLED_PAGE_AXIS1);
        Oled_DrawText(0U, 0U, "AXIS");
        number[0] = (char)('1' + axis);
        number[1] = '\0';
        Oled_DrawText(30U, 0U, number);
        Oled_DrawText(42U, 0U, Oled_AxisName(axis));
        Oled_DrawText(96U, 0U, (oled_view.online_mask & (1U << axis)) ? "ON" : "OFF");
        Oled_DrawText(0U, 1U, "ACT:");
        Oled_FormatAngle(angle, oled_view.actual_angle_tenths[axis]);
        Oled_DrawText(30U, 1U, angle);
        Oled_DrawText(0U, 2U, "TGT:");
        Oled_FormatAngle(angle, oled_view.target_angle_tenths[axis]);
        Oled_DrawText(30U, 2U, angle);
        Oled_DrawText(0U, 3U, "RPM SET:");
        Oled_FormatU16(number, oled_view.speed_rpm[axis], 4U);
        Oled_DrawText(54U, 3U, number);
        Oled_DrawText(0U, 4U, "ACC:");
        if (oled_view.accel[axis] == 0U) {
            Oled_DrawText(30U, 4U, "DIRECT");
        } else {
            Oled_FormatU16(number, oled_view.accel[axis], 3U);
            Oled_DrawText(30U, 4U, number);
        }
        Oled_DrawText(0U, 5U, "VEL:");
        Oled_FormatS16(number, oled_view.actual_rpm[axis]);
        Oled_DrawText(30U, 5U, number);
        notice_text = Oled_NoticeText(oled_view.notice);
        Oled_DrawText(0U, 6U, (notice_text != 0) ? notice_text :
                      ((oled_view.flags[axis] & 0x0CU) ? "FLAG:STALL" : "FLAG:OK"));
        Oled_DrawProgress(0U, 7U, 96U, oled_view.actual_angle_tenths[axis]);
    } else if (oled_view.page == OLED_PAGE_ZERO) {
        Oled_DrawText(0U, 0U, "ZERO CONFIRM");
        Oled_DrawText(0U, 1U, "HOLD ARM SAFE");
        Oled_DrawText(0U, 2U, "SELECT AXIS:");
        number[0] = (char)('1' + oled_view.selected_axis);
        number[1] = '\0';
        Oled_DrawText(78U, 2U, number);
        Oled_DrawText(0U, 3U, "K6 ZERO SELECT");
        Oled_DrawText(0U, 4U, "K3 ZERO ALL");
        Oled_DrawText(0U, 5U, "Z:");
        for (axis = 0U; axis < 4U; ++axis) {
            number[0] = (oled_view.zero_mask & (1U << axis)) ? 'Y' : 'N';
            number[1] = '\0';
            Oled_DrawText((uint8_t)(18U + axis * 18U), 5U, number);
        }
        notice_text = Oled_NoticeText(oled_view.notice);
        Oled_DrawText(0U, 7U, (notice_text != 0) ? notice_text : "K4 ENABLE AFTER Z");
    } else {
        Oled_DrawText(0U, 0U, "ENCODER OBJECT");
        Oled_DrawText(0U, 2U, (oled_view.mode == OLED_MODE_ANGLE) ? "> ANGLE" : "  ANGLE");
        Oled_DrawText(0U, 3U, (oled_view.mode == OLED_MODE_SPEED) ? "> SPEED" : "  SPEED");
        Oled_DrawText(0U, 4U, (oled_view.mode == OLED_MODE_ACCEL) ? "> ACCEL" : "  ACCEL");
        Oled_DrawText(0U, 7U, "K1/K2 SEL K3 OK");
    }
}

static void Oled_RecoverBus(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t i;

    /*
     * SSD1306 或连线异常可能把 SDA 保持为低电平。先把 PB8/PB9 临时改成
     * 开漏 GPIO，手动输出 9 个 SCL 脉冲并生成 STOP，再恢复 HAL I2C。
     */
    (void)HAL_I2C_DeInit(&hi2c1);
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    for (i = 0U; i < 9U; ++i) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1U);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    MX_I2C1_Init();
}

static uint8_t Oled_SendPage(uint8_t page)
{
    uint8_t command[4] = {0x00U, (uint8_t)(0xB0U | page), 0x00U, 0x10U};
    uint8_t data[OLED_WIDTH + 1U];

    data[0] = 0x40U;
    (void)memcpy(&data[1], &oled_buffer[(uint16_t)page * OLED_WIDTH], OLED_WIDTH);
    if (HAL_I2C_Master_Transmit(&hi2c1, oled_address, command, sizeof(command), OLED_TRANSFER_TIMEOUT) != HAL_OK) {
        return 0U;
    }
    return (HAL_I2C_Master_Transmit(&hi2c1, oled_address, data, sizeof(data), OLED_TRANSFER_TIMEOUT) == HAL_OK) ? 1U : 0U;
}

void OLED_Init(void)
{
    static const uint8_t init_commands[] = {
        0x00U, 0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U,
        0x40U, 0x8DU, 0x14U, 0x20U, 0x00U, 0xA1U, 0xC8U, 0xDAU,
        0x12U, 0x81U, 0x7FU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U,
        0xA6U, 0xAFU
    };

    oled_available = 0U;
    oled_dirty = 1U;
    oled_page_index = 0U;
    if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_1, 2U, 10U) == HAL_OK) {
        oled_address = OLED_ADDR_1;
    } else if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_2, 2U, 10U) == HAL_OK) {
        oled_address = OLED_ADDR_2;
    } else {
        oled_next_refresh = HAL_GetTick() + 1000U;
        return;
    }
    if (HAL_I2C_Master_Transmit(&hi2c1, oled_address, (uint8_t *)init_commands,
                                sizeof(init_commands), 20U) == HAL_OK) {
        oled_available = 1U;
        oled_next_refresh = HAL_GetTick();
    } else {
        oled_next_refresh = HAL_GetTick() + 1000U;
    }
}

void OLED_UpdateView(const OledView_t *view)
{
    if (view == 0) return;

    /* 复制快照，避免 OLED 绘制时直接依赖 MotorBus 或 ArmControl 的内部变量。 */
    oled_view = *view;
    oled_dirty = 1U;
}

void OLED_Process(void)
{
    uint32_t now = HAL_GetTick();

    if (oled_available == 0U) {
        if (now >= oled_next_refresh) {
            oled_next_refresh = now + 1000U;
            OLED_Init();
        }
        return;
    }
    if ((oled_dirty == 0U) && (now < oled_next_refresh)) return;
    if (oled_page_index == 0U) {
        /* 一帧开始时统一重建显存，随后每次主循环仅发送一页（128 字节）。 */
        Oled_BuildPage();
    }
    if (Oled_SendPage(oled_page_index) == 0U) {
        oled_available = 0U;
        oled_next_refresh = now + 1000U;
        Oled_RecoverBus();
        return;
    }
    ++oled_page_index;
    if (oled_page_index >= OLED_PAGES) {
        oled_page_index = 0U;
        oled_dirty = 0U;
        oled_next_refresh = now + OLED_REFRESH_MS;
    }
}

uint8_t OLED_IsAvailable(void)
{
    return oled_available;
}

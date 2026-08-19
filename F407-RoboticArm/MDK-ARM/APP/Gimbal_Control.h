#ifndef __GIMBAL_CONTROL_H
#define __GIMBAL_CONTROL_H

#include <stdint.h>

/* MaixCAM 脚本当前使用 320x240 画面，所以中心点是 (160,120)。 */
#define CAM_CENTER_X      160.0f
#define CAM_CENTER_Y      120.0f

/* 视觉死区：目标落在中心附近时不再微调，减少来回抖动。 */
#define DEADZONE_X    5.0f
#define DEADZONE_Y    5.0f

/* 方向自检状态，可在 Keil 中通过 g_gimbal_debug.direction_state 观察。 */
typedef enum {
    GIMBAL_DIR_WAITING = 0,
    GIMBAL_DIR_PREPARING,
    GIMBAL_DIR_RUNNING,
    GIMBAL_DIR_SETTLING,
    GIMBAL_DIR_READY,
    GIMBAL_DIR_FAULT
} GimbalDirectionState_t;

/* 云台控制调试量。fault_axis: 0=无, 1=X, 2=Y；
 * fault_reason: 0=无, 1=两种方向都使误差增大,
 *               2=多次测试无明显位移, 3=初始化电机命令发送失败。
 */
typedef struct {
    uint8_t direction_state;
    uint8_t test_axis;
    uint8_t x_direction_verified;
    uint8_t y_direction_verified;
    uint8_t x_direction_inverted;
    uint8_t y_direction_inverted;
    uint8_t fault_axis;
    uint8_t fault_reason;
    uint8_t last_motor_tx_status;
    uint8_t last_motor_tx_status_x;
    uint8_t last_motor_tx_status_y;
    uint32_t valid_target_sample_count;
    uint32_t direction_test_count;
    int16_t test_start_coord;
    int16_t test_end_coord;
    float test_start_abs_error;
    float test_end_abs_error;
    int32_t commanded_rpm_x;
    int32_t commanded_rpm_y;
} GimbalDebug_t;

extern volatile GimbalDebug_t g_gimbal_debug;

void Gimbal_Init(void);
void Gimbal_UpdateTarget(int16_t x, int16_t y, uint8_t is_found);
void Gimbal_Loop(void);

#endif /* __GIMBAL_CONTROL_H */

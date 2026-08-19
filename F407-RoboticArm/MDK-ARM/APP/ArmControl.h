#ifndef __ARM_CONTROL_H
#define __ARM_CONTROL_H

/* 初始化默认主页、角度调节模式和选中轴，不产生任何自动运动。 */
void ArmControl_Init(void);

/* 在主循环中调用：消费输入事件、更新业务状态并提交 OLED 显示快照。 */
void ArmControl_Process(void);

#endif

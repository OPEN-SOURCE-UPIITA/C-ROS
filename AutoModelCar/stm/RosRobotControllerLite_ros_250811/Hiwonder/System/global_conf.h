#ifndef __GLOBAL_CONF_H
#define __GLOBAL_CONF_H

#define ENABLE_IMU  1                     /* IMU 任务是否启动 */
#define ENABLE_LVGL 0                     /* LVGL 任务是否启动 */
#define ENABLE_OLED 1                    /* 是否开启OLED显示 */
#define ENABLE_BATTERY_LOW_ALARM        1 /* 低电压报警是否开启 */

#define KEY1_PUSHED_LEVEL 0
#define KEY2_PUSHED_LEVEL 0
#define LED_SYS_LEVEL_ON  0

#define LED_TASK_PERIOD     30u /* LED状态刷新间隔 */
#define BUZZER_TASK_PERIOD  30u /* 蜂鸣器状态刷新间隔 */
#define BUTTON_TASK_PERIOD  30u /* 板载按键扫描间隔 */
#define BATTERY_TASK_PERIOD 50u /* 电池电量检测间隔 */

#endif


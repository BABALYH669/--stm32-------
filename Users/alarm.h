/**
 ****************************************************************************************************
 * @file        alarm.h
 * @brief       本地阈值报警状态机  —— 阶段一 T1.10
 * @note        状态与灯语严格按《项目计划书v2》§6.1：
 *              | 状态        | LED0   | LED1 烟雾 | LED2 温湿度 | 蜂鸣器                |
 *              | NORMAL      | 1Hz闪  | 灭        | 灭          | 关                    |
 *              | SMOKE       | 1Hz闪  | 常亮      | 灭          | 连续响                |
 *              | TEMP / HUMI | 1Hz闪  | 灭        | 2Hz闪       | 间歇响(200ms响/800ms停)|
 *              | MULTI       | 1Hz闪  | 常亮      | 2Hz闪       | 间歇急促(100ms/100ms)  |
 ****************************************************************************************************
 */

#ifndef __ALARM_H
#define __ALARM_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"
#include "env_data.h"
#include "./BSP/AT24C02/at24c02.h"

void    alarm_init(void);                   /* 初始化报警状态机（内部阈值先取默认值） */
void    alarm_update(const env_data_t *d);  /* 100 ms 调用一次：阈值判定（迟滞 + 连续确认） */
uint8_t alarm_get_state(void);              /* 返回当前 alarm_state_t */
void    alarm_tick(void);                   /* 10 ms 调用：LED0 心跳 / LED1 / LED2 / 蜂鸣器 / LED3 调光 */

/* 下面两个是【新增】接口（不改变上面任何签名），阶段二/阶段三会用到：
 *   alarm_reload_thresh() 已列入阶段三 §5 的最终冻结契约；
 *   alarm_state_str()     供串口 / 网络 JSON / UI 统一取字符串，避免各处硬编码。 */
void        alarm_reload_thresh(const thresh_t *t);     /* 阈值变化后重载 */
const char *alarm_state_str(uint8_t st);                /* "none"/"smoke"/"temp"/"humi"/"multi" */

#endif /* __ALARM_H */

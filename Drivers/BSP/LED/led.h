/**
 ****************************************************************************************************
 * @file        led.h
 * @brief       LED0 / LED1 / LED2 / LED4 驱动（GPIO，低电平点亮）  —— 阶段一 T1.2
 * @note        LED3 走 PWM 调光，不在本文件，见 ./BSP/LED_PWM/led_pwm.h
 *              本文件不得包含任何闪烁 / 时基逻辑（闪烁属于 alarm.c 与 main.c）
 ****************************************************************************************************
 */

#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"

/* 灯编号：LED_ID_3 不存在，LED3 用 led_pwm_set() 控制 */
typedef enum
{
    LED_ID_0 = 0,       /* PB5  板载   运行心跳   */
    LED_ID_1,           /* PE5  板载   烟雾报警   */
    LED_ID_2,           /* PF0  外接   温湿度报警 */
    LED_ID_4,           /* PF1  外接   上传指示   */
    LED_ID_MAX
} led_id_t;

void led_init(void);                    /* 初始化 4 路灯，上电默认全部熄灭 */
void led_on(led_id_t id);               /* 点亮（内部写低电平） */
void led_off(led_id_t id);              /* 熄灭 */
void led_toggle(led_id_t id);           /* 翻转 */
void led_set(led_id_t id, uint8_t on);  /* on=1 → 点亮，on=0 → 熄灭 */

#endif /* __LED_H */

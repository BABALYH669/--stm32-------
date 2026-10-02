/**
 ****************************************************************************************************
 * @file        led_pwm.h
 * @brief       LED3（PA6 = TIM3_CH1）PWM 调光驱动  —— 阶段一 T1.3
 * @note        PA6 是唯一预留灯，必须走 PWM：PF / PG 口在 STM32F103 上没有任何定时器通道。
 *              TIM3 无需重映射，也不会与蜂鸣器可能用到的 TIM4_CH3(PB8) 冲突。
 ****************************************************************************************************
 */

#ifndef __LED_PWM_H
#define __LED_PWM_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"

#define LED_PWM_FULL        1000U       /* ARR = 999，占空比范围 0 ~ 1000 */

/* 正/反相关：0 = 环境越暗 LED3 越亮（默认）；1 = 环境越亮 LED3 越亮 */
#define LIGHT_INVERT        0

void     led_pwm_init(void);            /* 初始化 TIM3_CH1，1 kHz，初始占空比 0 */
void     led_pwm_set(uint16_t duty);    /* duty: 0 ~ 1000，0 = 全灭，1000 = 全亮 */
uint16_t led_pwm_get(void);             /* 返回最近一次设定的占空比 */

#endif /* __LED_PWM_H */

/**
 ****************************************************************************************************
 * @file        beep.h
 * @brief       蜂鸣器驱动（PB8）  —— 阶段一 T1.4
 * @note        V2 板默认按【有源】蜂鸣器处理：GPIO 电平直接控制，高电平响。
 *              若实测为【无源】（给高电平不响、需方波才响），把下面的 BEEP_USE_PWM 置 1，
 *              内部自动改用 TIM4_CH3(PB8) PWM 驱动 —— 上层代码（alarm.c）完全不需要改动。
 ****************************************************************************************************
 */

#ifndef __BEEP_H
#define __BEEP_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"

#define BEEP_USE_PWM        0       /* 0 = 有源蜂鸣器（GPIO，默认）；1 = 无源蜂鸣器（PWM） */

void    beep_init(void);            /* 初始化，上电默认不响 */
void    beep_set(uint8_t on);       /* on=1 → 响，on=0 → 停 */
uint8_t beep_get(void);             /* 1 = 正在响，0 = 停 */

#endif /* __BEEP_H */

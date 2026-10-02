/**
 ****************************************************************************************************
 * @file        lsens.h
 * @brief       光敏传感器换算与滤波（板载 LS1，PF8 = ADC3_IN6）  —— 阶段一 T1.7
 ****************************************************************************************************
 */

#ifndef __LSENS_H
#define __LSENS_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"

#define LSENS_FILTER_LEN    8U      /* 8 点滑动平均，避免 LED3 与显示数值抖动 */

void    lsens_init(void);           /* 复位滤波器状态（ADC 由 adc3_dma_init() 初始化） */
uint8_t lsens_get(void);            /* 0 ~ 100：0 = 最暗，100 = 最亮 */

#endif /* __LSENS_H */

/**
 ****************************************************************************************************
 * @file        tp.h
 * @brief       电阻触摸屏驱动（XPT2046，软件 SPI）  —— 阶段二 T2.2
 *
 * @note        引脚（与《项目计划书v2》§2.4 冻结区一致）：
 *                  T_CLK = PB1    T_MISO = PB2    T_MOSI = PF9
 *                  T_PEN = PF10   T_CS   = PF11
 *              ⚠️ PB2 是 BOOT1，上电瞬间的电平别乱拉（代码里只在初始化后才驱动它）。
 *
 *              校准系数存在 24C02 的 0x40 起（13 字节），所以调用本模块前
 *              必须先 at24c02_init()（main.c 里已经调了）。
 ****************************************************************************************************
 */

#ifndef __TP_H
#define __TP_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"

/* 校准数据在 24C02 里的位置（0x40 起 13 字节：xfac4 + yfac4 + xc2 + yc2 + 标记1）*/
#define TP_CAL_ADDR         0x40U
#define TP_CAL_MAGIC        0x0AU       /* 已校准标记 */

typedef struct
{
    uint16_t x;             /* 屏幕坐标（已换算，0 ~ LCD_WIDTH-1）*/
    uint16_t y;             /* 屏幕坐标（已换算，0 ~ LCD_HEIGHT-1）*/
    uint8_t  pressed;       /* 1 = 当前被按下 */
} tp_point_t;

void    tp_init(void);                  /* 初始化触摸（GPIO + 从 24C02 读校准系数）*/
uint8_t tp_scan(tp_point_t *p);         /* 1 = 有有效触摸（p 里是屏幕坐标）；0 = 无 */
uint8_t tp_is_calibrated(void);         /* 1 = 校准数据有效（电容屏恒为 1）*/
uint8_t tp_is_capacitive(void);         /* 1 = 电容屏（GT9xxx 走 I2C）；0 = 电阻屏（XPT2046）*/
uint8_t tp_calibrate(void);             /* 五点校准：屏幕上依次点 5 个十字；0 = 成功 */
uint8_t tp_get_pen(void);               /* 诊断用：T_PEN 电平，1 = 未按下，0 = 按下 */

#endif /* __TP_H */

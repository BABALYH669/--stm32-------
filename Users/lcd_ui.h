/**
 ****************************************************************************************************
 * @file        lcd_ui.h
 * @brief       用 LCD 直接显示环境数据（**不使用 LVGL**）
 *
 * @note        屏幕：正点原子 3.5 寸 TFTLCD（ATK-MD0350 系列，320×480，FSMC 16 位并口）
 *              驱动：Drivers/BSP/LCD/lcd.c —— 来自官方「实验13 TFTLCD（MCU屏）」，
 *                    引脚与《项目计划书v2》§2.4 冻结区一致：
 *                    WR=PD5  RD=PD4  CS=PG12(NE4)  RS=PG0(A10)  BL=PB0
 *
 *              交互：电阻触摸屏（XPT2046，见 ./BSP/TP/tp.h）
 *                    · 左右滑动 → 上一页 / 下一页
 *                    · 点左半屏 / 右半屏 → 上一页 / 下一页（滑动的备用操作）
 *                    一页只显示一个传感器，顶部有网络/序号状态栏，底部有页码圆点。
 *
 *              ⚠️ 屏上文字只能用 ASCII（正点原子的 lcd_show_string 只带 ASCII 字库）。
 ****************************************************************************************************
 */

#ifndef __LCD_UI_H
#define __LCD_UI_H

#include "./SYSTEM/sys/sys.h"

void lcd_ui_init(void);         /* 初始化 LCD，并画出第 1 页 */
void lcd_ui_refresh(void);      /* 刷新当前页的数值与状态（建议 1 Hz 调用）*/
void lcd_ui_touch_poll(void);   /* 读触摸 + 滑动/点击切页（建议 10 ms 调用）*/

/* 手动切页：给按键用（触摸没调通时的备用操作，调试时也方便）*/
void lcd_ui_page_next(void);
void lcd_ui_page_prev(void);

#endif /* __LCD_UI_H */

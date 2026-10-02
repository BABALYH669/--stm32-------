/**
 ****************************************************************************************************
 * @file        key.h
 * @brief       5 方向按键驱动（上 / 下 / 左 / 右 / 确认）  —— 阶段一 T1.5
 * @note        2026-09-22 变更：摇杆已移除，改为 5 个按键。
 *
 *              物理映射（详见 app_config.h §5）：
 *                  KEY_UP    = 板载 KEY0  丝印 → PE4   低电平有效
 *                  KEY_DOWN  = 板载 KEY1  丝印 → PE3   低电平有效
 *                  KEY_LEFT  = 新接轻触键（插在原摇杆 X 的位置）→ PC0  低电平有效
 *                  KEY_RIGHT = 新接轻触键（插在原摇杆 Y 的位置）→ PC1  低电平有效
 *                  KEY_OK    = 板载 WK_UP 丝印 → PA0   高电平有效
 *
 *              完全非阻塞：消抖用 HAL_GetTick() 状态机实现，不含任何 HAL_Delay()。
 *
 *              事件模型（key_scan() 返回值 == 1 时代表一次事件）：
 *                  1. 按下沿            → 立刻报 1 次
 *                  2. 按住 ≥ 500 ms     → 每 150 ms 报 1 次（慢速连续滚动）
 *                  3. 按住 ≥ 1500 ms    → 改为每 60 ms 报 1 次（"连按"加速）
 *                  4. 松开              → 停止重复，不补报
 *              这是给阶段二 UI 在列表 / 阈值调节里连续滚动用的；阶段一的
 *              采集与报警不依赖按键。
 ****************************************************************************************************
 */

#ifndef __KEY_H
#define __KEY_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"

/* ---- 消抖 ---- */
#define KEY_SAMPLE_MS       20U     /* 采样间隔：20 ms                              */
#define KEY_DEBOUNCE_CNT    2U      /* 连续 2 次采样一致才确认电平                  */

/* ---- 长按重复 ---- */
#define KEY_LONG_PRESS_MS   500U    /* 按住超过 500 ms → 开始重复触发                */
#define KEY_REPEAT_MS       150U    /* 慢速段重复间隔                                */
#define KEY_ACCEL_PRESS_MS  1500U   /* 按住超过 1.5 s → 进入"连按"加速段              */
#define KEY_REPEAT_FAST_MS  60U     /* 加速段重复间隔                                */

typedef enum
{
    KEY_UP = 0,     /* 上   —— 板载 KEY0 丝印 (PE4)，低电平有效 */
    KEY_DOWN,       /* 下   —— 板载 KEY1 丝印 (PE3)，低电平有效 */
    KEY_LEFT,       /* 左   —— 新接 (PC0)，低电平有效           */
    KEY_RIGHT,      /* 右   —— 新接 (PC1)，低电平有效           */
    KEY_OK,         /* 确认 —— 板载 WK_UP 丝印 (PA0)，高电平有效 */
    KEY_MAX
} key_id_t;

void    key_init(void);                 /* 初始化 5 个按键 */
uint8_t key_scan(key_id_t id);          /* 返回 1 = 本次有一个按键事件（按下沿 或 长按重复）*/

#endif /* __KEY_H */

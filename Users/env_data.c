/**
 ****************************************************************************************************
 * @file        env_data.c
 * @brief       全局数据快照实现  —— 阶段一 T1.10
 ****************************************************************************************************
 */

#include "env_data.h"

static env_data_t s_env_data;       /* 静态存储区，无动态分配 */

/**
 * @brief       取全局数据快照
 * @param       无
 * @retval      指向唯一一份 env_data_t 的指针
 */
env_data_t *env_data_get(void)
{
    return &s_env_data;
}

/**
 * @brief       更新秒计数（主循环 1 Hz 调用一次）
 * @param       无
 * @retval      无
 * @note        ts 是"上电起的秒计数"。板上 RTC 若无 VBAT 电池，无法给出真实 Unix 时间，
 *              由电脑侧网关用接收时刻换算，这与《项目计划书v2》§7.2 的约定一致。
 */
void env_data_mark_ts(void)
{
    s_env_data.ts++;
}

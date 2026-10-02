/**
 ****************************************************************************************************
 * @file        env_data.h
 * @brief       全局数据快照 env_data_t  —— 阶段一 T1.10
 * @note        ⚠️ 本文件是给阶段二（UI 取数）和阶段三（任务间传数）的核心契约，签名不得改动。
 *              阶段一的架构是「裸机单线程」，因此 env_data_get() 直接返回可写指针是安全的；
 *              阶段三会改成 SensorTask → Queue → 各任务 的副本传递，届时本文件的签名仍不变。
 ****************************************************************************************************
 */

#ifndef __ENV_DATA_H
#define __ENV_DATA_H

#include "./SYSTEM/sys/sys.h"

/* valid 位掩码 */
#define ENV_VALID_LSENS     0x01        /* lux 有效        */
#define ENV_VALID_SMOKE     0x02        /* smoke_raw 有效  */
#define ENV_VALID_DHT11     0x04        /* temp/humi 有效  */

typedef enum
{
    ALARM_NONE = 0,     /* 所有量都在阈值内                     */
    ALARM_SMOKE,        /* 烟雾超上限                           */
    ALARM_TEMP,         /* 温度超上限                           */
    ALARM_HUMI,         /* 湿度低于下限                         */
    ALARM_MULTI         /* 烟雾与温湿度同时越限                 */
} alarm_state_t;

typedef struct
{
    uint8_t       valid;        /* ENV_VALID_* 位掩码                     */
    uint8_t       lux;          /* 0~100，0 = 最暗，100 = 最亮            */
    uint16_t      smoke_raw;    /* 烟雾原始 ADC 值                        */
    uint16_t      smoke_rel;    /* 相对基线后的值（用于阈值判定与上报）    */
    int16_t       temp_x10;     /* 温度 ×10，251 = 25.1℃                  */
    uint16_t      humi_x10;     /* 湿度 ×10，601 = 60.1%RH                */
    /* 2026-09-22：摇杆已移除，原 joy_x / joy_y「预留」字段一并删除，
       改用「上/下/左/右/确认」5 个按键（见 ./BSP/KEY/key.h） */
    alarm_state_t alarm;        /* 当前报警状态                           */
    uint32_t      ts;           /* 秒计数（上电起，1 Hz 递增）             */
    uint32_t      seq;          /* 单调递增序号，网关按此去重             */
} env_data_t;

env_data_t *env_data_get(void);         /* 返回可写指针（单线程裸机安全） */
void        env_data_mark_ts(void);     /* 更新 ts（1 Hz 调用）           */

#endif /* __ENV_DATA_H */

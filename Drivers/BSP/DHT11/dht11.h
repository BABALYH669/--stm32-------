/**
 ****************************************************************************************************
 * @file        dht11.h
 * @brief       DHT11 单总线数字温湿度（板载 U4 口，PG11）  —— 阶段一 T1.8
 * @note        位时序为微秒级，读取期间必须关中断；整个位读取段耗时 < 5 ms。
 *              上层一律用 dht11_read()，不要直接碰 GPIO。
 ****************************************************************************************************
 */

#ifndef __DHT11_H
#define __DHT11_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"

/* dht11_read() 的失败码 */
#define DHT11_OK            0       /* 读取成功                    */
#define DHT11_ERR_NULL      1       /* 入参为 NULL                 */
#define DHT11_ERR_NO_ACK    2       /* 无应答 / 位时序超时         */
#define DHT11_ERR_CRC       3       /* 校验和错误                  */
#define DHT11_ERR_RETRY     4       /* 连续 3 次重试全部失败       */

typedef struct
{
    int16_t  temp_x10;      /* 温度 ×10，251 = 25.1℃ */
    uint16_t humi_x10;      /* 湿度 ×10，601 = 60.1%RH */
    uint8_t  ok;            /* 1 = 本次数据有效 */
} dht11_data_t;

void    dht11_init(void);                       /* 初始化 DQ 引脚（开漏 + 上拉） */
uint8_t dht11_read(dht11_data_t *out);          /* 0 = 成功；非 0 = 失败码，调用方跳过本轮 */

#endif /* __DHT11_H */

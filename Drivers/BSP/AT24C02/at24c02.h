/**
 ****************************************************************************************************
 * @file        at24c02.h
 * @brief       阈值持久化（板载 24C02，软件 IIC：PB6=SCL / PB7=SDA）  —— 阶段一 T1.9
 * @note        1. 阈值结构体存放在 24C02 地址 0x00 起；0x40 起预留给阶段二的触摸校准系数，
 *                 两段互不重叠。
 *              2. 结构体带 magic + ver + CRC16(Modbus)；magic 不符或 CRC 错误一律回落默认值，
 *                 且【不回写】—— 保证只读不改。
 *              3. 24C02 只有 256 B，写入必须按 8 字节页边界分页，页间等待写周期。
 ****************************************************************************************************
 */

#ifndef __AT24C02_H
#define __AT24C02_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"

#define THRESH_MAGIC        0xA55AU
#define THRESH_VER          1U
#define THRESH_ADDR         0x00U           /* 阈值区起始地址（0x00 ~ 0x1F）*/
#define AT24C02_CAL_ADDR    0x40U           /* 阶段二触摸校准系数起始地址（本阶段不使用）*/

typedef struct
{
    uint16_t magic;             /* 0xA55A                       */
    uint8_t  ver;               /* 1                            */
    uint16_t th_smoke_hi;       /* 烟雾相对上限                 */
    int16_t  th_temp_hi_x10;    /* 温度上限 ×10（350 = 35.0℃）  */
    uint16_t th_humi_lo_x10;    /* 湿度下限 ×10（300 = 30.0%RH）*/
    uint16_t hyst[3];           /* 迟滞：[0]烟雾 [1]温度×10 [2]湿度×10 */
    uint16_t crc16;             /* Modbus CRC16，覆盖本结构体 crc16 之前的全部字节 */
} thresh_t;

void    at24c02_init(void);                 /* 初始化软件 IIC 并探测器件 */
uint8_t at24c02_present(void);              /* 1 = 器件在线（ACK 正常） */
void    thresh_set_default(thresh_t *t);    /* 用 app_config.h 的默认值填充 */
uint8_t thresh_load(thresh_t *t);           /* 0 = 成功且 CRC 正确；非 0 = 失败（调用方用默认值）*/
uint8_t thresh_save(const thresh_t *t);     /* 0 = 写入成功 */

/* 【新增】通用字节读写，给触摸校准等模块用（阶段二 T2.2 把校准系数放在 0x40 起）。
 * 返回 0 = 成功，非 0 = 器件无应答；写入内部按 8 字节页管理并等写周期，可跨页调用。 */
uint8_t at24c02_read(uint16_t addr, uint8_t *pbuf, uint16_t len);
uint8_t at24c02_write(uint16_t addr, const uint8_t *pbuf, uint16_t len);

#endif /* __AT24C02_H */

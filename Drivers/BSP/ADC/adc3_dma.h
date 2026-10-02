/**
 ****************************************************************************************************
 * @file        adc3_dma.h
 * @brief       ADC3 双通道规则扫描 + DMA2_Channel5 循环搬运  —— 阶段一 T1.6
 * @note        为什么必须全落在 ADC3 上：光敏固定在 PF8 = ADC3_IN6 改不了，
 *              而 F103 的 ADC→DMA 映射是 ADC1→DMA1_CH1、ADC3→DMA2_CH5（ADC2 不能单独走 DMA）。
 *              所以烟雾 + 光敏全部排在 ADC3 的规则序列里，一条 DMA 扫完。
 *              PA5/PA6/PA7 属于 ADC1/ADC2，不能与光敏共用 DMA —— 这是最容易踩的坑。
 *              （2026-09-22：摇杆已移除，序列从 4 通道缩到 2 通道，DMA 长度随之变短。）
 ****************************************************************************************************
 */

#ifndef __ADC3_DMA_H
#define __ADC3_DMA_H

#include "./SYSTEM/sys/sys.h"
#include "app_config.h"

/* 扫描顺序（rank 1~2），顺序固定，勿改 */
typedef enum
{
    ADC3_IDX_SMOKE = 0,     /* rank1  CH4  PF6  烟雾 AO  */
    ADC3_IDX_LSENS,         /* rank2  CH6  PF8  光敏 AO  */
    ADC3_IDX_MAX            /* = 2（摇杆已移除）          */
} adc3_idx_t;

#define ADC3_AVG_TIMES      8U      /* 去极值平均：连续取 8 次，去掉最大最小后平均 */

extern volatile uint16_t g_adc3_raw[ADC3_IDX_MAX];      /* DMA 目标缓冲，主循环直接读 */

void     adc3_dma_init(void);               /* 配置 ADC3 + DMA2_CH5（不启动转换） */
void     adc3_dma_start(void);              /* 启动后由 DMA 循环搬运，不需再干预 */
uint16_t adc3_get(adc3_idx_t idx);          /* 返回去极值平均后的转换值 */

#endif /* __ADC3_DMA_H */

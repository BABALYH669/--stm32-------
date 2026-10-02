/**
 ****************************************************************************************************
 * @file        lsens.c
 * @brief       光敏传感器换算与滤波（板载 LS1，PF8 = ADC3_IN6）  —— 阶段一 T1.7
 * @note        换算公式对齐正点原子例程：原始值 / 40 → 钳位到 100 → 取 100 - 值（0 最暗）。
 *              在此基础上先对"原始值"做 8 点滑动平均，再换算，避免 0~100 量化带来台阶抖动。
 ****************************************************************************************************
 */

#include "./BSP/LSENS/lsens.h"
#include "./BSP/ADC/adc3_dma.h"

static uint16_t s_lsens_buf[LSENS_FILTER_LEN];
static uint32_t s_lsens_sum = 0;
static uint8_t  s_lsens_idx = 0;        /* 环形写入位置 */
static uint8_t  s_lsens_cnt = 0;        /* 已填充的有效样本数 */

/**
 * @brief       复位光强滤波器
 * @param       无
 * @retval      无
 */
void lsens_init(void)
{
    uint8_t i;

    for (i = 0; i < LSENS_FILTER_LEN; i++)
    {
        s_lsens_buf[i] = 0;
    }

    s_lsens_sum = 0;
    s_lsens_idx = 0;
    s_lsens_cnt = 0;
}

/**
 * @brief       读取光强
 * @param       无
 * @retval      0 ~ 100：0 = 最暗，100 = 最亮
 */
uint8_t lsens_get(void)
{
    uint32_t raw;
    uint32_t temp_val;

    /* 滑动平均：先减去最旧样本，再写入最新样本 */
    s_lsens_sum -= s_lsens_buf[s_lsens_idx];
    s_lsens_buf[s_lsens_idx] = adc3_get(ADC3_IDX_LSENS);
    s_lsens_sum += s_lsens_buf[s_lsens_idx];

    s_lsens_idx = (uint8_t)((s_lsens_idx + 1U) % LSENS_FILTER_LEN);
    if (s_lsens_cnt < LSENS_FILTER_LEN)
    {
        s_lsens_cnt++;
    }

    raw = s_lsens_sum / s_lsens_cnt;

    temp_val = raw / 40U;
    if (temp_val > 100U)
    {
        temp_val = 100U;
    }

    return (uint8_t)(100U - temp_val);
}

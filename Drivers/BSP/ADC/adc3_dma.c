/**
 ****************************************************************************************************
 * @file        adc3_dma.c
 * @brief       ADC3 双通道规则扫描 + DMA2_Channel5 循环搬运  —— 阶段一 T1.6
 * @note        本阶段不使用 DMA 中断（不使能 DMA2_Channel5_IRQn），主循环轮询读缓冲即可。
 ****************************************************************************************************
 */

#include "./BSP/ADC/adc3_dma.h"

volatile uint16_t g_adc3_raw[ADC3_IDX_MAX];     /* DMA 目标缓冲 */

static ADC_HandleTypeDef g_adc3_handle;
static DMA_HandleTypeDef g_dma_adc3_handle;

/**
 * @brief       初始化 ADC3 的四通道扫描与 DMA2_Channel5
 * @note        1) 采样时间取 ADC_SAMPLETIME_239CYCLES_5（MQ 烟雾传感器内阻高，需要长采样）
 *              2) ADC 时钟 = 72MHz / 6 = 12MHz
 *              3) DMA 为循环模式、外设→内存、半字、内存地址递增
 * @param       无
 * @retval      无
 */
void adc3_dma_init(void)
{
    GPIO_InitTypeDef          gpio_init_struct;
    ADC_ChannelConfTypeDef    adc_ch_conf;
    RCC_PeriphCLKInitTypeDef  adc_clk_init = {0};
    uint8_t                   i;
    static const uint32_t     s_adc3_ch[ADC3_IDX_MAX] =
    {
        SMOKE_AO_CH,        /* ADC3_IDX_SMOKE */
        LSENS_AO_CH,        /* ADC3_IDX_LSENS */
    };

    /* ---- 1. 时钟 ---- */
    __HAL_RCC_GPIOF_CLK_ENABLE();       /* 模拟脚只剩 PF6 / PF8，都在 GPF */
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    /* 注意：PC0/PC1 已改作「左/右」方向按键（数字输入），GPIOC 时钟由 key.c 使能，这里不碰 */

    adc_clk_init.PeriphClockSelection = RCC_PERIPHCLK_ADC;      /* ADC 外设时钟 */
    adc_clk_init.AdcClockSelection    = RCC_ADCPCLK2_DIV6;      /* 72MHz/6 = 12MHz */
    HAL_RCCEx_PeriphCLKConfig(&adc_clk_init);

    /* ---- 2. 2 个模拟输入引脚（烟雾 PF6 + 光敏 PF8）---- */
    gpio_init_struct.Mode  = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull  = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;       /* 模拟模式不关心速度，填个确定值 */
    gpio_init_struct.Pin   = SMOKE_AO_PIN | LSENS_AO_PIN;
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);

    /* ---- 3. DMA2_Channel5：外设→内存，循环模式 ---- */
    g_dma_adc3_handle.Instance                 = DMA2_Channel5;
    g_dma_adc3_handle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    g_dma_adc3_handle.Init.PeriphInc           = DMA_PINC_DISABLE;      /* 外设地址不递增（ADC->DR）*/
    g_dma_adc3_handle.Init.MemInc              = DMA_MINC_ENABLE;       /* 内存地址递增        */
    g_dma_adc3_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    g_dma_adc3_handle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    g_dma_adc3_handle.Init.Mode                = DMA_CIRCULAR;          /* 循环搬运，永不停 */
    g_dma_adc3_handle.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&g_dma_adc3_handle);

    /* ---- 4. ADC3 本体：扫描 + 连续转换 ---- */
    g_adc3_handle.Instance                   = ADC3;
    g_adc3_handle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    g_adc3_handle.Init.ScanConvMode          = ADC_SCAN_ENABLE;         /* 扫描模式 */
    g_adc3_handle.Init.ContinuousConvMode    = ENABLE;                 /* 连续转换 */
    g_adc3_handle.Init.NbrOfConversion       = ADC3_IDX_MAX;            /* 规则序列 2 个 */
    g_adc3_handle.Init.DiscontinuousConvMode = DISABLE;
    g_adc3_handle.Init.NbrOfDiscConversion   = 0;
    g_adc3_handle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;      /* 软件触发 */
    HAL_ADC_Init(&g_adc3_handle);

    HAL_ADCEx_Calibration_Start(&g_adc3_handle);                        /* ADC 校准 */

    /* ---- 5. 规则序列：rank1~2 对应 g_adc3_raw[0..1] ---- */
    adc_ch_conf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    for (i = 0; i < ADC3_IDX_MAX; i++)
    {
        adc_ch_conf.Channel = s_adc3_ch[i];
        adc_ch_conf.Rank    = (uint32_t)(ADC_REGULAR_RANK_1 + i);
        HAL_ADC_ConfigChannel(&g_adc3_handle, &adc_ch_conf);
    }

    /* 注意：本阶段不使能 DMA2_Channel5_IRQn，转换完成由主循环轮询缓冲 */
}

/**
 * @brief       启动 ADC3 + DMA 循环搬运
 * @note        顺序很关键：必须先启动 DMA、再打开 ADC 的 DMA 请求位、最后触发转换。
 *              若先触发转换，第一次 EOC 无人搬运，DMA 缓冲会永久错位一格（通道串位）。
 * @param       无
 * @retval      无
 */
void adc3_dma_start(void)
{
    HAL_DMA_Start(&g_dma_adc3_handle,
                  (uint32_t)&ADC3->DR,
                  (uint32_t)g_adc3_raw,
                  ADC3_IDX_MAX);

    SET_BIT(ADC3->CR2, ADC_CR2_DMA);        /* 打开 ADC 的 DMA 请求 */

    HAL_ADC_Start(&g_adc3_handle);          /* 使能 ADC 并软件触发；连续模式下会一直循环 */
}

/**
 * @brief       读取某个通道的值（去极值平均）
 * @param       idx: ADC3_IDX_SMOKE / ADC3_IDX_LSENS
 * @retval      去极值平均后的 12 位转换值
 * @note        DMA 在后台不断刷新 g_adc3_raw，因此连续 8 次读取天然就是 8 个不同的采样点。
 *              取 8 次 → 排序 → 丢掉最大最小 → 剩下 6 个求平均，抑制单次尖峰噪声。
 */
uint16_t adc3_get(adc3_idx_t idx)
{
    uint16_t sample[ADC3_AVG_TIMES];
    uint32_t sum = 0;
    uint16_t tmp;
    uint8_t  i, j;

    if (idx >= ADC3_IDX_MAX)
    {
        return 0;
    }

    for (i = 0; i < ADC3_AVG_TIMES; i++)
    {
        sample[i] = g_adc3_raw[idx];
    }

    /* 插入排序（只有 8 个元素，够快且不占栈） */
    for (i = 1; i < ADC3_AVG_TIMES; i++)
    {
        tmp = sample[i];
        j = i;
        while (j > 0 && sample[j - 1] > tmp)
        {
            sample[j] = sample[j - 1];
            j--;
        }
        sample[j] = tmp;
    }

    for (i = 1; i < (ADC3_AVG_TIMES - 1); i++)      /* 去掉 sample[0] 与 sample[7] */
    {
        sum += sample[i];
    }

    return (uint16_t)(sum / (ADC3_AVG_TIMES - 2));
}

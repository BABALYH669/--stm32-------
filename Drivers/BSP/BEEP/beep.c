/**
 ****************************************************************************************************
 * @file        beep.c
 * @brief       蜂鸣器驱动（PB8）  —— 阶段一 T1.4
 ****************************************************************************************************
 */

#include "./BSP/BEEP/beep.h"

static uint8_t g_beep_on = 0;       /* 当前响/停状态 */

#if BEEP_USE_PWM
/* ------------------------------------------------------------------------------------------------
 * 无源蜂鸣器分支：TIM4_CH3 = PB8
 * 72MHz / (71+1) = 1MHz → 1MHz / (369+1) = 2.7kHz，人耳最容易听到的频段之一
 * ---------------------------------------------------------------------------------------------- */
#define BEEP_TIM                    TIM4
#define BEEP_TIM_CHANNEL            TIM_CHANNEL_3
#define BEEP_TIM_PRESCALER          (72U - 1U)
#define BEEP_TIM_PERIOD             (370U - 1U)

static TIM_HandleTypeDef g_beep_handle;

void beep_init(void)
{
    GPIO_InitTypeDef   gpio_init_struct;
    TIM_OC_InitTypeDef tim_oc_init_struct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    gpio_init_struct.Pin   = BEEP_PIN;
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;               /* 复用推挽：TIM4_CH3 */
    gpio_init_struct.Pull  = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BEEP_PORT, &gpio_init_struct);

    g_beep_handle.Instance               = BEEP_TIM;
    g_beep_handle.Init.Prescaler         = BEEP_TIM_PRESCALER;
    g_beep_handle.Init.Period            = BEEP_TIM_PERIOD;
    g_beep_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    g_beep_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    g_beep_handle.Init.RepetitionCounter = 0;
    g_beep_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&g_beep_handle);

    tim_oc_init_struct.OCMode       = TIM_OCMODE_PWM1;
    tim_oc_init_struct.Pulse        = BEEP_TIM_PERIOD / 2;  /* 50% 占空比，最响 */
    tim_oc_init_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;
    tim_oc_init_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    tim_oc_init_struct.OCFastMode   = TIM_OCFAST_DISABLE;
    tim_oc_init_struct.OCIdleState  = TIM_OCIDLESTATE_RESET;
    tim_oc_init_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_beep_handle, &tim_oc_init_struct, BEEP_TIM_CHANNEL);

    /* 先不启动 PWM，保持默认不响 */
    __HAL_TIM_SET_COMPARE(&g_beep_handle, BEEP_TIM_CHANNEL, 0);
    g_beep_on = 0;
}

void beep_set(uint8_t on)
{
    if (on)
    {
        HAL_TIM_PWM_Start(&g_beep_handle, BEEP_TIM_CHANNEL);
        __HAL_TIM_SET_COMPARE(&g_beep_handle, BEEP_TIM_CHANNEL, BEEP_TIM_PERIOD / 2);
        g_beep_on = 1;
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&g_beep_handle, BEEP_TIM_CHANNEL, 0);
        HAL_TIM_PWM_Stop(&g_beep_handle, BEEP_TIM_CHANNEL);
        g_beep_on = 0;
    }
}

#else
/* ------------------------------------------------------------------------------------------------
 * 有源蜂鸣器分支（默认）：PB8 推挽输出，高电平响
 * ---------------------------------------------------------------------------------------------- */
void beep_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio_init_struct.Pin   = BEEP_PIN;
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull  = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BEEP_PORT, &gpio_init_struct);

    beep_set(0);        /* 上电默认不响 */
}

void beep_set(uint8_t on)
{
    g_beep_on = (on != 0) ? 1 : 0;
    HAL_GPIO_WritePin(BEEP_PORT, BEEP_PIN, g_beep_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
#endif

/**
 * @brief       读取蜂鸣器当前状态（供 alarm.c 做幂等设置，避免重复写 IO）
 * @param       无
 * @retval      1 = 正在响，0 = 停
 */
uint8_t beep_get(void)
{
    return g_beep_on;
}

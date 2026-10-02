/**
 ****************************************************************************************************
 * @file        led_pwm.c
 * @brief       LED3（PA6 = TIM3_CH1）PWM 调光驱动  —— 阶段一 T1.3
 * @note        参数固定：Prescaler = 71、Period = 999
 *              72 MHz / (71+1) = 1 MHz 计数时钟；1 MHz / (999+1) = 1 kHz PWM
 *              1 kHz 既无可见闪烁，也避开了可听噪声频段。
 ****************************************************************************************************
 */

#include "./BSP/LED_PWM/led_pwm.h"

#define LED_PWM_TIM                     TIM3
#define LED_PWM_TIM_CHANNEL             TIM_CHANNEL_1
#define LED_PWM_TIM_PRESCALER           (72U - 1U)      /* 72MHz/72 = 1MHz   */
#define LED_PWM_TIM_PERIOD              (LED_PWM_FULL - 1U)

#define LED_PWM_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)
#define LED_PWM_TIM_CLK_ENABLE()        do{ __HAL_RCC_TIM3_CLK_ENABLE();  }while(0)

static TIM_HandleTypeDef g_led_pwm_handle;
static uint16_t          g_led_pwm_duty = 0;    /* 最近一次设定的占空比 */

/**
 * @brief       初始化 LED3 的 PWM 输出
 * @note        PA6 配为复用推挽输出；只在本函数里 Init 一次，
 *              led_pwm_set() 只改写 CCR1，不得反复 Init。
 * @param       无
 * @retval      无
 */
void led_pwm_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    TIM_OC_InitTypeDef tim_oc_init_struct;

    LED_PWM_GPIO_CLK_ENABLE();
    LED_PWM_TIM_CLK_ENABLE();

    /* PA6 = TIM3_CH1（默认复用，无需 AFIO 重映射） */
    gpio_init_struct.Pin   = LED3_PIN;
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;           /* 复用推挽输出 */
    gpio_init_struct.Pull  = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LED3_PORT, &gpio_init_struct);

    /* 时基：1 kHz */
    g_led_pwm_handle.Instance               = LED_PWM_TIM;
    g_led_pwm_handle.Init.Prescaler         = LED_PWM_TIM_PRESCALER;
    g_led_pwm_handle.Init.Period            = LED_PWM_TIM_PERIOD;
    g_led_pwm_handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    g_led_pwm_handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    g_led_pwm_handle.Init.RepetitionCounter = 0;
    g_led_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&g_led_pwm_handle);

    /* 输出比较：极性 HIGH，占空比 = CCR1 / (ARR+1)，便于用示波器直接比对设定值 */
    tim_oc_init_struct.OCMode       = TIM_OCMODE_PWM1;
    tim_oc_init_struct.Pulse        = 0;                /* 初始全灭 */
    tim_oc_init_struct.OCPolarity   = TIM_OCPOLARITY_HIGH;
    tim_oc_init_struct.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    tim_oc_init_struct.OCFastMode   = TIM_OCFAST_DISABLE;
    tim_oc_init_struct.OCIdleState  = TIM_OCIDLESTATE_RESET;
    tim_oc_init_struct.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_led_pwm_handle, &tim_oc_init_struct, LED_PWM_TIM_CHANNEL);

    HAL_TIM_PWM_Start(&g_led_pwm_handle, LED_PWM_TIM_CHANNEL);

    g_led_pwm_duty = 0;
}

/**
 * @brief       设置 LED3 占空比
 * @param       duty: 0 ~ 1000（0 = 全灭，1000 = 全亮）
 * @retval      无
 */
void led_pwm_set(uint16_t duty)
{
    if (duty > LED_PWM_FULL)
    {
        duty = LED_PWM_FULL;
    }

    g_led_pwm_duty = duty;

    /* 只改写 CCR1，不动时基 */
    __HAL_TIM_SET_COMPARE(&g_led_pwm_handle, LED_PWM_TIM_CHANNEL, duty);
}

/**
 * @brief       读取最近一次设定的占空比
 * @param       无
 * @retval      0 ~ 1000
 */
uint16_t led_pwm_get(void)
{
    return g_led_pwm_duty;
}

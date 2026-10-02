/**
 ****************************************************************************************************
 * @file        led.c
 * @brief       LED0 / LED1 / LED2 / LED4 驱动（GPIO，低电平点亮）  —— 阶段一 T1.2
 ****************************************************************************************************
 */

#include "./BSP/LED/led.h"

/* 4 路灯涉及端口的 GPIO 时钟使能（引脚号本身在 app_config.h 中定义） */
#define LED_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOB_CLK_ENABLE();   \
                                        __HAL_RCC_GPIOE_CLK_ENABLE();   \
                                        __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)

/* 灯编号 → 引脚映射表，顺序必须与 led_id_t 一致 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} led_hw_t;

static const led_hw_t s_led_hw[LED_ID_MAX] =
{
    { LED0_PORT, LED0_PIN },
    { LED1_PORT, LED1_PIN },
    { LED2_PORT, LED2_PIN },
    { LED4_PORT, LED4_PIN },
};

/**
 * @brief       初始化 4 路指示灯
 * @note        推挽输出 / 无上下拉 / 50MHz；上电默认全部熄灭（输出高电平）
 * @param       无
 * @retval      无
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    uint8_t i;

    LED_GPIO_CLK_ENABLE();

    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;       /* 推挽输出   */
    gpio_init_struct.Pull  = GPIO_NOPULL;               /* 无上下拉   */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;      /* 50MHz      */

    for (i = 0; i < LED_ID_MAX; i++)
    {
        gpio_init_struct.Pin = s_led_hw[i].pin;
        HAL_GPIO_Init(s_led_hw[i].port, &gpio_init_struct);

        /* 低电平点亮，所以默认熄灭要写高电平 */
        HAL_GPIO_WritePin(s_led_hw[i].port, s_led_hw[i].pin, GPIO_PIN_SET);
    }
}

/**
 * @brief       点亮指定灯
 * @param       id: LED_ID_0 / LED_ID_1 / LED_ID_2 / LED_ID_4
 * @retval      无
 */
void led_on(led_id_t id)
{
    if (id >= LED_ID_MAX)
    {
        return;
    }

    HAL_GPIO_WritePin(s_led_hw[id].port, s_led_hw[id].pin, GPIO_PIN_RESET);     /* 低电平点亮 */
}

/**
 * @brief       熄灭指定灯
 * @param       id: LED_ID_0 / LED_ID_1 / LED_ID_2 / LED_ID_4
 * @retval      无
 */
void led_off(led_id_t id)
{
    if (id >= LED_ID_MAX)
    {
        return;
    }

    HAL_GPIO_WritePin(s_led_hw[id].port, s_led_hw[id].pin, GPIO_PIN_SET);
}

/**
 * @brief       翻转指定灯
 * @param       id: LED_ID_0 / LED_ID_1 / LED_ID_2 / LED_ID_4
 * @retval      无
 */
void led_toggle(led_id_t id)
{
    if (id >= LED_ID_MAX)
    {
        return;
    }

    HAL_GPIO_TogglePin(s_led_hw[id].port, s_led_hw[id].pin);
}

/**
 * @brief       设置指定灯的亮灭
 * @param       id: LED_ID_0 / LED_ID_1 / LED_ID_2 / LED_ID_4
 * @param       on: 1 = 点亮，0 = 熄灭
 * @retval      无
 */
void led_set(led_id_t id, uint8_t on)
{
    if (on)
    {
        led_on(id);
    }
    else
    {
        led_off(id);
    }
}

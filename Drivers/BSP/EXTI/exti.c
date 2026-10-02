#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/BEEP/beep.h"
#include "./BSP/KEY/key.h"
#include "./BSP/EXTI/exti.h"
/************************************************************/
//设置中断服务函数
void KEY1_INT_IRQHandler(void)
{
    //中断处理公共函数
    HAL_GPIO_EXTI_IRQHandler(KEY1_INT_GPIO_PIN);
    //推出回调函数前再强制清空一次标志
    //彻底杜绝抖动带来的重复触发
    __HAL_GPIO_EXTI_CLEAR_IT(KEY1_INT_GPIO_PIN);
}

void KEY0_INT_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(KEY0_INT_GPIO_PIN);
    __HAL_GPIO_EXTI_CLEAR_IT(KEY0_INT_GPIO_PIN);
}

void KEY_UP_INT_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(KEY_UP_INT_GPIO_PIN);
    __HAL_GPIO_EXTI_CLEAR_IT(KEY_UP_INT_GPIO_PIN);
}

/************************************************************/
//Callback函数,用于写自己的逻辑的函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    delay_ms(20);      /* 消抖 */
    switch(GPIO_Pin)
    {
        case KEY0_INT_GPIO_PIN:
            if (KEY0 == 0)
            {
                LED0_TOGGLE();  /* LED0 状态取反 */ 
                LED1_TOGGLE();  /* LED1 状态取反 */ 
            }
            break;
        case KEY1_INT_GPIO_PIN:
            if (KEY1 == 0)
            {
                LED0_TOGGLE();  /* LED0 状态取反 */ 
            }
            break;
        case KEY_UP_INT_GPIO_PIN:
            if (KEY_UP == 1)
            {
                BEEP_TOGGLE();  /* 蜂鸣器状态取反 */ 
            }
            break;
    }
}
/************************************************************/
void extix_init(void)
{
    //初始化GPIO结构体
    GPIO_InitTypeDef gpio_init_struct;
    //使能GPIO时钟
    KEY1_INT_GPIO_CLK_ENABLE();
    KEY0_INT_GPIO_CLK_ENABLE();
    KEY_UP_INT_GPIO_CLK_ENABLE();
    //给了上拉电阻就使用下降沿触发
    gpio_init_struct.Pin = KEY1_INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY1_INT_GPIO_PORT,&gpio_init_struct);
    
    gpio_init_struct.Pin = KEY0_INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_FALLING;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY0_INT_GPIO_PORT,&gpio_init_struct);
    //给了下拉电阻就使用上升沿触发
    gpio_init_struct.Pin = KEY_UP_INT_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_IT_RISING;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(KEY_UP_INT_GPIO_PORT,&gpio_init_struct);
    //设置中断分组
    //已经自动设置无需再次设置,默认是[2:2]分组
    
    //设置中断优先级 抢断优先级数字越小优先级越高,响应优先级同理
    HAL_NVIC_SetPriority(KEY0_INT_IRQn,0,2);
    HAL_NVIC_SetPriority(KEY1_INT_IRQn,1,2);
    HAL_NVIC_SetPriority(KEY_UP_INT_IRQn,2,2);
    //使能中断
    HAL_NVIC_EnableIRQ(KEY0_INT_IRQn);
    HAL_NVIC_EnableIRQ(KEY1_INT_IRQn);
    HAL_NVIC_EnableIRQ(KEY_UP_INT_IRQn);
}


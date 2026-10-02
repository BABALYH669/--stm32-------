#ifndef __EXTI_H
#define __EXTI_H

#include "./SYSTEM/sys/sys.h"
//INT是interrupt(中断)的缩写,标注此GPIO端口是用于终端功能的KEY0引脚
#define KEY0_INT_GPIO_PORT  GPIOE
#define KEY0_INT_GPIO_PIN   GPIO_PIN_4
#define KEY0_INT_GPIO_CLK_ENABLE()  do{__HAL_RCC_GPIOE_CLK_ENABLE();}while(0)
#define KEY0_INT_IRQn   EXTI4_IRQn //中断编号
#define KEY0_INT_IRQHandler     EXTI4_IRQHandler //中断服务函数

#define KEY1_INT_GPIO_PORT  GPIOE
#define KEY1_INT_GPIO_PIN   GPIO_PIN_3
#define KEY1_INT_GPIO_CLK_ENABLE()  do{__HAL_RCC_GPIOE_CLK_ENABLE();}while(0)
#define KEY1_INT_IRQn   EXTI3_IRQn      //中断编号
#define KEY1_INT_IRQHandler     EXTI3_IRQHandler //中断服务函数

#define KEY_UP_INT_GPIO_PORT  GPIOA
#define KEY_UP_INT_GPIO_PIN   GPIO_PIN_0
#define KEY_UP_INT_GPIO_CLK_ENABLE()  do{__HAL_RCC_GPIOA_CLK_ENABLE();}while(0)
#define KEY_UP_INT_IRQn   EXTI0_IRQn
#define KEY_UP_INT_IRQHandler     EXTI0_IRQHandler

void extix_init(void);
#endif


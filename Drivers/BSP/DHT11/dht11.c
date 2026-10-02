/**
 ****************************************************************************************************
 * @file        dht11.c
 * @brief       DHT11 单总线数字温湿度（板载 U4 口，PG11）  —— 阶段一 T1.8
 * @note        1. DQ 配为「开漏输出 + 上拉」：输出 1 时引脚被释放，可直接当输入读，
 *                 因此不需要来回切换输入/输出方向。
 *              2. 复位脉冲（拉低 ≥18 ms）在中断开启状态下完成，只需普通延时；
 *                 真正不能被打断的是「主机释放总线 → 从机应答 → 40 位数据」这一段，
 *                 所以 __disable_irq() 只包住这一段（实测约 4 ms，< 5 ms）。
 *              3. 关键时序：
 *                   主机拉低 ≥18ms → 释放 → 从机拉低 80us → 从机拉高 80us → 40 位数据
 *                   每 bit：50us 低电平 + 26~28us(0) / 70us(1) 高电平
 *                   在上升沿之后延时 40us 采样：仍为高 → 1，已变低 → 0
 ****************************************************************************************************
 */

#include "./BSP/DHT11/dht11.h"
#include "./SYSTEM/delay/delay.h"

#define DHT11_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)

/* 用 if/else 而不是三元：避免三元运算符右结合在 x 是表达式时产生 int/void 类型错误（同 at24c02.c）*/
#define DHT11_DQ_OUT(x)             do{ if (x) { HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET); } \
                                        else    { HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET); } \
                                    }while(0)                       /* 数据端口输出 */
#define DHT11_DQ_IN                 HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)     /* 数据端口输入 */

#define DHT11_TIMEOUT_US            100U    /* 单步等待超时（us） */
#define DHT11_RETRY_MAX             3U      /* 一轮最多尝试 3 次，失败即跳过本轮 */

/**
 * @brief       等待 DQ 变成指定电平
 * @param       level: 0 = 等低电平，1 = 等高电平
 * @param       timeout_us: 超时时间（us）
 * @retval      0 = 已到达指定电平；1 = 超时
 */
static uint8_t dht11_wait_level(uint8_t level, uint32_t timeout_us)
{
    while (timeout_us--)
    {
        if ((DHT11_DQ_IN == GPIO_PIN_SET ? 1U : 0U) == level)
        {
            return 0;
        }

        delay_us(1);
    }

    return 1;
}

/**
 * @brief       读取一个位
 * @note        必须在关中断状态下调用
 * @param       无
 * @retval      0 / 1
 */
static uint8_t dht11_read_bit(void)
{
    /* 等本位的 50us 低电平结束（即上升沿） */
    if (dht11_wait_level(0, DHT11_TIMEOUT_US))
    {
        return 0;
    }

    if (dht11_wait_level(1, DHT11_TIMEOUT_US))
    {
        return 0;
    }

    delay_us(40);               /* 上升沿后 40us 采样：仍为高 = 1（70us），已变低 = 0（26~28us） */

    return (DHT11_DQ_IN == GPIO_PIN_SET) ? 1U : 0U;
}

/**
 * @brief       读取一个字节（高位先出）
 * @note        必须在关中断状态下调用
 * @param       无
 * @retval      读到的字节
 */
static uint8_t dht11_read_byte(void)
{
    uint8_t i;
    uint8_t data = 0;

    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        data |= dht11_read_bit();
    }

    return data;
}

/**
 * @brief       单次读取温湿度
 * @note        关中断只覆盖「应答 + 40 位数据」这一段
 * @param       out: 输出数据
 * @retval      0 = 成功；非 0 = 失败码
 */
static uint8_t dht11_read_once(dht11_data_t *out)
{
    uint8_t buf[5];
    uint8_t i;
    uint8_t ret;

    /* 复位 + 释放总线：不关中断 */
    DHT11_DQ_OUT(0);
    delay_ms(20);
    DHT11_DQ_OUT(1);
    delay_us(30);

    /* 从机应答 + 40 位数据：这一段不能被 SysTick 或其他中断打断 */
    __disable_irq();

    ret = 0;

    if (dht11_wait_level(0, DHT11_TIMEOUT_US) || dht11_wait_level(1, DHT11_TIMEOUT_US))
    {
        ret = DHT11_ERR_NO_ACK;
    }
    else
    {
        for (i = 0; i < 5; i++)
        {
            buf[i] = dht11_read_byte();
        }
    }

    __enable_irq();

    if (ret != 0)
    {
        return ret;
    }

    /* 校验：湿度整数 + 湿度小数 + 温度整数 + 温度小数 == 校验和 */
    if ((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3]) != buf[4])
    {
        return DHT11_ERR_CRC;
    }

    out->humi_x10 = (uint16_t)((uint16_t)buf[0] * 10U + (uint16_t)(buf[1] & 0x0FU));
    out->temp_x10 = (int16_t)((int16_t)buf[2] * 10 + (int16_t)(buf[3] & 0x0FU));
    out->ok        = 1;

    return DHT11_OK;
}

/**
 * @brief       初始化 DHT11 的 DQ 引脚
 * @note        只做引脚配置，不做存在性检测：上电时序紧张，
 *              是否真的接好由第一次 dht11_read() 的返回值体现。
 * @param       无
 * @retval      无
 */
void dht11_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    DHT11_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin   = DHT11_PIN;
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_OD;       /* 开漏输出 */
    gpio_init_struct.Pull  = GPIO_PULLUP;               /* 板载已有 4.7K，这里再使能内部上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_PORT, &gpio_init_struct);

    DHT11_DQ_OUT(1);        /* 释放总线，等待空闲高电平 */
}

/**
 * @brief       读取温湿度（带 3 次重试）
 * @param       out: 输出数据，成功后 out->ok = 1
 * @retval      0 = 成功；非 0 = 失败码（本轮数据不可信，调用方跳过）
 */
uint8_t dht11_read(dht11_data_t *out)
{
    uint8_t ret = DHT11_ERR_RETRY;
    uint8_t i;

    if (out == 0)
    {
        return DHT11_ERR_NULL;
    }

    out->ok = 0;

    for (i = 0; i < DHT11_RETRY_MAX; i++)
    {
        ret = dht11_read_once(out);

        if (ret == DHT11_OK)
        {
            return DHT11_OK;
        }
    }

    /* 连续 3 次都失败：不得死等，直接返回让调用方跳过本轮 */
    out->ok = 0;

    return ret;
}

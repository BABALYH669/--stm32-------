/**
 ****************************************************************************************************
 * @file        at24c02.c
 * @brief       阈值持久化（板载 24C02，软件 IIC：PB6=SCL / PB7=SDA）  —— 阶段一 T1.9
 * @note        本文件自带一套最小软件 IIC（静态函数），不新增 BSP/IIC 目录，
 *              保持交付物清单与《阶段一计划书》§3 的文件树一致。
 ****************************************************************************************************
 */

#include "./BSP/AT24C02/at24c02.h"
#include "./SYSTEM/delay/delay.h"
#include <string.h>

/* ================================================================================================
 * 1. 软件 IIC 底层（PB6 / PB7，板上已有 4.7K 上拉）
 * ================================================================================================ */
#define IIC_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

/* 用 if/else 而不是三元：(x) ? ... : ... 这种写法在 x 是表达式时（例如 IIC_SDA(ack ? 0 : 1)）
 * 会因为三元运算符右结合被解析成 ack ? 0 : (1 ? A : B)，而 int 与 void 不能构成条件表达式 ——
 * ARMCC 直接报 error #32。改成 if/else 就彻底没有这个坑。 */
#define IIC_SCL(x)                  do{ if (x) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); } \
                                        else    { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); } \
                                    }while(0)
#define IIC_SDA(x)                  do{ if (x) { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); } \
                                        else    { HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); } \
                                    }while(0)
#define IIC_READ_SDA                HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)

#define AT24C02_DEV_W               0xA0U   /* 器件地址 + 写 */
#define AT24C02_DEV_R               0xA1U   /* 器件地址 + 读 */
#define AT24C02_PAGE_SIZE           8U      /* 24C02 页大小 8 字节 */
#define AT24C02_WRITE_CYCLE_MS      6U      /* 写周期最大 5ms，取 6ms 留余量 */

static uint8_t s_iic_delay_us = 2;          /* 2us → 250kHz 以内，稳定可靠 */
static uint8_t s_present = 0;

static void iic_delay(void)
{
    delay_us(s_iic_delay_us);
}

static void iic_start(void)
{
    IIC_SDA(1);
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(0);         /* SCL 为高时 SDA 由高变低 = 起始 */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}

static void iic_stop(void)
{
    IIC_SDA(0);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);         /* SCL 为高时 SDA 由低变高 = 停止 */
    iic_delay();
}

/** @retval 0 = 收到 ACK，1 = 失败 */
static uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;

    IIC_SDA(1);         /* 主机释放 SDA */
    iic_delay();
    IIC_SCL(1);
    iic_delay();

    while (IIC_READ_SDA)
    {
        waittime++;

        if (waittime > 250)
        {
            iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);
    iic_delay();

    return rack;
}

static void iic_send_byte(uint8_t data)
{
    uint8_t t;

    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80U) >> 7);       /* 高位先发 */
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        data <<= 1;
    }

    IIC_SDA(1);         /* 释放 SDA，等从机 ACK */
}

/** @param ack: 1 = 读完发 ACK，0 = 读完发 NACK（最后一字节） */
static uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    IIC_SDA(1);         /* 主机释放 SDA，交给从机驱动 */

    for (i = 0; i < 8; i++)
    {
        receive <<= 1;
        iic_delay();
        IIC_SCL(1);
        iic_delay();

        if (IIC_READ_SDA)
        {
            receive++;
        }

        IIC_SCL(0);
    }

    /* 应答位 */
    iic_delay();
    IIC_SDA(ack ? 0 : 1);
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SCL(0);
    iic_delay();
    IIC_SDA(1);

    return receive;
}

/* ================================================================================================
 * 2. 24C02 字节/缓冲读写（24C02 为 1 字节地址）
 * ================================================================================================ */

/** @retval 0 = 成功，1 = 器件无应答 */
static uint8_t at24c02_read_buf(uint16_t addr, uint8_t *pbuf, uint16_t len)
{
    uint16_t i;

    iic_start();
    iic_send_byte(AT24C02_DEV_W);

    if (iic_wait_ack())
    {
        return 1;
    }

    iic_send_byte((uint8_t)(addr & 0xFFU));     /* 24C02 地址只有低 8 位 */

    if (iic_wait_ack())
    {
        return 1;
    }

    iic_start();                                /* 重启总线，转入读模式 */
    iic_send_byte(AT24C02_DEV_R);

    if (iic_wait_ack())
    {
        return 1;
    }

    for (i = 0; i < len; i++)
    {
        pbuf[i] = iic_read_byte((i < (len - 1U)) ? 1U : 0U);    /* 最后一个字节发 NACK */
    }

    iic_stop();

    return 0;
}

/** @retval 0 = 成功，1 = 器件无应答 */
static uint8_t at24c02_write_buf(uint16_t addr, const uint8_t *pbuf, uint16_t len)
{
    uint16_t written = 0;

    while (written < len)
    {
        uint16_t page_left;
        uint16_t i;

        /* 本页还能写几个字节（不跨页） */
        page_left = AT24C02_PAGE_SIZE - (uint16_t)((addr + written) % AT24C02_PAGE_SIZE);
        if (page_left > (len - written))
        {
            page_left = len - written;
        }

        iic_start();
        iic_send_byte(AT24C02_DEV_W);

        if (iic_wait_ack())
        {
            return 1;
        }

        iic_send_byte((uint8_t)((addr + written) & 0xFFU));

        if (iic_wait_ack())
        {
            return 1;
        }

        for (i = 0; i < page_left; i++)
        {
            iic_send_byte(pbuf[written + i]);

            if (iic_wait_ack())
            {
                return 1;
            }
        }

        iic_stop();

        delay_ms(AT24C02_WRITE_CYCLE_MS);       /* 等 EEPROM 内部写周期结束 */

        written += page_left;
    }

    return 0;
}

/* ================================================================================================
 * 3. CRC16 Modbus
 * ================================================================================================ */
static uint16_t crc16_modbus(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= (uint16_t)buf[i];

        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001U)
            {
                crc >>= 1;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* ================================================================================================
 * 4. 对外接口
 * ================================================================================================ */

/* 通用读写包装（供触摸校准等模块调用）—— 内部就是上面那两个按页管理的静态函数 */
uint8_t at24c02_read(uint16_t addr, uint8_t *pbuf, uint16_t len)
{
    return at24c02_read_buf(addr, pbuf, len);
}

uint8_t at24c02_write(uint16_t addr, const uint8_t *pbuf, uint16_t len)
{
    return at24c02_write_buf(addr, pbuf, len);
}

/**
 * @brief       初始化软件 IIC 并探测 24C02
 * @param       无
 * @retval      无
 */
void at24c02_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    IIC_GPIO_CLK_ENABLE();

    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;       /* SCL 推挽 */
    gpio_init_struct.Pull  = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Pin   = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_OD;       /* SDA 开漏，输出 1 时可当输入读 */
    gpio_init_struct.Pin   = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);

    iic_stop();         /* 释放总线 */

    /* 只做「起始 + 器件地址」探测，不写任何字节，免得破坏已有数据 */
    iic_start();
    iic_send_byte(AT24C02_DEV_W);
    s_present = (iic_wait_ack() == 0) ? 1 : 0;
    iic_stop();
}

/**
 * @brief       24C02 是否在线
 * @param       无
 * @retval      1 = 在线，0 = 无应答
 */
uint8_t at24c02_present(void)
{
    return s_present;
}

/**
 * @brief       用 app_config.h 中的默认阈值填充结构体
 * @param       t: 输出结构体
 * @retval      无
 */
void thresh_set_default(thresh_t *t)
{
    if (t == 0)
    {
        return;
    }

    memset(t, 0, sizeof(thresh_t));

    t->magic          = THRESH_MAGIC;
    t->ver            = THRESH_VER;
    t->th_smoke_hi    = TH_SMOKE_HI;
    t->th_temp_hi_x10 = (int16_t)TH_TEMP_HI_X10;
    t->th_humi_lo_x10 = TH_HUMI_LO_X10;
    t->hyst[0]        = HYST_SMOKE;
    t->hyst[1]        = HYST_TEMP_X10;
    t->hyst[2]        = HYST_HUMI_X10;
    t->crc16          = crc16_modbus((const uint8_t *)t, (uint16_t)(sizeof(thresh_t) - sizeof(uint16_t)));
}

/**
 * @brief       从 24C02 加载阈值
 * @param       t: 输出结构体
 * @retval      0 = 成功且 magic/ver/CRC 全部正确；非 0 = 失败（调用方应改用默认值）
 * @note        magic 不符或 CRC 错误时【不回写】，只返回失败。
 */
uint8_t thresh_load(thresh_t *t)
{
    thresh_t tmp;
    uint16_t crc_calc;

    if (t == 0)
    {
        return 1;
    }

    if (at24c02_read_buf(THRESH_ADDR, (uint8_t *)&tmp, (uint16_t)sizeof(thresh_t)))
    {
        return 2;                                   /* 器件无应答 */
    }

    if ((tmp.magic != THRESH_MAGIC) || (tmp.ver != THRESH_VER))
    {
        return 3;                                   /* 未写过 / 版本不符 */
    }

    crc_calc = crc16_modbus((const uint8_t *)&tmp, (uint16_t)(sizeof(thresh_t) - sizeof(uint16_t)));

    if (crc_calc != tmp.crc16)
    {
        return 4;                                   /* CRC 错误 */
    }

    *t = tmp;

    return 0;
}

/**
 * @brief       把阈值写入 24C02
 * @param       t: 待保存的阈值
 * @retval      0 = 成功；非 0 = 失败
 * @note        写入前把整块缓冲清零，保证结构体对齐填充字节是确定的 0，
 *              否则填充字节里的随机值会让 CRC 每次都不一样。
 */
uint8_t thresh_save(const thresh_t *t)
{
    thresh_t tmp;

    if (t == 0)
    {
        return 1;
    }

    if (!s_present)
    {
        return 2;                                   /* 器件不在线，禁止盲写 */
    }

    memset(&tmp, 0, sizeof(thresh_t));

    tmp.magic          = THRESH_MAGIC;
    tmp.ver            = THRESH_VER;
    tmp.th_smoke_hi    = t->th_smoke_hi;
    tmp.th_temp_hi_x10 = t->th_temp_hi_x10;
    tmp.th_humi_lo_x10 = t->th_humi_lo_x10;
    tmp.hyst[0]        = t->hyst[0];
    tmp.hyst[1]        = t->hyst[1];
    tmp.hyst[2]        = t->hyst[2];
    tmp.crc16          = crc16_modbus((const uint8_t *)&tmp, (uint16_t)(sizeof(thresh_t) - sizeof(uint16_t)));

    if (at24c02_write_buf(THRESH_ADDR, (const uint8_t *)&tmp, (uint16_t)sizeof(thresh_t)))
    {
        return 3;
    }

    return 0;
}

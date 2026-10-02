/**
 ****************************************************************************************************
 * @file        tp.c
 * @brief       电阻触摸屏驱动（XPT2046，软件 SPI）  —— 阶段二 T2.2
 *
 * @note        1. 读时序照正点原子「实验26」：发命令 → 等 6us（转换）→ 给 1 个时钟清 BUSY
 *                  → 读 16 位（只有高 12 位有效）→ 右移 4 位。
 *              2. 每个坐标读 5 次，排序后去掉最低/最高各 1 个再取平均（抗尖峰）。
 *              3. 坐标换算（官方五点校准法的模型）：
 *                     屏幕坐标 = (物理坐标 - 物理中心) / fac + 屏幕中心
 *                 fac 带符号，所以"左右/上下反向"会自动被校准吸收，不用另设变量。
 *              4. 判"按下"用 T_PEN 引脚（硬件状态，比 ADC 读数可靠）；
 *                 只有 PEN 抬起才算松开，中间某帧 ADC 抖动不会让 UI 误判成"滑动结束"。
 ****************************************************************************************************
 */

#include "./BSP/TP/tp.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/AT24C02/at24c02.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"

#include <stdlib.h>
#include <string.h>

/* ================================ 引脚（计划书 §2.4 冻结区） ================================ */
#define T_CLK_PORT      GPIOB
#define T_CLK_PIN       GPIO_PIN_1
#define T_MISO_PORT     GPIOB
#define T_MISO_PIN      GPIO_PIN_2      /* ⚠️ PB2 也是 BOOT1，只在初始化后才驱动 */
#define T_MOSI_PORT     GPIOF
#define T_MOSI_PIN      GPIO_PIN_9
#define T_PEN_PORT      GPIOF
#define T_PEN_PIN       GPIO_PIN_10
#define T_CS_PORT       GPIOF
#define T_CS_PIN        GPIO_PIN_11

/* 一律用 if/else 包，避免"宏参数是表达式时三元运算符右结合"那个坑 */
#define T_CLK(x)        do{ if (x) { HAL_GPIO_WritePin(T_CLK_PORT,  T_CLK_PIN,  GPIO_PIN_SET); } \
                            else   { HAL_GPIO_WritePin(T_CLK_PORT,  T_CLK_PIN,  GPIO_PIN_RESET); } }while(0)
#define T_MOSI(x)       do{ if (x) { HAL_GPIO_WritePin(T_MOSI_PORT, T_MOSI_PIN, GPIO_PIN_SET); } \
                            else   { HAL_GPIO_WritePin(T_MOSI_PORT, T_MOSI_PIN, GPIO_PIN_RESET); } }while(0)
#define T_CS(x)         do{ if (x) { HAL_GPIO_WritePin(T_CS_PORT,   T_CS_PIN,   GPIO_PIN_SET); } \
                            else   { HAL_GPIO_WritePin(T_CS_PORT,   T_CS_PIN,   GPIO_PIN_RESET); } }while(0)

#define T_MISO          HAL_GPIO_ReadPin(T_MISO_PORT, T_MISO_PIN)
#define T_PEN           HAL_GPIO_ReadPin(T_PEN_PORT,  T_PEN_PIN)

/* XPT2046 读命令（竖屏：X 用 0xD0、Y 用 0x90；横屏要对调）*/
#define TP_CMD_X        0xD0U
#define TP_CMD_Y        0x90U

/* 采样与滤波 */
#define TP_READ_TIMES   5U      /* 每个坐标读 5 次 */
#define TP_LOST_VAL     1U      /* 排序后去掉最低、最高各 1 个 */
#define TP_SAMPLE_DIFF  25U     /* 两次采样差超过这个 AD 值就认为手在动/抖动 → 本帧作废 */

/* 校准 */
#define TP_CAL_POINTS   5U      /* 五点校准：4 个角 + 中心 */

/* ================================ 校准参数（校准后从 24C02 读回） ================================ */
static float   s_xfac;              /* 触摸距离 / 屏幕距离（带符号）*/
static float   s_yfac;
static short   s_xc;                /* 屏幕中心的物理坐标 */
static short   s_yc;
static uint8_t s_cal_ok;            /* 1 = 校准数据有效 */

/* ================================ 电容触摸（GT9xxx，软件 I2C） ================================
 * 说明：这块 ATK-MD0350 是【电容版】，触摸 IC 走 I2C，引脚正好复用电阻屏那两个：
 *          CT_SCL = PB1 （原 T_CLK）
 *          CT_SDA = PF9 （原 T_MOSI）
 *       INT = PF10、RST = PF11 —— 所以之前把 PF10 当"按下"读才会一直乱跳。
 *       电容屏输出的是**屏幕绝对坐标**（lcddev.id=0x7796 时直接就是 0~319 / 0~479），
 *       因此**不需要五点校准**。
 * ================================================================================================ */
#define GT_SCL_PORT     GPIOB
#define GT_SCL_PIN      GPIO_PIN_1
#define GT_SDA_PORT     GPIOF
#define GT_SDA_PIN      GPIO_PIN_9

#define GT_CMD_WR       0x28U       /* 器件写地址 */
#define GT_CMD_RD       0x29U       /* 器件读地址 */

#define GT_REG_CTRL     0x8040U     /* 控制寄存器：写 2=软复位，写 0=结束复位进正常模式 */
#define GT_REG_PID      0x8140U     /* 产品 ID（4 字节，ASCII，如 "9147"）*/
#define GT_REG_STATUS   0x814EU     /* 状态：bit7=有新数据，bit0~3=触点数 */
#define GT_REG_TP1      0x8150U     /* 第一个触摸点的坐标（x_l,x_h,y_l,y_h）*/

#define GT_IIC_DELAY_US 2U

static uint8_t s_is_ctp;            /* 1 = 探测到电容触摸 IC（GT9xxx）*/

/** @brief 电容 I2C 的 GPIO 初始化（SCL 推挽、SDA 开漏可读）*/
static void gt_iic_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;       /* SCL 推挽 */
    gpio_init_struct.Pull  = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Pin   = GT_SCL_PIN;
    HAL_GPIO_Init(GT_SCL_PORT, &gpio_init_struct);

    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_OD;       /* SDA 开漏（输出 1 时可读回）*/
    gpio_init_struct.Pin   = GT_SDA_PIN;
    HAL_GPIO_Init(GT_SDA_PORT, &gpio_init_struct);

    HAL_GPIO_WritePin(GT_SCL_PORT, GT_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GT_SDA_PORT, GT_SDA_PIN, GPIO_PIN_SET);
}

#define GT_SCL(x)       do{ if (x) { HAL_GPIO_WritePin(GT_SCL_PORT, GT_SCL_PIN, GPIO_PIN_SET); } \
                            else   { HAL_GPIO_WritePin(GT_SCL_PORT, GT_SCL_PIN, GPIO_PIN_RESET); } }while(0)
#define GT_SDA(x)       do{ if (x) { HAL_GPIO_WritePin(GT_SDA_PORT, GT_SDA_PIN, GPIO_PIN_SET); } \
                            else   { HAL_GPIO_WritePin(GT_SDA_PORT, GT_SDA_PIN, GPIO_PIN_RESET); } }while(0)
#define GT_SDA_IN       HAL_GPIO_ReadPin(GT_SDA_PORT, GT_SDA_PIN)

static void gt_iic_delay(void)
{
    delay_us(GT_IIC_DELAY_US);
}

static void gt_iic_start(void)
{
    GT_SDA(1); GT_SCL(1); gt_iic_delay();
    GT_SDA(0); gt_iic_delay();
    GT_SCL(0); gt_iic_delay();
}

static void gt_iic_stop(void)
{
    GT_SDA(0); gt_iic_delay();
    GT_SCL(1); gt_iic_delay();
    GT_SDA(1); gt_iic_delay();
}

/** @retval 0 = 收到 ACK；1 = 无应答（器件没接/地址不对）*/
static uint8_t gt_iic_wait_ack(void)
{
    uint8_t t = 0;

    GT_SDA(1);              /* 主机释放 SDA */
    gt_iic_delay();
    GT_SCL(1);
    gt_iic_delay();

    while (GT_SDA_IN)
    {
        t++;

        if (t > 250U)
        {
            gt_iic_stop();
            return 1;
        }
    }

    GT_SCL(0);
    gt_iic_delay();

    return 0;
}

static void gt_iic_send_byte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8U; i++)
    {
        GT_SDA((data & 0x80U) ? 1 : 0);
        gt_iic_delay();
        GT_SCL(1);
        gt_iic_delay();
        GT_SCL(0);
        data <<= 1;
    }

    GT_SDA(1);              /* 释放，等 ACK */
}

static uint8_t gt_iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;

    GT_SDA(1);

    for (i = 0; i < 8U; i++)
    {
        receive <<= 1;
        gt_iic_delay();
        GT_SCL(1);
        gt_iic_delay();

        if (GT_SDA_IN)
        {
            receive++;
        }

        GT_SCL(0);
    }

    /* 应答位：ack=1 回 ACK，ack=0 回 NACK */
    gt_iic_delay();
    GT_SDA(ack ? 0 : 1);
    gt_iic_delay();
    GT_SCL(1);
    gt_iic_delay();
    GT_SCL(0);
    gt_iic_delay();
    GT_SDA(1);

    return receive;
}

/** @brief 写 GT9xxx 寄存器（先发 2 字节地址，再发数据）*/
static void gt_wr_reg(uint16_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    gt_iic_start();
    gt_iic_send_byte(GT_CMD_WR);                (void)gt_iic_wait_ack();
    gt_iic_send_byte((uint8_t)(reg >> 8));      (void)gt_iic_wait_ack();
    gt_iic_send_byte((uint8_t)(reg & 0xFFU));   (void)gt_iic_wait_ack();

    for (i = 0; i < len; i++)
    {
        gt_iic_send_byte(buf[i]);
        (void)gt_iic_wait_ack();
    }

    gt_iic_stop();
}

/** @brief 读 GT9xxx 寄存器（先发地址，再 restart 后读）*/
static void gt_rd_reg(uint16_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    gt_iic_start();
    gt_iic_send_byte(GT_CMD_WR);                (void)gt_iic_wait_ack();
    gt_iic_send_byte((uint8_t)(reg >> 8));      (void)gt_iic_wait_ack();
    gt_iic_send_byte((uint8_t)(reg & 0xFFU));   (void)gt_iic_wait_ack();

    gt_iic_start();
    gt_iic_send_byte(GT_CMD_RD);                (void)gt_iic_wait_ack();

    for (i = 0; i < len; i++)
    {
        buf[i] = gt_iic_read_byte((i == (len - 1U)) ? 0U : 1U);   /* 最后一字节回 NACK */
    }

    gt_iic_stop();
}

/**
 * @brief       探测电容触摸 IC
 * @retval      1 = 有 GT9xxx（电容屏）；0 = 没有
 * @note        读 0x8140 的产品 ID：全 0 或全 0xFF 视为没有器件。
 */
static uint8_t gt_probe(void)
{
    uint8_t id[5] = {0, 0, 0, 0, 0};
    uint8_t cmd;
    uint8_t st = 0;

    gt_rd_reg(GT_REG_PID, id, 4);
    id[4] = 0;                  /* 补结束符，方便按字符串比较 */

    if (((id[0] == 0x00U) && (id[1] == 0x00U) && (id[2] == 0x00U) && (id[3] == 0x00U)) ||
        ((id[0] == 0xFFU) && (id[1] == 0xFFU) && (id[2] == 0xFFU) && (id[3] == 0xFFU)))
    {
        return 0;
    }

    printf("[TP] CTP id = %s\r\n", (char *)id);

    /* 只认官方支持的 GT 系列型号，别的当作没接通 */
    if ((strcmp((char *)id, "911") != 0) && (strcmp((char *)id, "9147") != 0) &&
        (strcmp((char *)id, "1158") != 0) && (strcmp((char *)id, "9271") != 0))
    {
        printf("[TP] unsupported CTP id, ignored\r\n");
        return 0;
    }

    /* ★ 关键：软复位 → 结束复位，让 IC 进入"上报坐标"的正常模式。
       少了这两步，状态寄存器(0x814E)的"有新数据"位永远不置起，读坐标会一直失败。
       （这一步对应官方 gt9xxx_init() 结尾的 CTRL 寄存器写 0x02、再写 0x00。）*/
    cmd = 0x02;
    gt_wr_reg(GT_REG_CTRL, &cmd, 1);        /* 软复位 */
    delay_ms(10);

    cmd = 0x00;
    gt_wr_reg(GT_REG_CTRL, &cmd, 1);        /* 结束复位，进入正常模式 */
    delay_ms(10);

    /* 自检：把状态寄存器的初始值打出来，便于下次对比 */
    gt_rd_reg(GT_REG_STATUS, &st, 1);
    printf("[TP] status reg = 0x%02X\r\n", st);

    return 1;
}

/**
 * @brief       读一个电容触摸点
 * @param       x, y: 输出屏幕坐标
 * @retval      1 = 有触摸；0 = 没有
 */
static uint8_t gt_read_point(uint16_t *x, uint16_t *y)
{
    uint8_t st = 0;
    uint8_t buf[4];
    uint8_t zero = 0;
    static uint8_t s_oor_cnt = 0;       /* 坐标越界的打印计数（诊断，最多 5 次）*/

    gt_rd_reg(GT_REG_STATUS, &st, 1);

    if ((st & 0x80U) == 0U)             /* bit7 = 0：还没有新数据 */
    {
        return 0;
    }

    if ((st & 0x0FU) == 0U)             /* 触点数 = 0：手指抬起了 */
    {
        gt_wr_reg(GT_REG_STATUS, &zero, 1);     /* 清标志，否则以后不再更新 */
        return 0;
    }

    gt_rd_reg(GT_REG_TP1, buf, 4);
    gt_wr_reg(GT_REG_STATUS, &zero, 1);         /* 必须清标志 */

    *x = (uint16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    *y = (uint16_t)(((uint16_t)buf[3] << 8) | buf[2]);

    /* 越界就认为不可信（也给 IC 还没初始化好的情况兜底）。
       诊断：若触摸 IC 的输出范围不是屏幕分辨率（例如 0~4095），会在这里打印原始值，
       看到这行就说明需要按比例缩放坐标。 */
    if ((*x >= lcddev.width) || (*y >= lcddev.height))
    {
        if (s_oor_cnt < 5U)
        {
            s_oor_cnt++;
            printf("[TP] point out of range: raw=(%u,%u) screen=(%u,%u)\r\n",
                   (unsigned)*x, (unsigned)*y, (unsigned)lcddev.width, (unsigned)lcddev.height);
        }

        return 0;
    }

    return 1;
}

/* ================================ 底层：XPT2046 读 ================================ */

/** @brief 软件 SPI 发一个字节（上升沿有效）*/
static void tp_write_byte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80U)
        {
            T_MOSI(1);
        }
        else
        {
            T_MOSI(0);
        }

        data <<= 1;
        T_CLK(0);
        delay_us(1);
        T_CLK(1);
    }
}

/**
 * @brief       读一次 AD 值
 * @param       cmd: TP_CMD_X / TP_CMD_Y
 * @retval      12 位 AD 值
 */
static uint16_t tp_read_ad(uint8_t cmd)
{
    uint8_t  i;
    uint16_t num = 0;

    T_CLK(0);
    T_MOSI(0);
    T_CS(0);                    /* 选中触摸 IC */

    tp_write_byte(cmd);         /* 发命令字 */
    delay_us(6);                /* ADS7846 转换时间最长 6us */

    T_CLK(0);
    delay_us(1);
    T_CLK(1);                   /* 给 1 个时钟，清 BUSY */
    delay_us(1);
    T_CLK(0);

    for (i = 0; i < 16U; i++)   /* 读 16 位，只有高 12 位有效 */
    {
        num <<= 1;
        T_CLK(0);
        delay_us(1);
        T_CLK(1);

        if (T_MISO)
        {
            num++;
        }
    }

    num >>= 4;                  /* 只保留高 12 位 */
    T_CS(1);                    /* 释放片选 */

    return num;
}

/**
 * @brief       读一个方向的坐标（读 5 次，去极值后平均）
 * @param       cmd: TP_CMD_X / TP_CMD_Y
 * @retval      滤波后的 12 位 AD 值
 */
static uint16_t tp_read_xoy(uint8_t cmd)
{
    uint16_t buf[TP_READ_TIMES];
    uint16_t i, j, tmp, sum = 0;

    for (i = 0; i < TP_READ_TIMES; i++)
    {
        buf[i] = tp_read_ad(cmd);
    }

    /* 插入排序（只有 5 个元素）*/
    for (i = 1; i < TP_READ_TIMES; i++)
    {
        tmp = buf[i];
        j   = i;

        while ((j > 0U) && (buf[j - 1U] > tmp))
        {
            buf[j] = buf[j - 1U];
            j--;
        }

        buf[j] = tmp;
    }

    for (i = TP_LOST_VAL; i < (TP_READ_TIMES - TP_LOST_VAL); i++)
    {
        sum += buf[i];
    }

    return (uint16_t)(sum / (TP_READ_TIMES - 2U * TP_LOST_VAL));
}

/** @brief 读一次原始物理坐标（竖屏 X=0xD0 / Y=0x90；横屏对调）*/
static void tp_read_raw(uint16_t *x, uint16_t *y)
{
    if (lcddev.dir & 0x01U)     /* 横屏 */
    {
        *x = tp_read_xoy(TP_CMD_Y);
        *y = tp_read_xoy(TP_CMD_X);
    }
    else                        /* 竖屏（本项目用这个）*/
    {
        *x = tp_read_xoy(TP_CMD_X);
        *y = tp_read_xoy(TP_CMD_Y);
    }
}

/**
 * @brief       读一个有效的屏幕坐标点
 * @param       x, y: 输出屏幕坐标
 * @retval      1 = 本次读数可信；0 = 抖动/未校准，调用方沿用上次坐标
 */
static uint8_t tp_read_point(uint16_t *x, uint16_t *y)
{
    uint16_t px1, py1, px2, py2;
    int32_t  sx, sy;

    if (!s_cal_ok)
    {
        return 0;
    }

    tp_read_raw(&px1, &py1);
    tp_read_raw(&px2, &py2);

    /* 连续两次采样差太多 → 认为手在移动或抖动，本帧作废 */
    if ((abs((int)px1 - (int)px2) > (int)TP_SAMPLE_DIFF) ||
        (abs((int)py1 - (int)py2) > (int)TP_SAMPLE_DIFF))
    {
        return 0;
    }

    px1 = (uint16_t)((px1 + px2) / 2U);
    py1 = (uint16_t)((py1 + py2) / 2U);

    /* 物理坐标 → 屏幕坐标（fac 带符号，反向会自动被吸收）*/
    sx = (int32_t)((float)(short)(px1 - s_xc) / s_xfac) + (int32_t)(lcddev.width / 2U);
    sy = (int32_t)((float)(short)(py1 - s_yc) / s_yfac) + (int32_t)(lcddev.height / 2U);

    /* 越界钳位（手滑到屏外时别给出离谱坐标）*/
    if (sx < 0)
    {
        sx = 0;
    }
    if (sx >= (int32_t)lcddev.width)
    {
        sx = (int32_t)lcddev.width - 1;
    }
    if (sy < 0)
    {
        sy = 0;
    }
    if (sy >= (int32_t)lcddev.height)
    {
        sy = (int32_t)lcddev.height - 1;
    }

    *x = (uint16_t)sx;
    *y = (uint16_t)sy;

    return 1;
}

/* ================================ 校准数据存取（24C02 0x40 起） ================================ */

/** @brief 把 4 个校准参数 + 1 字节标记写进 24C02 */
static void tp_save_cal(void)
{
    uint8_t buf[13];

    memcpy(buf,      &s_xfac, 4);
    memcpy(buf + 4,  &s_yfac, 4);
    memcpy(buf + 8,  &s_xc,   2);
    memcpy(buf + 10, &s_yc,   2);
    buf[12] = TP_CAL_MAGIC;

    (void)at24c02_write(TP_CAL_ADDR, buf, sizeof(buf));
}

/** @retval 1 = 读到了有效校准数据 */
static uint8_t tp_load_cal(void)
{
    uint8_t buf[13];

    if (at24c02_read(TP_CAL_ADDR, buf, sizeof(buf)) != 0)
    {
        return 0;
    }

    if (buf[12] != TP_CAL_MAGIC)
    {
        return 0;
    }

    memcpy(&s_xfac, buf,      4);
    memcpy(&s_yfac, buf + 4,  4);
    memcpy(&s_xc,   buf + 8,  2);
    memcpy(&s_yc,   buf + 10, 2);

    /* 合法性检查：fac 不能是 0，也不能是离谱值（一但数据坏了就要求重新校准）*/
    if ((s_xfac == 0.0f) || (s_yfac == 0.0f))
    {
        return 0;
    }

    if ((s_xfac > 100.0f) || (s_xfac < -100.0f) || (s_yfac > 100.0f) || (s_yfac < -100.0f))
    {
        return 0;
    }

    return 1;
}

/* ================================ 对外接口 ================================ */

/**
 * @brief       初始化触摸屏
 * @note        必须在 lcd_init() 与 at24c02_init() 之后调用。
 * @param       无
 * @retval      无
 */
void tp_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    /* ① 先按【电容屏】探测：这块 ATK-MD0350 是电容版，触摸 IC 走软件 I2C，
          引脚正好复用 PB1(SCL) / PF9(SDA)（INT=PF10、RST=PF11）。
          电容屏输出的是绝对坐标，不需要校准，探测到就直接收工。 */
    gt_iic_gpio_init();
    delay_ms(10);
    s_is_ctp = gt_probe();

    if (s_is_ctp)
    {
        printf("[TP] capacitive touch detected, calibration not needed\r\n");
        s_cal_ok = 1;
        return;
    }

    printf("[TP] no capacitive IC, fall back to resistive (XPT2046)\r\n");

    /* ② 电阻屏：把 PB1 / PF9 重新配回 SPI 用的推挽输出 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* 输出：CLK / MOSI / CS */
    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull  = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio_init_struct.Pin = T_CLK_PIN;
    HAL_GPIO_Init(T_CLK_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = T_MOSI_PIN;
    HAL_GPIO_Init(T_MOSI_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = T_CS_PIN;
    HAL_GPIO_Init(T_CS_PORT, &gpio_init_struct);

    /* 输入：MISO / PEN（都上拉）*/
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;

    gpio_init_struct.Pin = T_MISO_PIN;
    HAL_GPIO_Init(T_MISO_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = T_PEN_PIN;
    HAL_GPIO_Init(T_PEN_PORT, &gpio_init_struct);

    T_CS(1);            /* 释放片选 */
    T_CLK(0);

    s_cal_ok = tp_load_cal();
}

/**
 * @brief       查询是否已校准
 * @param       无
 * @retval      1 = 24C02 里有有效校准数据
 */
uint8_t tp_is_calibrated(void)
{
    return s_cal_ok;
}

/**
 * @brief       查询是不是电容屏
 * @param       无
 * @retval      1 = 电容屏（GT9xxx 走 I2C）；0 = 电阻屏（XPT2046 走 SPI）
 */
uint8_t tp_is_capacitive(void)
{
    return s_is_ctp;
}
/**
 * @brief       查询 T_PEN 引脚电平（诊断用，仅电阻屏有意义）
 * @param       无
 * @retval      1 = 未按下（高）；0 = 按下（低）
 * @note        电容屏上 PF10 是触摸 IC 的 INT 引脚，会周期性跳动，读它没有意义。
 */
uint8_t tp_get_pen(void)
{
    return (T_PEN != 0U) ? 1U : 0U;
}

/**
 * @brief       扫描触摸
 * @param       p: 输出（屏幕坐标 + pressed）
 * @retval      1 = 当前有**有效**触摸；0 = 没有
 * @note        ⚠️ 两道门槛，缺一不可：
 *              1. T_PEN 必须为低（硬件上确实按着）；
 *              2. 坐标读数必须有效（tp_read_point 成功）。
 *              第 2 条很关键：未校准、或者 PEN 引脚被干扰/悬空导致乱跳时，
 *              读数不成立就返回 0，UI 层就不会把噪声当成"点击"而自动翻页。
 */
uint8_t tp_scan(tp_point_t *p)
{
    uint16_t x, y;

    if (p == 0)
    {
        return 0;
    }

    /* ★ 电容屏：直接读 I2C，没有"PEN 引脚判按下"这回事 */
    if (s_is_ctp)
    {
        if (!gt_read_point(&x, &y))
        {
            p->pressed = 0;
            return 0;
        }

        p->x       = x;
        p->y       = y;
        p->pressed = 1;

        return 1;
    }

    if (T_PEN != 0U)            /* PEN 高 = 没按下 */
    {
        p->pressed = 0;
        return 0;
    }

    if (!tp_read_point(&x, &y)) /* 按着但读数不可信（未校准 / 抖动 / 干扰）→ 不认 */
    {
        p->pressed = 0;
        return 0;
    }

    p->x       = x;
    p->y       = y;
    p->pressed = 1;

    return 1;
}

/** @brief 画一个十字（校准用）*/
static void tp_draw_cross(uint16_t x, uint16_t y, uint16_t color)
{
    lcd_draw_line(x - 12U, y, x + 12U, y, color);
    lcd_draw_line(x, y - 12U, x, y + 12U, color);
}

/**
 * @brief       五点校准（依次点 4 个角 + 中心）
 * @note        这是交互式函数，会阻塞（最长约 10 秒/点），只在"未校准"时调用一次。
 *              校准完把参数写进 24C02，断电不丢。
 * @param       无
 * @retval      0 = 成功；1 = 失败/超时（保持原来的校准数据不变）
 */
uint8_t tp_calibrate(void)
{
    uint16_t sx[TP_CAL_POINTS], sy[TP_CAL_POINTS];      /* 5 个十字的屏幕坐标 */
    uint16_t px[TP_CAL_POINTS], py[TP_CAL_POINTS];      /* 5 个点的物理坐标   */
    uint16_t w = lcddev.width;
    uint16_t h = lcddev.height;
    uint8_t  i;
    int32_t  s1, s2, s3, s4;
    float    fac1, fac2;

    /* 电容屏输出的是绝对坐标，不需要做五点校准 */
    if (s_is_ctp)
    {
        printf("[TP] capacitive touch: calibration skipped\r\n");
        return 0;
    }

    /* 五个点：左上 → 右上 → 左下 → 右下 → 中心 */
    sx[0] = 20U;          sy[0] = 20U;
    sx[1] = w - 20U;      sy[1] = 20U;
    sx[2] = 20U;          sy[2] = h - 20U;
    sx[3] = w - 20U;      sy[3] = h - 20U;
    sx[4] = w / 2U;       sy[4] = h / 2U;

    lcd_clear(WHITE);
    lcd_show_string(10, h / 2U - 40U, w, 16, 16, "TOUCH CALIBRATION", BLUE);
    lcd_show_string(10, h / 2U - 16U, w, 16, 16, "TAP EACH CROSS", BLACK);
    lcd_show_string(10, h / 2U + 8U,  w, 16, 16, "IN TURN (5 POINTS)", BLACK);
    delay_ms(2000);
    lcd_clear(WHITE);

    for (i = 0; i < TP_CAL_POINTS; i++)
    {
        uint32_t t0;

        tp_draw_cross(sx[i], sy[i], RED);
        t0 = HAL_GetTick();

        /* --- 等手指按下去 --- */
        while (T_PEN != 0U)
        {
            if ((uint32_t)(HAL_GetTick() - t0) > 10000U)
            {
                lcd_clear(WHITE);
                return 1;                       /* 超时 */
            }
        }

        delay_ms(40);                           /* 让电平稳定一下 */

        tp_read_raw(&px[i], &py[i]);            /* 记录物理坐标 */

        /* --- 等手指抬起来 --- */
        while (T_PEN == 0U)
        {
            if ((uint32_t)(HAL_GetTick() - t0) > 15000U)
            {
                lcd_clear(WHITE);
                return 1;                       /* 超时 */
            }
        }

        delay_ms(60);
        tp_draw_cross(sx[i], sy[i], WHITE);     /* 擦掉这个十字 */
    }

    /* ---------------- 算校准参数 ---------------- */
    s1 = (int32_t)px[1] - (int32_t)px[0];       /* 点2-点1，X 方向物理距离 */
    s3 = (int32_t)px[3] - (int32_t)px[2];       /* 点4-点3，X 方向物理距离 */
    s2 = (int32_t)py[3] - (int32_t)py[1];       /* 点4-点2，Y 方向物理距离 */
    s4 = (int32_t)py[2] - (int32_t)py[0];       /* 点3-点1，Y 方向物理距离 */

    if ((s1 == 0) || (s2 == 0) || (s3 == 0) || (s4 == 0))
    {
        lcd_clear(WHITE);
        return 1;                               /* 数据不可用 */
    }

    fac1 = (float)s1 / (float)s3;               /* 两个 X 距离的比值，越接近 1 说明点得越准 */
    fac2 = (float)s2 / (float)s4;

    if (fac1 < 0.0f)
    {
        fac1 = -fac1;
    }
    if (fac2 < 0.0f)
    {
        fac2 = -fac2;
    }

    if ((fac1 < 0.90f) || (fac1 > 1.10f) || (fac2 < 0.90f) || (fac2 > 1.10f))
    {
        lcd_clear(WHITE);
        lcd_show_string(20, h / 2U, w, 16, 16, "CAL FAIL, TRY AGAIN", RED);
        delay_ms(1500);
        lcd_clear(WHITE);
        return 1;                               /* 点得太偏 */
    }

    s_xfac = (float)(s1 + s3) / (2.0f * (float)((int32_t)w - 40));
    s_yfac = (float)(s2 + s4) / (2.0f * (float)((int32_t)h - 40));
    s_xc   = (short)px[4];                      /* 中心点的物理坐标 */
    s_yc   = (short)py[4];

    if ((s_xfac == 0.0f) || (s_yfac == 0.0f))
    {
        lcd_clear(WHITE);
        return 1;
    }

    s_cal_ok = 1;
    tp_save_cal();

    lcd_clear(WHITE);
    lcd_show_string(20, h / 2U, w, 16, 16, "CALIBRATION OK", GREEN);
    delay_ms(1200);
    lcd_clear(WHITE);

    return 0;
}

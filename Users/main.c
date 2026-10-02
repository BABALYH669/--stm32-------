/**
 ****************************************************************************************************
 * @file        main.c
 * @brief       基于 STM32F103 的环境监测系统 —— 阶段一主程序  —— 阶段一 T1.12
 * @note        架构：裸机前后台。主循环用 HAL_GetTick() 差值做非阻塞时基，
 *              除了 ESP8266 硬复位脉冲（≤20ms）与 DHT11 复位（20ms）之外没有任何阻塞延时。
 *
 *              本阶段的固定时基（顺序与《阶段一计划书》T1.12 一致）：
 *                 10 ms  → alarm_tick()  （LED0 心跳 / LED1 / LED2 / 蜂鸣器 / LED3 调光）+ 按键扫描
 *                 100 ms → 读传感器 + alarm_update()
 *                 1000 ms→ ts++ + 1 Hz 串口输出
 *                 5000 ms→ seq++ + TCP 上报
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

#include "app_config.h"
#include "env_data.h"
#include "alarm.h"

#include "./BSP/LED/led.h"
#include "./BSP/LED_PWM/led_pwm.h"
#include "./BSP/BEEP/beep.h"
#include "./BSP/KEY/key.h"
#include "./BSP/ADC/adc3_dma.h"
#include "./BSP/LSENS/lsens.h"
#include "./BSP/DHT11/dht11.h"
#include "./BSP/AT24C02/at24c02.h"
#include "./BSP/NET/net.h"

#include "lcd_ui.h"
#include "./BSP/TP/tp.h"

#include <stdio.h>

/* 时基判定宏：无符号差值比较，天然免疫 HAL_GetTick() 回绕 */
#define TICK_DUE(now, last, period)     ((uint32_t)((now) - (last)) >= (uint32_t)(period))

/* LED4 上传指示的常亮时长（ms） */
#define LED4_ON_MS                      100U

/* ------------------------------------------------------------------------------------------------
 * 模块内静态状态
 * ---------------------------------------------------------------------------------------------- */
static thresh_t g_th;                       /* 当前生效阈值（24C02 → 默认值） */

/**
 * @brief       读一路传感器并更新全局快照
 * @note        1. 只读，不做判定；判定交给 alarm_update()
 *              2. DHT11 采样率上限是 1 Hz，所以按 100ms × 10 分频，不能每 100ms 都读
 *              3. 烟雾基线：预热窗口（SMOKE_WARMUP_MS）结束时取一次清洁空气基线，
 *                 之后一律用相对值 smoke_rel 参与判定与上报（《项目计划书v2》§6.2）
 * @param       无
 * @retval      无
 */
static void read_sensors(void)
{
    static uint8_t  s_dht_div         = 0;
    static uint8_t  s_smoke_base_ready = 0;
    static uint16_t s_smoke_base      = 0;

    env_data_t *d = env_data_get();

    d->lux       = lsens_get();
    d->smoke_raw = adc3_get(ADC3_IDX_SMOKE);
    d->valid    |= (ENV_VALID_LSENS | ENV_VALID_SMOKE);

    if (HAL_GetTick() >= SMOKE_WARMUP_MS)
    {
        if (!s_smoke_base_ready)
        {
            s_smoke_base       = d->smoke_raw;
            s_smoke_base_ready = 1;
            printf("[SMOKE] warmup done, baseline=%u\r\n", (unsigned)s_smoke_base);
        }

        d->smoke_rel = (d->smoke_raw > s_smoke_base) ? (uint16_t)(d->smoke_raw - s_smoke_base) : 0U;
    }
    else
    {
        d->smoke_rel = 0;                   /* 预热期内 AO 输出不可信，相对值固定为 0 */
    }

    /* ---- DHT11：1 Hz ---- */
    s_dht_div++;

    if (s_dht_div >= 10U)
    {
        dht11_data_t dd;

        s_dht_div = 0;

        if (dht11_read(&dd) == DHT11_OK)
        {
            d->temp_x10 = dd.temp_x10;
            d->humi_x10 = dd.humi_x10;
            d->valid   |= ENV_VALID_DHT11;
        }
        else
        {
            d->valid &= (uint8_t)(~ENV_VALID_DHT11);    /* 本轮数据不可信，不能参与报警判定 */
        }
    }
}

/**
 * @brief       1 Hz 串口输出（格式固定，后续阶段不得改动）
 * @note        格式：[123s] lux=045 smoke=312 temp=25.3C humi=60.1% alarm=none seq=1024 net=ON ack=100
 * @param       无
 * @retval      无
 */
static void report_1hz_to_uart(void)
{
    const env_data_t *d = env_data_get();
    int32_t  t  = (int32_t)d->temp_x10;
    uint32_t ta = (uint32_t)((t < 0) ? -t : t);

    printf("[%lus] lux=%03u smoke=%u temp=%d.%luC humi=%u.%lu%% alarm=%s seq=%lu net=%s ack=%lu\r\n",
           (unsigned long)d->ts,
           (unsigned)d->lux,
           (unsigned)d->smoke_rel,
           (int)(t / 10),
           (unsigned long)(ta % 10UL),
           (unsigned)(d->humi_x10 / 10U),
           (unsigned long)(d->humi_x10 % 10U),
           alarm_state_str((uint8_t)d->alarm),
           (unsigned long)d->seq,
           net_is_online() ? "ON" : "OFF",
           (unsigned long)net_ack_count());
}

/**
 * @brief       按键扫描并打印（T1.5 的验收动作）
 * @note        5 个键分别是 上/下/左/右/确认，物理映射见 app_config.h §5。
 *              长按会按"慢速重复(150ms) / 加速重复(60ms)"两段节拍连续打印 —— 这是刻意的，
 *              阶段二的 UI 就是靠这个在列表里连续滚动。按键功能本身（调阈值 / 消音）
 *              在计划书里属可选，阶段一不做。
 * @param       无
 * @retval      无
 */
static void key_poll_and_log(void)
{
    if (key_scan(KEY_UP))
    {
        printf("[KEY] UP\r\n");
    }

    if (key_scan(KEY_DOWN))
    {
        printf("[KEY] DOWN\r\n");
    }

    /* ★ 左 / 右键 = 切页（触摸没调通时的备用操作，调试时也最方便）
       key.c 里有长按重复，所以按住不放可以连续翻页。 */
    if (key_scan(KEY_LEFT))
    {
        printf("[KEY] LEFT  -> prev page\r\n");
        lcd_ui_page_prev();
    }

    if (key_scan(KEY_RIGHT))
    {
        printf("[KEY] RIGHT -> next page\r\n");
        lcd_ui_page_next();
    }

    if (key_scan(KEY_OK))
    {
        printf("[KEY] OK\r\n");
    }
}

int main(void)
{
    uint32_t tick_10   = 0;
    uint32_t tick_100  = 0;
    uint32_t tick_1000 = 0;
    uint32_t tick_5000 = 0;

    uint32_t last_ack     = 0;
    uint32_t led4_off_at  = 0;

    /* ---------------- 1. 系统与驱动初始化 ---------------- */
    HAL_Init();
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* 72 MHz */
    delay_init(72);
    usart_init(115200);                         /* USART1 = 调试串口（禁用 DMA） */

    lcd_ui_init();                              /* 3.5 寸 TFTLCD（FSMC）；内部会打印 LCD 控制器 ID */

    led_init();
    led_pwm_init();
    beep_init();
    key_init();

    adc3_dma_init();
    adc3_dma_start();
    lsens_init();
    dht11_init();

    /* ---------------- 2. 阈值：24C02 → 默认值 ---------------- */
    at24c02_init();

    /* ---------------- 2.5 触摸屏：初始化 + 首次使用先做五点校准 ----------------
     * 校准系数存在 24C02 的 0x40 起（见 tp.h），所以必须放在 at24c02_init() 之后；
     * 屏幕上要画十字，所以也要放在 lcd_ui_init() 之后。
     * 首次上电（或换了屏幕）会阻塞在这里等你点 5 个十字，点完就写进 24C02，以后不再问。 */
    tp_init();

    if (tp_is_capacitive())
    {
        printf("[TP] capacitive touch (GT9xxx via I2C), calibration not needed\r\n");
    }
    else if (!tp_is_calibrated())
    {
        printf("[TP] resistive touch (XPT2046), NOT calibrated: tap the 5 crosses...\r\n");
        (void)tp_calibrate();
    }
    else
    {
        printf("[TP] resistive touch (XPT2046), calibration loaded from 24C02\r\n");
    }

    /* 校准过程（不管成功还是失败）都会清屏，所以这里把页面重画一次：
       保证不管触摸有没有调通，屏幕上都能正常显示数据。 */
    lcd_ui_init();

    if (thresh_load(&g_th) == 0)
    {
        printf("[TH] loaded from 24C02\r\n");
    }
    else
    {
        thresh_set_default(&g_th);
        printf("[TH] no valid record, use default\r\n");
    }

    printf("[TH] smoke_hi=%u temp_hi=%d.%u humi_lo=%u.%u hyst=%u/%u/%u\r\n",
           (unsigned)g_th.th_smoke_hi,
           (int)(g_th.th_temp_hi_x10 / 10), (unsigned)(g_th.th_temp_hi_x10 % 10),
           (unsigned)(g_th.th_humi_lo_x10 / 10U), (unsigned)(g_th.th_humi_lo_x10 % 10U),
           (unsigned)g_th.hyst[0], (unsigned)g_th.hyst[1], (unsigned)g_th.hyst[2]);

    /* ---------------- 3. 报警状态机 + 网络 ---------------- */
    alarm_init();
    alarm_reload_thresh(&g_th);

    net_init();

    printf("boot: warmup %lu s\r\n", (unsigned long)(SMOKE_WARMUP_MS / 1000UL));

    tick_10 = tick_100 = tick_1000 = tick_5000 = HAL_GetTick();

    /* ---------------- 4. 主循环 ---------------- */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        net_poll();                                     /* AT 时序非阻塞推进 + 收帧解析 */

        if (TICK_DUE(now, tick_10, 10))
        {
            tick_10 = now;
            alarm_tick();                               /* 10 ms：灯语 / 蜂鸣器 / LED3 调光 */
            key_poll_and_log();
            lcd_ui_touch_poll();                        /* 10 ms：读触摸 + 滑动/点击切页 */
        }

        if (TICK_DUE(now, tick_100, 100))
        {
            tick_100 = now;
            read_sensors();
            alarm_update(env_data_get());
            env_data_get()->alarm = (alarm_state_t)alarm_get_state();
        }

        if (TICK_DUE(now, tick_1000, 1000))
        {
            tick_1000 = now;
            env_data_mark_ts();
            report_1hz_to_uart();
            lcd_ui_refresh();                           /* 把最新数据刷到 LCD 上 */
        }

        if (TICK_DUE(now, tick_5000, 5000))
        {
            tick_5000 = now;
            env_data_get()->seq++;
            (void)net_send_data(env_data_get());
        }

        /* LED4 上传指示：只在收到新的 ACK 时点亮 100ms；net=OFF 时永远不会闪 */
        if (net_ack_count() != last_ack)
        {
            last_ack = net_ack_count();
            led_on(LED_ID_4);
            led4_off_at = now + LED4_ON_MS;
        }

        if ((led4_off_at != 0U) && ((int32_t)(now - led4_off_at) >= 0))
        {
            led_off(LED_ID_4);
            led4_off_at = 0;
        }
    }
}

/**
 ****************************************************************************************************
 * @file        alarm.c
 * @brief       本地阈值报警状态机  —— 阶段一 T1.10
 * @note        判定规则（《项目计划书v2》§6.2）：
 *                触发： value > TH_HI   连续 CONFIRM_CNT 次成立
 *                解除： value < TH_HI-HYST 连续 CONFIRM_CNT 次成立
 *                湿度反向：value < TH_LO 触发，value > TH_LO+HYST 解除
 *              落在迟滞带内（TH_HI-HYST ~ TH_HI）时两个计数器都清零，
 *              因此阈值附近来回摆动不会让蜂鸣器"哒哒"抖动。
 ****************************************************************************************************
 */

#include "alarm.h"
#include "./BSP/LED/led.h"
#include "./BSP/LED_PWM/led_pwm.h"
#include "./BSP/BEEP/beep.h"

/* LED0 心跳 / LED2 闪烁 / LED3 调光 的周期（ms） */
#define ALARM_LED0_HALF_MS      500U    /* 500ms 翻转 = 1Hz 闪烁 */
#define ALARM_LED2_HALF_MS      250U    /* 250ms 翻转 = 2Hz 闪烁 */
#define ALARM_LED3_PERIOD_MS    200U    /* LED3 每 200ms 更新一次占空比 */

/* 蜂鸣器节拍（ms）：以 tick 的绝对时间为基准，不累积漂移 */
#define BEEP_TEMP_ON_MS         200U    /* TEMP/HUMI：响 200ms            */
#define BEEP_TEMP_PERIOD_MS     1000U   /* TEMP/HUMI：周期 1s → 停 800ms  */
#define BEEP_MULTI_ON_MS        100U    /* MULTI：响 100ms                */
#define BEEP_MULTI_PERIOD_MS    200U    /* MULTI：周期 200ms → 停 100ms   */

static thresh_t s_th;                   /* 当前生效阈值 */

static uint8_t s_smoke_latch;           /* 三个独立的"越限"锁存位 */
static uint8_t s_temp_latch;
static uint8_t s_humi_latch;

static uint8_t s_cnt_smoke_up, s_cnt_smoke_dn;
static uint8_t s_cnt_temp_up,  s_cnt_temp_dn;
static uint8_t s_cnt_humi_up,  s_cnt_humi_dn;

static uint8_t s_alarm;                 /* 当前报警状态（alarm_state_t） */

static uint32_t s_led0_tick;
static uint32_t s_led2_tick;
static uint32_t s_led3_tick;

/**
 * @brief       把报警状态映射成字符串
 * @param       st: alarm_state_t
 * @retval      常量字符串
 */
const char *alarm_state_str(uint8_t st)
{
    switch (st)
    {
        case ALARM_SMOKE: return "smoke";
        case ALARM_TEMP:  return "temp";
        case ALARM_HUMI:  return "humi";
        case ALARM_MULTI: return "multi";
        default:          return "none";
    }
}

/**
 * @brief       重载阈值
 * @note        必须做合法性检查：温度上限/湿度下限为 0 或离谱值会让状态机永远报警。
 * @param       t: 新阈值，NULL 表示忽略
 * @retval      无
 */
void alarm_reload_thresh(const thresh_t *t)
{
    if (t == 0)
    {
        return;
    }

    s_th = *t;
}

/**
 * @brief       初始化报警状态机
 * @param       无
 * @retval      无
 */
void alarm_init(void)
{
    thresh_set_default(&s_th);          /* 主循环若已 thresh_load()，随后调用 alarm_reload_thresh() 覆盖 */

    s_smoke_latch = 0;
    s_temp_latch  = 0;
    s_humi_latch  = 0;

    s_cnt_smoke_up = s_cnt_smoke_dn = 0;
    s_cnt_temp_up  = s_cnt_temp_dn  = 0;
    s_cnt_humi_up  = s_cnt_humi_dn  = 0;

    s_alarm = ALARM_NONE;

    /* 上电先落到 NORMAL 的灯语 */
    led_off(LED_ID_1);
    led_off(LED_ID_2);
    beep_set(0);

    s_led0_tick = HAL_GetTick();
    s_led2_tick = HAL_GetTick();
    s_led3_tick = HAL_GetTick();
}

/**
 * @brief       根据三个锁存位合成最终报警状态
 * @param       无
 * @retval      alarm_state_t
 */
static uint8_t alarm_combine(void)
{
    if (s_smoke_latch && (s_temp_latch || s_humi_latch))
    {
        return ALARM_MULTI;
    }

    if (s_smoke_latch)
    {
        return ALARM_SMOKE;
    }

    if (s_temp_latch)
    {
        return ALARM_TEMP;
    }

    if (s_humi_latch)
    {
        return ALARM_HUMI;
    }

    return ALARM_NONE;
}

/**
 * @brief       报警判定（100 ms 调用一次）
 * @param       d: 当前数据快照（只读）
 * @retval      无
 */
void alarm_update(const env_data_t *d)
{
    if (d == 0)
    {
        return;
    }

    /* ---------------- 烟雾：超上限报警 ---------------- */
    if ((d->valid & ENV_VALID_SMOKE) == 0)
    {
        s_cnt_smoke_up = 0;
        s_cnt_smoke_dn = 0;
    }
    else if (HAL_GetTick() < SMOKE_WARMUP_MS)
    {
        /* 预热窗口内 AO 输出不可信：强制不触发 */
        s_smoke_latch  = 0;
        s_cnt_smoke_up = 0;
        s_cnt_smoke_dn = 0;
    }
    else if (d->smoke_rel > s_th.th_smoke_hi)
    {
        s_cnt_smoke_dn = 0;

        if (s_cnt_smoke_up < CONFIRM_CNT)
        {
            s_cnt_smoke_up++;
        }

        if (s_cnt_smoke_up >= CONFIRM_CNT)
        {
            s_smoke_latch = 1;
        }
    }
    /* 解除条件写成"smoke_rel + HYST < TH_HI"的等价形式：
       smoke_rel < TH_HI - HYST，用无符号运算避免 TH_HI < HYST 时相减回绕 */
    else if (((uint32_t)d->smoke_rel + s_th.hyst[0]) < s_th.th_smoke_hi)
    {
        s_cnt_smoke_up = 0;

        if (s_cnt_smoke_dn < CONFIRM_CNT)
        {
            s_cnt_smoke_dn++;
        }

        if (s_cnt_smoke_dn >= CONFIRM_CNT)
        {
            s_smoke_latch = 0;
        }
    }
    else
    {
        s_cnt_smoke_up = 0;             /* 落在迟滞带内：不触发也不解除 */
        s_cnt_smoke_dn = 0;
    }

    /* ---------------- 温度：超上限报警 ---------------- */
    if ((d->valid & ENV_VALID_DHT11) == 0)
    {
        s_cnt_temp_up = 0;
        s_cnt_temp_dn = 0;
    }
    else if (d->temp_x10 > s_th.th_temp_hi_x10)
    {
        s_cnt_temp_dn = 0;

        if (s_cnt_temp_up < CONFIRM_CNT)
        {
            s_cnt_temp_up++;
        }

        if (s_cnt_temp_up >= CONFIRM_CNT)
        {
            s_temp_latch = 1;
        }
    }
    /* 等价于 temp_x10 < TH_HI - HYST，用 32 位运算避免减法溢出 */
    else if (((int32_t)d->temp_x10 + (int32_t)s_th.hyst[1]) < (int32_t)s_th.th_temp_hi_x10)
    {
        s_cnt_temp_up = 0;

        if (s_cnt_temp_dn < CONFIRM_CNT)
        {
            s_cnt_temp_dn++;
        }

        if (s_cnt_temp_dn >= CONFIRM_CNT)
        {
            s_temp_latch = 0;
        }
    }
    else
    {
        s_cnt_temp_up = 0;
        s_cnt_temp_dn = 0;
    }

    /* ---------------- 湿度：低于下限报警（方向相反） ---------------- */
    if ((d->valid & ENV_VALID_DHT11) == 0)
    {
        s_cnt_humi_up = 0;
        s_cnt_humi_dn = 0;
    }
    else if ((uint16_t)d->humi_x10 < s_th.th_humi_lo_x10)
    {
        s_cnt_humi_dn = 0;

        if (s_cnt_humi_up < CONFIRM_CNT)
        {
            s_cnt_humi_up++;
        }

        if (s_cnt_humi_up >= CONFIRM_CNT)
        {
            s_humi_latch = 1;
        }
    }
    else if ((uint32_t)d->humi_x10 > ((uint32_t)s_th.th_humi_lo_x10 + s_th.hyst[2]))
    {
        s_cnt_humi_up = 0;

        if (s_cnt_humi_dn < CONFIRM_CNT)
        {
            s_cnt_humi_dn++;
        }

        if (s_cnt_humi_dn >= CONFIRM_CNT)
        {
            s_humi_latch = 0;
        }
    }
    else
    {
        s_cnt_humi_up = 0;
        s_cnt_humi_dn = 0;
    }

    s_alarm = alarm_combine();
}

/**
 * @brief       读取当前报警状态
 * @param       无
 * @retval      alarm_state_t
 */
uint8_t alarm_get_state(void)
{
    return s_alarm;
}

/**
 * @brief       报警时基（10 ms 调用一次）
 * @note        只做"输出"，不做判定；判定在 alarm_update() 里。
 * @param       无
 * @retval      无
 */
void alarm_tick(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t  beep_on;

    /* ---- LED0：所有状态下都是 1 Hz 心跳 ---- */
    if ((uint32_t)(now - s_led0_tick) >= ALARM_LED0_HALF_MS)
    {
        s_led0_tick = now;
        led_toggle(LED_ID_0);
    }

    /* ---- LED1：SMOKE / MULTI 常亮，其余灭 ---- */
    led_set(LED_ID_1, (s_alarm == ALARM_SMOKE || s_alarm == ALARM_MULTI) ? 1 : 0);

    /* ---- LED2：TEMP / HUMI / MULTI 以 2 Hz 闪，其余灭 ---- */
    if (s_alarm == ALARM_TEMP || s_alarm == ALARM_HUMI || s_alarm == ALARM_MULTI)
    {
        if ((uint32_t)(now - s_led2_tick) >= ALARM_LED2_HALF_MS)
        {
            s_led2_tick = now;
            led_toggle(LED_ID_2);
        }
    }
    else
    {
        s_led2_tick = now;
        led_off(LED_ID_2);
    }

    /* ---- 蜂鸣器 ---- */
    switch (s_alarm)
    {
        case ALARM_SMOKE:
            beep_on = 1;                                            /* 连续响 */
            break;

        case ALARM_TEMP:
        case ALARM_HUMI:
            beep_on = ((now % BEEP_TEMP_PERIOD_MS) < BEEP_TEMP_ON_MS) ? 1 : 0;
            break;

        case ALARM_MULTI:
            beep_on = ((now % BEEP_MULTI_PERIOD_MS) < BEEP_MULTI_ON_MS) ? 1 : 0;
            break;

        default:
            beep_on = 0;
            break;
    }

    beep_set(beep_on);

    /* ---- LED3：每 200ms 按当前光强重算占空比（与报警状态无关） ---- */
    if ((uint32_t)(now - s_led3_tick) >= ALARM_LED3_PERIOD_MS)
    {
        uint16_t duty;
        uint8_t  lux = env_data_get()->lux;

        s_led3_tick = now;

#if LIGHT_INVERT
        duty = (uint16_t)lux * 10U;                                 /* 亮 → 亮 */
#else
        duty = LED_PWM_FULL - (uint16_t)lux * 10U;                  /* 暗 → 亮（默认） */
#endif

        led_pwm_set(duty);
    }
}

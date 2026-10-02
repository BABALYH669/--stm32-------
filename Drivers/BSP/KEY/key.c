/**
 ****************************************************************************************************
 * @file        key.c
 * @brief       5 方向按键驱动（上 / 下 / 左 / 右 / 确认）  —— 阶段一 T1.5
 *
 * @note        【消抖】每 20 ms 采样一次，连续 2 次电平一致才认为该电平稳定；
 *              只有"稳定电平由松开变按住"时才产生一次按下事件。
 *
 *              【长按】按住期间按两段节拍重复产生事件：
 *                      0 ~ 500 ms      : 只有按下沿那一次
 *                      500 ~ 1500 ms   : 每 150 ms 一次（慢速连续滚动）
 *                      ≥ 1500 ms       : 每 60 ms 一次（"连按"加速）
 *              松开时立刻停止重复，不补报。
 ****************************************************************************************************
 */

#include "./BSP/KEY/key.h"

/* 5 个键分布在三个端口：PA0(确认) / PC0,PC1(左,右) / PE3,PE4(下,上) */
#define KEY_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOA_CLK_ENABLE();   \
                                        __HAL_RCC_GPIOC_CLK_ENABLE();   \
                                        __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       active_level;     /* 1 = 高电平有效（WK_UP 丝印），0 = 低电平有效 */
} key_hw_t;

typedef struct
{
    uint8_t  last_raw;              /* 上次采样到的原始状态（1 = 按住）                */
    uint8_t  stable_cnt;            /* 连续一致的采样次数                              */
    uint8_t  level;                 /* 已确认的稳定状态（1 = 按住）                    */
    uint8_t  pending;               /* 待取走的事件                                    */
    uint8_t  repeating;             /* 已进入长按重复阶段                              */
    uint32_t last_tick;             /* 上次采样时刻                                    */
    uint32_t press_tick;            /* 确认按住的时刻（用于算按住时长）                */
    uint32_t repeat_tick;           /* 上次重复触发的时刻                              */
} key_state_t;

/* 顺序必须与 key_id_t 一致 */
static const key_hw_t s_key_hw[KEY_MAX] =
{
    { KEY_UP_PORT,    KEY_UP_PIN,    0 },   /* 上   —— 板载 KEY0 丝印 */
    { KEY_DOWN_PORT,  KEY_DOWN_PIN,  0 },   /* 下   —— 板载 KEY1 丝印 */
    { KEY_LEFT_PORT,  KEY_LEFT_PIN,  0 },   /* 左   —— 新接          */
    { KEY_RIGHT_PORT, KEY_RIGHT_PIN, 0 },   /* 右   —— 新接          */
    { KEY_OK_PORT,    KEY_OK_PIN,    1 },   /* 确认 —— 板载 WK_UP 丝印，高电平有效 */
};

static key_state_t s_key_state[KEY_MAX];

/**
 * @brief       读取某个按键的"按住"状态（未消抖）
 * @param       id: 按键编号
 * @retval      1 = 按住，0 = 松开
 */
static uint8_t key_read_raw(key_id_t id)
{
    GPIO_PinState ps = HAL_GPIO_ReadPin(s_key_hw[id].port, s_key_hw[id].pin);

    return (ps == GPIO_PIN_SET) ? s_key_hw[id].active_level : (uint8_t)(!s_key_hw[id].active_level);
}

/**
 * @brief       初始化 5 个按键
 * @param       无
 * @retval      无
 * @note        active_level == 1（WK_UP 丝印）走内部下拉，其余 4 个键走内部上拉。
 *              上电一律把 level 记为"未按住"：万一上电瞬间某个键正好被按住（或者新接的
 *              PC0/PC1 还没接好而接地），也不会被当成"一直按住"而立刻狂报长按重复，
 *              而是走正常的 40 ms 消抖后报一次按下沿。
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    uint8_t i;

    KEY_GPIO_CLK_ENABLE();

    gpio_init_struct.Mode  = GPIO_MODE_INPUT;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;

    for (i = 0; i < KEY_MAX; i++)
    {
        gpio_init_struct.Pull = s_key_hw[i].active_level ? GPIO_PULLDOWN : GPIO_PULLUP;
        gpio_init_struct.Pin  = s_key_hw[i].pin;
        HAL_GPIO_Init(s_key_hw[i].port, &gpio_init_struct);

        s_key_state[i].last_raw    = key_read_raw((key_id_t)i);
        s_key_state[i].stable_cnt  = KEY_DEBOUNCE_CNT;
        s_key_state[i].level       = 0;
        s_key_state[i].pending     = 0;
        s_key_state[i].repeating   = 0;
        s_key_state[i].last_tick   = HAL_GetTick();
        s_key_state[i].press_tick  = HAL_GetTick();
        s_key_state[i].repeat_tick = HAL_GetTick();
    }
}

/**
 * @brief       按键扫描（非阻塞，主循环随时调用）
 * @param       id: 按键编号
 * @retval      1 = 本次有一个事件（按下沿 或 长按重复）；0 = 无事件
 * @note        必须在主循环里以远高于 20 ms 的频率调用（例如 10 ms 节拍），
 *              否则采样间隔不准，快速点按会漏，长按重复的节拍也会被拉长。
 */
uint8_t key_scan(key_id_t id)
{
    key_state_t *k;
    uint32_t     now;

    if (id >= KEY_MAX)
    {
        return 0;
    }

    k   = &s_key_state[id];
    now = HAL_GetTick();

    /* ---------------- 1. 每 20 ms 采样一次 + 消抖 ---------------- */
    if ((uint32_t)(now - k->last_tick) >= KEY_SAMPLE_MS)
    {
        uint8_t raw;

        k->last_tick = now;
        raw = key_read_raw(id);

        if (raw == k->last_raw)
        {
            if (k->stable_cnt < KEY_DEBOUNCE_CNT)
            {
                k->stable_cnt++;
            }
        }
        else
        {
            k->last_raw   = raw;
            k->stable_cnt = 1;
        }

        /* 连续 KEY_DEBOUNCE_CNT 次一致 → 确认电平翻转 */
        if ((k->stable_cnt >= KEY_DEBOUNCE_CNT) && (k->level != raw))
        {
            k->level = raw;

            if (raw)
            {
                /* 按下沿：立刻报一次，并从此刻开始计长按时长 */
                k->press_tick  = now;
                k->repeat_tick = now;
                k->repeating   = 0;
                k->pending     = 1;
            }
            else
            {
                k->repeating = 0;           /* 松开：停止重复 */
            }
        }
    }

    /* ---------------- 2. 长按重复 / 连按加速 ---------------- */
    if (k->level)
    {
        uint32_t held = (uint32_t)(now - k->press_tick);

        if (held >= KEY_LONG_PRESS_MS)
        {
            uint32_t interval = (held >= KEY_ACCEL_PRESS_MS) ? KEY_REPEAT_FAST_MS : KEY_REPEAT_MS;

            if (!k->repeating)
            {
                k->repeating   = 1;
                k->repeat_tick = now;       /* 进入重复的这一刻重新计时，避免立刻多报一次 */
            }

            if ((uint32_t)(now - k->repeat_tick) >= interval)
            {
                k->repeat_tick = now;
                k->pending     = 1;
            }
        }
    }

    if (k->pending)
    {
        k->pending = 0;
        return 1;
    }

    return 0;
}

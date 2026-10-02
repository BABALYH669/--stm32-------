/**
 ****************************************************************************************************
 * @file        lcd_ui.c
 * @brief       用 LCD 直接显示环境数据（不使用 LVGL）—— 4 个传感器页 + 触摸滑动切换
 *
 * @note        1. 一页只显示一个传感器（TEMP / HUMI / SMOKE / LUX），左右滑动或点半屏切换。
 *              2. 页面切换时才整屏重画；平时只刷"数值/状态"那几小块，所以 1 Hz 刷新不闪。
 *              3. 中文字库没上，屏上文字一律用 ASCII。
 ****************************************************************************************************
 */

#include "./BSP/LCD/lcd.h"
#include "./BSP/TP/tp.h"
#include "./BSP/NET/net.h"
#include "app_config.h"
#include "alarm.h"
#include "env_data.h"
#include "lcd_ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 正点原子的调色板里没有橙色，按 RGB565 自己补一个 */
#define ORANGE          0xFD20

/* ---------------- 交互参数 ---------------- */
#define UI_SWIPE_MIN    40U     /* 水平位移 ≥ 40 像素才算"滑动"            */
#define UI_TAP_MAX      15U     /* 位移 < 15 像素算"点击"                  */
#define UI_TAP_MS       600U    /* 点击的按下时长上限                      */
#define UI_SWIPE_MS     1200U   /* 滑动的按下时长上限（超过就当长按，不切页）*/
#define UI_PAGE_COOLDOWN_MS 400U /* 切页后的冷却时间：防止一次误触连翻好几页 */
#define UI_RELEASE_MS   40U     /* 抬手去抖：连续这么久都读不到触摸，才算真的抬手 */

/* ---------------- 布局（320×480 竖屏）---------------- */
#define UI_Y_HEADER      6U
#define UI_Y_HLINE1     28U
#define UI_Y_LABEL     105U
#define UI_Y_VALUE     155U
#define UI_Y_UNIT      210U
#define UI_Y_HLINE2    265U
#define UI_Y_ALARM_T   285U
#define UI_Y_ALARM_V   308U
#define UI_Y_DOTS      380U
#define UI_Y_HINT      445U

/* ---------------- 页面表 ---------------- */
#define UI_PAGE_COUNT   4U

typedef struct
{
    const char *label;      /* 大字标签 */
    const char *unit;       /* 单位     */
    uint16_t    color;      /* 数值颜色 */
} ui_page_def_t;

static const ui_page_def_t s_pages[UI_PAGE_COUNT] =
{
    { "TEMP",  "C",    BLUE    },       /* 页 0：温度 */
    { "HUMI",  "%RH",  GREEN   },       /* 页 1：湿度 */
    { "SMOKE", "REL",  ORANGE  },       /* 页 2：烟雾（相对基线值）*/
    { "LUX",   "/100", MAGENTA },       /* 页 3：光强 */
};

static uint8_t s_page = 0;

/* 触摸手势状态 */
static uint8_t  s_touch_down = 0;
static uint16_t s_start_x, s_start_y;       /* 按下点 */
static uint16_t s_last_x,  s_last_y;        /* 最后已知位置 */
static uint32_t s_down_tick;
static uint32_t s_page_tick;                /* 上次切页的时刻（冷却用）*/
static uint8_t  s_releasing;                /* 1 = 正在确认"抬手" */
static uint32_t s_release_tick;             /* 开始确认抬手的时刻 */

/* ================================ 小工具 ================================ */

/** @brief 居中显示一行字符串（字号 size，字符宽 = size/2）*/
static void ui_center_string(uint16_t y, uint8_t size, const char *s, uint16_t color)
{
    uint16_t w = (uint16_t)(strlen(s) * (uint16_t)(size / 2U));
    uint16_t x = (lcddev.width > w) ? (uint16_t)((lcddev.width - w) / 2U) : 0U;

    lcd_show_string(x, y, lcddev.width, size, size, (char *)s, color);
}

/** @brief 取当前页要显示的值（字符串）*/
static void ui_page_value(uint8_t page, char *buf, uint32_t bufsz)
{
    const env_data_t *d = env_data_get();

    switch (page)
    {
        case 0:     /* 温度：×10 的整数拆成 整数.小数，避免用浮点 */
        {
            int32_t t  = d->temp_x10;
            int32_t ti = t / 10;
            int32_t td = t % 10;

            if (td < 0)
            {
                td = -td;
            }

            (void)snprintf(buf, bufsz, "%ld.%ld", (long)ti, (long)td);
            break;
        }

        case 1:     /* 湿度 */
            (void)snprintf(buf, bufsz, "%u.%u",
                           (unsigned)(d->humi_x10 / 10U), (unsigned)(d->humi_x10 % 10U));
            break;

        case 2:     /* 烟雾（相对基线）*/
            (void)snprintf(buf, bufsz, "%u", (unsigned)d->smoke_rel);
            break;

        default:    /* 光强 */
            (void)snprintf(buf, bufsz, "%u", (unsigned)d->lux);
            break;
    }
}

/* ================================ 三个固定的显示块 ================================ */

/** @brief 顶部状态栏：NET / 页码 / SEQ */
static void ui_draw_header(void)
{
    char buf[24];

    lcd_fill(0, 0, lcddev.width - 1U, UI_Y_HLINE1, WHITE);

    if (net_is_online())
    {
        lcd_show_string(6, UI_Y_HEADER, 110, 16, 16, (char *)"NET:ON ", GREEN);
    }
    else
    {
        lcd_show_string(6, UI_Y_HEADER, 110, 16, 16, (char *)"NET:OFF", RED);
    }

    (void)snprintf(buf, sizeof(buf), "%u/%u", (unsigned)(s_page + 1U), (unsigned)UI_PAGE_COUNT);
    lcd_show_string((uint16_t)(lcddev.width / 2U - 16U), UI_Y_HEADER, 60, 16, 16, buf, BLUE);

    (void)snprintf(buf, sizeof(buf), "SEQ:%lu", (unsigned long)env_data_get()->seq);
    lcd_show_string((uint16_t)(lcddev.width - 100U), UI_Y_HEADER, 96, 16, 16, buf, BLACK);
}

/** @brief 报警状态（用颜色区分）*/
static void ui_draw_alarm(void)
{
    uint8_t     st = alarm_get_state();
    const char *txt;
    uint16_t    col;

    switch (st)
    {
        case ALARM_SMOKE: txt = "SMOKE !"; col = RED;    break;
        case ALARM_TEMP:  txt = "TEMP HI"; col = ORANGE; break;
        case ALARM_HUMI:  txt = "HUMI LO"; col = ORANGE; break;
        case ALARM_MULTI: txt = "MULTI !!"; col = RED;   break;
        default:          txt = "NORMAL";  col = GREEN;  break;
    }

    lcd_fill(0, UI_Y_ALARM_T, lcddev.width - 1U, UI_Y_ALARM_V + 24U, WHITE);
    lcd_show_string(6, UI_Y_ALARM_T, 110, 16, 16, (char *)"ALARM", BLACK);
    lcd_show_string(120, UI_Y_ALARM_V, 200, 24, 24, (char *)txt, col);
}

/** @brief 底部页码圆点（当前页画大且高亮）*/
static void ui_draw_dots(void)
{
    uint8_t  i;
    uint16_t pitch = 18U;
    uint16_t x0 = (uint16_t)((lcddev.width - (UI_PAGE_COUNT - 1U) * pitch) / 2U);

    lcd_fill(0, UI_Y_DOTS - 8U, lcddev.width - 1U, UI_Y_DOTS + 8U, WHITE);

    for (i = 0; i < UI_PAGE_COUNT; i++)
    {
        uint16_t x = (uint16_t)(x0 + i * pitch);

        if (i == s_page)
        {
            lcd_fill_circle(x, UI_Y_DOTS, 5, BLUE);
        }
        else
        {
            lcd_fill_circle(x, UI_Y_DOTS, 3, GRAY);
        }
    }
}

/* ================================ 整页绘制 ================================ */

/**
 * @brief       画出某一页（整屏重画）
 * @param       page: 页码 0 ~ UI_PAGE_COUNT-1
 * @retval      无
 */
static void ui_page_show(uint8_t page)
{
    char val[16];

    s_page = page;

    lcd_clear(WHITE);           /* 约 60ms，只在切页/上电时发生 */

    /* 顶部状态栏 */
    ui_draw_header();
    lcd_draw_hline(0, UI_Y_HLINE1, lcddev.width, GRAY);

    /* 中间：标签 / 数值 / 单位（都居中）*/
    ui_center_string(UI_Y_LABEL, 24, s_pages[page].label, BLACK);

    ui_page_value(page, val, sizeof(val));
    ui_center_string(UI_Y_VALUE, 32, val, s_pages[page].color);

    ui_center_string(UI_Y_UNIT, 24, s_pages[page].unit, GRAY);

    /* 报警区 */
    lcd_draw_hline(0, UI_Y_HLINE2, lcddev.width, GRAY);
    ui_draw_alarm();

    /* 页码 + 操作提示 */
    ui_draw_dots();
    ui_center_string(UI_Y_HINT, 16, "swipe or tap to switch", GRAY);
}

/* ================================ 对外接口 ================================ */

/**
 * @brief       初始化 LCD 并显示第 1 页
 * @note        触摸的初始化与校准不在这里做，见 main.c 的 tp_init()/tp_calibrate()。
 * @param       无
 * @retval      无
 */
void lcd_ui_init(void)
{
    lcd_init();
    lcd_display_dir(0);                 /* 竖屏 320 × 480 */

    g_back_color  = WHITE;              /* 文字背景填充成底色，避免黑底 */
    g_point_color = BLACK;

    ui_page_show(0);
}

/**
 * @brief       刷新当前页（建议 1 Hz 调用）
 * @note        只重画会变的那几块（数值 / 状态栏 / 报警），不整屏重画，所以不闪。
 * @param       无
 * @retval      无
 */
void lcd_ui_refresh(void)
{
    char val[16];

    /* 数值区：整条清掉再居中写，避免新旧字符串长度不同留下残影 */
    lcd_fill(0, UI_Y_VALUE - 4U, lcddev.width - 1U, UI_Y_VALUE + 34U, WHITE);
    ui_page_value(s_page, val, sizeof(val));
    ui_center_string(UI_Y_VALUE, 32, val, s_pages[s_page].color);

    ui_draw_header();
    ui_draw_alarm();
}

/* ================================ 触摸手势 ================================ */

/** @brief 切到上一页（右滑 / 点左半屏）*/
static void ui_prev_page(void)
{
    uint8_t p = (s_page == 0U) ? (UI_PAGE_COUNT - 1U) : (uint8_t)(s_page - 1U);

    ui_page_show(p);
    s_page_tick = HAL_GetTick();
}

/** @brief 切到下一页（左滑 / 点右半屏）*/
static void ui_next_page(void)
{
    uint8_t p = (uint8_t)((s_page + 1U) % UI_PAGE_COUNT);

    ui_page_show(p);
    s_page_tick = HAL_GetTick();
}

/** @brief 手动切到下一页（供按键调用）*/
void lcd_ui_page_next(void)
{
    ui_next_page();
}

/** @brief 手动切到上一页（供按键调用）*/
void lcd_ui_page_prev(void)
{
    ui_prev_page();
}

/**
 * @brief       触摸轮询与手势识别（建议 10 ms 调用）
 * @note        采用"按下 → 记录起点 / 抬起 → 一次性判定"的方式：
 *                水平位移 ≥ UI_SWIPE_MIN 且水平分量明显大于垂直 → 滑动切页；
 *                位移很小、按住时间短 → 当成点击（左半屏上一页、右半屏下一页）。
 * @param       无
 * @retval      无
 */
void lcd_ui_touch_poll(void)
{
    tp_point_t p;
    uint8_t    now;

    /* ① 未校准 → 坐标不可信，干脆完全不响应触摸。
       否则 T_PEN 引脚被干扰/悬空时，噪声会被当成"连续点击"而让页面自动翻页。 */
    if (!tp_is_calibrated())
    {
        return;
    }

    /* ② 切页后的冷却时间：防止一次误触连翻好几页 */
    if ((uint32_t)(HAL_GetTick() - s_page_tick) < UI_PAGE_COOLDOWN_MS)
    {
        return;
    }

    now = tp_scan(&p);

    if (now)
    {
        s_releasing = 0;                /* 又读到触摸 → 取消"抬手确认" */

        if (!s_touch_down)              /* 按下沿 */
        {
            s_touch_down = 1;
            s_start_x    = p.x;
            s_start_y    = p.y;
            s_last_x     = p.x;
            s_last_y     = p.y;
            s_down_tick  = HAL_GetTick();
        }
        else                            /* 按住中：只更新最新位置 */
        {
            s_last_x = p.x;
            s_last_y = p.y;
        }

        return;
    }

    if (!s_touch_down)                  /* 本来就没按下 */
    {
        return;
    }

    /* ★ 抬手去抖：电容屏的上报是有间歇的（手指在移动时可能漏报几帧）。
       如果一读不到就判"抬手"，一次滑动会被拆成"滑动 + 点击"，导致多切一页。
       所以先等 UI_RELEASE_MS，期间持续读不到才认为真的抬手。 */
    if (!s_releasing)
    {
        s_releasing    = 1;
        s_release_tick = HAL_GetTick();
        return;
    }

    if ((uint32_t)(HAL_GetTick() - s_release_tick) < UI_RELEASE_MS)
    {
        return;
    }

    s_releasing = 0;

    /* ---------------- 抬起：判定手势 ---------------- */
    {
        int32_t  dx = (int32_t)s_last_x - (int32_t)s_start_x;
        int32_t  dy = (int32_t)s_last_y - (int32_t)s_start_y;
        uint32_t dt = (uint32_t)(HAL_GetTick() - s_down_tick);
        int32_t  adx = (dx < 0) ? -dx : dx;
        int32_t  ady = (dy < 0) ? -dy : dy;

        s_touch_down = 0;

        /* 诊断：抬手时把这一笔的位移和坐标打出来。
           电容屏的坐标范围如果不是 0~319 / 0~479（例如是 0~4095），
           从这里就能立刻看出来，再决定要不要做缩放。 */
        printf("[UI] dx=%ld dy=%ld dt=%lums (%u,%u)->(%u,%u)\r\n",
               (long)dx, (long)dy, (unsigned long)dt,
               (unsigned)s_start_x, (unsigned)s_start_y,
               (unsigned)s_last_x, (unsigned)s_last_y);

        /* 1. 滑动：水平位移够大、且水平分量明显大于垂直分量、时间不长 */
        if ((adx >= (int32_t)UI_SWIPE_MIN) && (adx > (ady * 3 / 2)) && (dt < UI_SWIPE_MS))
        {
            if (dx > 0)
            {
                ui_prev_page();         /* 右滑 → 上一页 */
            }
            else
            {
                ui_next_page();         /* 左滑 → 下一页 */
            }

            return;
        }

        /* 2. 点击：位移很小、时间不长 → 左半屏上一页 / 右半屏下一页 */
        if ((adx < (int32_t)UI_TAP_MAX) && (ady < (int32_t)UI_TAP_MAX) && (dt < UI_TAP_MS))
        {
            if (s_last_x < (lcddev.width / 2U))
            {
                ui_prev_page();
            }
            else
            {
                ui_next_page();
            }
        }
    }
}

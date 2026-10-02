/**
 ****************************************************************************************************
 * @file        net.c
 * @brief       ESP8266 + MQTT 3.1.1 通信层  —— 阶段一 T1.11
 *
 * @note        【架构：设备侧直连 broker（2026-09-22 用户拍板）】
 *
 *                  STM32 ──USART2──▶ ESP8266 ──TCP 透传──▶ broker ──▶ MQTT Explorer
 *                     │                  ▲
 *                     └── AT+CIPMODE=1 进入透传后，串口上跑的就是裸 MQTT 3.1.1 报文
 *
 *              【三个关键设计决定】
 *
 *              1. 为什么自己实现 MQTT 而不用 AT+MQTTxxx 命令？
 *                 依赖 AT 固件的 MQTT 命令 = 固件版本/行为不可控（v2.2 的 R2 风险）。
 *                 自己组报文后，只依赖最基础的 AT，且协议行为完全可见可控。
 *
 *              2. 为什么一旦进入透传就只能靠硬复位重连？
 *                 AT+CIPMODE=1 + AT+CIPSEND 之后，串口上所有字节都被当成 TCP 数据，
 *                 AT 命令通道消失，`+++` 退出又要求前后各 1 s 静默（很难在非阻塞主循环里保证）。
 *                 所以本模块的重连策略是：**硬复位 + 全流程重连**，而且复位等待全部走状态机，
 *                 不阻塞主循环（每次 net_poll 只推进一小步）。
 *
 *              3. 为什么不用 DMA、用"中断收 + 阻塞发"？
 *                 - RX：自己开 RXNE 中断推环形缓冲，不走 HAL_UART_Receive_IT，
 *                       避免和 usart.c 里已有的 HAL_UART_RxCpltCallback()（弱符号）抢回调；
 *                 - TX：单条 MQTT 报文 ≤160 B，115200 下 ≈14 ms，阻塞发最简单可靠。
 *                 （《阶段三计划书》§4.1 的 UART IDLE + DMA 方案随时可换，属内部实现。）
 *
 *              【超长 / 粘包 / 半包的处理】
 *                 - 透传模式下没有 `+IPD,<len>:` 前缀，数据是裸流，解析反而更简单；
 *                 - AT 阶段：滚动响应缓冲只做子串匹配，单行 > NET_LINE_MAX 只计数不参与匹配；
 *                 - MQTT 阶段：mqtt_parser_feed() 是一个逐字节状态机，
 *                   "剩余长度"先收完再按长度精确取报文体，因此粘包/半包都天然正确；
 *                   报文体超过 MQTT_BODY_MAX 时整条丢弃（DRAIN 状态），不影响后续报文同步。
 ****************************************************************************************************
 */

#include "./BSP/NET/net.h"
#include "./BSP/MQTT/mqtt.h"
#include "app_config.h"
#include "alarm.h"
#include "./SYSTEM/delay/delay.h"

#include <string.h>
#include <stdio.h>

/* ================================================================================================
 * 1. 内部常量
 * ================================================================================================ */
#define NET_RESP_MAX                    256U    /* AT 响应滚动缓冲（只看尾部，不用太大）    */
#define NET_ESP_RESET_LOW_MS            200U    /* RST 低电平保持                            */
#define NET_ESP_RESET_WAIT_MS           3000U   /* 复位后等模块启动（供电弱时启动慢）        */
#define NET_CIPSTART_TIMEOUT_MS         10000U  /* AT+CIPSTART 建立 TCP 的超时               */
#define NET_CIPSEND_TIMEOUT_MS          3000U   /* AT+CIPSEND 等 '>' 的超时                  */
#define NET_MQTT_CONNACK_TIMEOUT_MS     5000U   /* 等 CONNACK 的超时                         */
#define NET_MQTT_SUBACK_TIMEOUT_MS      5000U   /* 等 SUBACK 的超时                          */
#define NET_STEP_FAIL_LIMIT             3U      /* 同一(类)步骤连续失败 3 次 → 进入退避       */
#define NET_DRAIN_MAX_PER_POLL          256U    /* 单次 net_poll 最多消费多少字节            */
#define NET_TX_BUF_SIZE                 320U    /* 单条 MQTT 报文的组包缓冲                  */

/* ================================================================================================
 * 2. 类型与模块状态
 * ================================================================================================ */

/* AT / 连接时序状态。顺序即流水线顺序，net_enter_state(s_state + 1) 表示"前进一步"。 */
typedef enum
{
    NET_ST_RESET = 0,           /* 硬复位模组                       */
    NET_ST_AT,                  /* AT                               */
    NET_ST_CWMODE,              /* AT+CWMODE=1（STA 模式）           */
    NET_ST_CWJAP,               /* AT+CWJAP 连 WiFi                 */
    NET_ST_CIPMUX,              /* AT+CIPMUX=0（单连接）             */
    NET_ST_CIPMODE,             /* AT+CIPMODE=1（进入透传模式）      */
    NET_ST_CIPSTART,            /* AT+CIPSTART 连 broker            */
    NET_ST_CIPSEND,             /* AT+CIPSEND 进入透传，等 '>'       */
    NET_ST_MQTT_CONNECT,        /* 发 MQTT CONNECT，等 CONNACK       */
    NET_ST_MQTT_SUB_ACK,        /* 订阅 env/{dev}/ack，等 SUBACK     */
    NET_ST_MQTT_SUB_CMD,        /* 订阅 env/{dev}/cmd，等 SUBACK     */
    NET_ST_ONLINE,              /* 已连上 broker：收发 / 保活        */
    NET_ST_BACKOFF              /* 退避等待后重新复位重连            */
} net_state_t;

static UART_HandleTypeDef s_esp_uart;

/* ---- 连接与状态机 ---- */
static uint8_t     s_mqtt_connected;        /* 1 = CONNACK 成功，可以收发 */
static net_state_t s_state;
static uint32_t    s_step_tick;
static uint8_t     s_step_fail;
static uint8_t     s_reset_phase;           /* RESET 的两步：0=拉低中，1=已拉高等待启动 */
static uint8_t     s_backoff_exp;
static uint32_t    s_backoff_ms;

/* ---- 接收环形缓冲（中断写 / 主循环读）---- */
static uint8_t          s_rx_ring[NET_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;

/* ---- AT 阶段：滚动响应缓冲 ---- */
static char     s_resp[NET_RESP_MAX];
static uint16_t s_resp_len;
static uint16_t s_at_line_len;

/* ---- MQTT 阶段：报文解析与事件 ---- */
static mqtt_parser_t s_mqtt;
static uint8_t       s_mqtt_evt;            /* 待状态机处理的 MQTT 事件 */
static uint8_t       s_mqtt_connack_rc;
static uint16_t      s_packet_id;           /* 报文标识符自增 */

/* ---- 发送 ---- */
static uint8_t  s_tx_buf[NET_TX_BUF_SIZE];  /* MQTT 报数组包缓冲 */
static char     s_cmd[160];                 /* AT 命令行缓冲     */
static char     s_tx_line[NET_LINE_MAX + 1];/* 待发的 JSON 行（不含 '\n'）*/
static uint8_t  s_tx_pending;
static env_data_t s_tx_data;
static uint8_t    s_tx_data_valid;

/* ---- 确认（ACK）---- */
static env_data_t s_sent_data;              /* 最近一次真正发出去的数据 */
static uint8_t    s_sent_valid;
static uint8_t    s_pub_pending;            /* QoS1 PUBLISH 等 PUBACK */
static uint32_t   s_pub_tick;
static uint8_t    s_puback_fail;
static uint32_t   s_puback_count;

static uint32_t   s_ack_count;              /* 累计"上传成功"次数（两种模式都用，见下）*/

#if (MQTT_USE_APP_ACK == 1)
/* 下面这套只在「等后台应用 ACK」模式（MQTT_USE_APP_ACK == 1）下使用；
   若改成 0（用 broker 的 QoS1 PUBACK 判成功），它们就没有读写点了，
   包在 #if 里既能避免"set but never used"告警，也让意图更清楚。 */
static uint8_t    s_ack_waiting;
static uint32_t   s_ack_tick;
static uint8_t    s_ack_fail;
static uint32_t   s_ack_got_seq;
static uint8_t    s_ack_got_valid;
#endif

static env_data_t s_last_ack_data;
static uint8_t    s_last_ack_valid;

/* ---- 保活 ---- */
static uint8_t  s_ping_pending;
static uint32_t s_ping_tick;

/* ---- 统计（调试用）---- */
static uint16_t s_oversize_lines;
static uint16_t s_rx_overflow;
static uint16_t s_tx_dropped;
static uint32_t s_rx_total;             /* 累计从 ESP 收到的字节数（诊断：0 = 物理层完全不通）*/

/**
 * @brief       把状态机步骤名转成字符串（只用于日志）
 * @param       st: 步骤
 * @retval      常量字符串
 */
static const char *net_state_name(net_state_t st)
{
    switch (st)
    {
        case NET_ST_RESET:        return "RESET";
        case NET_ST_AT:           return "AT";
        case NET_ST_CWMODE:       return "CWMODE";
        case NET_ST_CWJAP:        return "CWJAP";
        case NET_ST_CIPMUX:       return "CIPMUX";
        case NET_ST_CIPMODE:      return "CIPMODE";
        case NET_ST_CIPSTART:     return "CIPSTART";
        case NET_ST_CIPSEND:      return "CIPSEND";
        case NET_ST_MQTT_CONNECT: return "MQTT_CONNECT";
        case NET_ST_MQTT_SUB_ACK: return "MQTT_SUB_ACK";
        case NET_ST_MQTT_SUB_CMD: return "MQTT_SUB_CMD";
        case NET_ST_ONLINE:       return "ONLINE";
        default:                  return "BACKOFF";
    }
}

/**
 * @brief       把 AT 响应缓冲的尾部打印出来（诊断用）
 * @note        1. s_resp 里已经把 \r \n 过滤掉了，是连续文本，直接 %s 打印即可。
 *              2. 只取最后 120 个字符：前面大多是模块启动信息和命令回显，没有诊断价值。
 *              3. ⚠️ 模块默认开着命令回显（ATE1），所以这里会带上 AT 命令原文，
 *                 其中**包含 WiFi 密码**。自己调试看没问题，要发给别人时记得把那一段打码。
 */
static void net_dump_tail(void)
{
    uint16_t len = s_resp_len;

    if (len == 0U)
    {
        printf("[NET]   resp: (empty)\r\n");
        return;
    }

    if (len > 120U)
    {
        len = 120U;
    }

    printf("[NET]   resp tail: %s\r\n", &s_resp[s_resp_len - len]);
}

/* ================================================================================================
 * 3. 环形缓冲
 * ================================================================================================ */
static void net_ring_push(uint8_t b)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % NET_RX_RING_SIZE);

    if (next == s_rx_tail)
    {
        s_rx_overflow++;
        return;
    }

    s_rx_ring[s_rx_head] = b;
    s_rx_head = next;
}

static uint8_t net_ring_pop(uint8_t *b)
{
    if (s_rx_tail == s_rx_head)
    {
        return 0;
    }

    *b = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % NET_RX_RING_SIZE);

    return 1;
}

static void net_rx_clear(void)
{
    s_rx_tail = s_rx_head;
}

/* ================================================================================================
 * 4. AT 阶段的响应缓冲与模式匹配
 * ================================================================================================ */
static void resp_reset(void)
{
    s_resp_len   = 0;
    s_resp[0]    = '\0';
    s_at_line_len = 0;
}

static void resp_push(uint8_t b)
{
    if (s_resp_len >= (NET_RESP_MAX - 1U))
    {
        uint16_t keep = NET_RESP_MAX / 2U;

        memmove(s_resp, &s_resp[s_resp_len - keep], keep);
        s_resp_len = keep;
    }

    s_resp[s_resp_len++] = (char)b;
    s_resp[s_resp_len]   = '\0';
}

static uint8_t resp_has(const char *pat)
{
    if ((pat == 0) || (s_resp_len == 0U))
    {
        return 0;
    }

    return (strstr(s_resp, pat) != 0) ? 1U : 0U;
}

/**
 * @brief       通用响应判定
 * @retval      0 = 继续等；1 = 成功；2 = 失败
 */
static uint8_t net_check_resp(const char *ok1, const char *ok2, const char *bad)
{
    if (resp_has(bad))
    {
        return 2;
    }

    if (resp_has(ok1))
    {
        return 1;
    }

    if (resp_has(ok2))
    {
        return 1;
    }

    return 0;
}

/**
 * @brief       从 JSON 文本里取无符号整数
 * @param       line: JSON 文本
 * @param       key : 形如 "\"seq\":" 的键
 * @param       out : 输出
 * @retval      1 = 找到；0 = 没找到
 */
static uint8_t net_json_u32(const char *line, const char *key, uint32_t *out)
{
    const char *p = strstr(line, key);
    uint32_t    v = 0;
    uint8_t     digits = 0;

    if (p == 0)
    {
        return 0;
    }

    p += strlen(key);

    while ((*p >= '0') && (*p <= '9'))
    {
        v = v * 10U + (uint32_t)(*p - '0');
        p++;
        digits = 1;
    }

    if (!digits)
    {
        return 0;
    }

    *out = v;

    return 1;
}

/* ================================================================================================
 * 5. USART2 与 ESP8266 控制脚
 * ================================================================================================ */
static void net_uart_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    gpio_init_struct.Pin   = ESP_TX_PIN;                /* PA2 = USART2_TX */
    gpio_init_struct.Mode  = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull  = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ESP_TX_PORT, &gpio_init_struct);

    gpio_init_struct.Pin  = ESP_RX_PIN;                 /* PA3 = USART2_RX */
    gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;
    HAL_GPIO_Init(ESP_RX_PORT, &gpio_init_struct);

    s_esp_uart.Instance          = ESP_UART;
    s_esp_uart.Init.BaudRate     = 115200;
    s_esp_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    s_esp_uart.Init.StopBits     = UART_STOPBITS_1;
    s_esp_uart.Init.Parity       = UART_PARITY_NONE;
    s_esp_uart.Init.Mode         = UART_MODE_TX_RX;
    s_esp_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_esp_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_esp_uart);     /* 会调用 usart.c 里的 HAL_UART_MspInit：它只处理 USART1 */

    __HAL_UART_ENABLE_IT(&s_esp_uart, UART_IT_RXNE);
    HAL_NVIC_SetPriority(ESP_UART_IRQn, 3, 3);
    HAL_NVIC_EnableIRQ(ESP_UART_IRQn);
}

/**
 * @brief       USART2 中断服务函数（只搬运字节，绝不打印 / 不解析）
 */
void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&s_esp_uart, UART_FLAG_RXNE) != RESET)
    {
        uint8_t b = (uint8_t)(s_esp_uart.Instance->DR & 0xFFU);     /* 读 DR 同时清 RXNE */

        net_ring_push(b);
    }

    if (__HAL_UART_GET_FLAG(&s_esp_uart, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(&s_esp_uart);
    }
}

static void net_esp_pins_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio_init_struct.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull  = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;

    gpio_init_struct.Pin = ESP_EN_PIN;
    HAL_GPIO_Init(ESP_EN_PORT, &gpio_init_struct);
    gpio_init_struct.Pin = ESP_RST_PIN;
    HAL_GPIO_Init(ESP_RST_PORT, &gpio_init_struct);

    HAL_GPIO_WritePin(ESP_RST_PORT, ESP_RST_PIN, GPIO_PIN_SET);     /* 先退出复位 */
    HAL_GPIO_WritePin(ESP_EN_PORT, ESP_EN_PIN, GPIO_PIN_SET);       /* EN 必须拉高，否则模组不工作 */
}

static void net_uart_write(const uint8_t *s, uint16_t len)
{
    if ((s == 0) || (len == 0U))
    {
        return;
    }

    (void)HAL_UART_Transmit(&s_esp_uart, (uint8_t *)s, len, 100);
}

static void net_send_str(const char *s)
{
    net_uart_write((const uint8_t *)s, (uint16_t)strlen(s));
}

/* ================================================================================================
 * 6. AT 阶段：字节流 → 滚动响应缓冲
 * ================================================================================================ */
static void net_at_parser_feed(uint8_t b)
{
    if (b == '\n')
    {
        s_at_line_len = 0;
        return;
    }

    if (b == '\r')
    {
        return;
    }

    /* 注意：AT 响应不一定带换行（例如 AT+CIPSEND 的 '>' 提示符），所以逐字节入缓冲 */
    resp_push(b);

    s_at_line_len++;

    if (s_at_line_len > NET_LINE_MAX)
    {
        /* 单行过长：只计数（模块的启动信息/异常输出），不参与模式匹配 */
        s_oversize_lines++;
        s_at_line_len = 0;
    }
}

/* ================================================================================================
 * 7. MQTT 阶段的收发、事件与下行分发
 * ================================================================================================ */
static uint16_t net_next_packet_id(void)
{
    s_packet_id++;

    if (s_packet_id == 0U)
    {
        s_packet_id = 1U;
    }

    return s_packet_id;
}

/**
 * @brief       处理 MQTT 主题下行消息
 * @param       topic  : 主题
 * @param       payload: 内容
 * @retval      无
 * @note        本阶段只处理 env/{dev}/ack 里的 {"type":"ack","seq":N}；
 *              env/{dev}/cmd 留给阶段三的 cmd_handler。
 */
static void net_handle_downlink(const char *topic, const char *payload)
{
    uint32_t seq = 0;

    if (strcmp(topic, MQTT_TOPIC_ACK) == 0)
    {
        if (strstr(payload, "\"type\":\"ack\"") == 0)
        {
            return;
        }

        if (net_json_u32(payload, "\"seq\":", &seq) == 0)
        {
            return;
        }

#if (MQTT_USE_APP_ACK == 1)
        /* 「等后台应用 ACK」模式：这条应用层 ACK 才是"上传成功"的判据 */
        s_ack_count++;
        s_ack_got_seq   = seq;
        s_ack_got_valid = 1;
        s_ack_waiting   = 0;
        s_ack_fail      = 0;
#else
        /* 「用 broker PUBACK」模式：上传成功已经在 MQTT_EV_PUBACK 分支里计过数了，
           这里绝不能再 s_ack_count++，否则同一条数据会被算两次，ACK 数虚高。 */
        (void)seq;
#endif

        /* 两种模式都要做：保存"最后一次被确认的数据快照"，供阶段二的上报回显页使用 */
        if (s_sent_valid)
        {
            s_last_ack_data  = s_sent_data;
            s_last_ack_valid = 1;
        }

        return;
    }

    /* 其它主题（含阶段三的 cmd）本阶段忽略 */
}

/**
 * @brief       处理一条由 mqtt_parser_feed() 凑齐的报文
 * @param       ev: MQTT_EV_*
 * @retval      无
 */
static void net_mqtt_event(uint8_t ev)
{
    switch (ev)
    {
        case MQTT_EV_CONNACK:
            s_mqtt_connack_rc = s_mqtt.connack_rc;
            s_mqtt_evt        = MQTT_EV_CONNACK;
            break;

        case MQTT_EV_SUBACK:
            s_mqtt_evt = MQTT_EV_SUBACK;
            break;

        case MQTT_EV_PUBACK:
            s_pub_pending = 0;
            s_puback_fail = 0;
            s_puback_count++;

#if (MQTT_USE_APP_ACK == 0)
            /* 直接用 broker 的 PUBACK 当"上传成功"（不需要任何后台程序）*/
            s_ack_count++;
            if (s_sent_valid)
            {
                s_last_ack_data  = s_sent_data;
                s_last_ack_valid = 1;
            }
#endif
            break;

        case MQTT_EV_PINGRESP:
            s_ping_pending = 0;
            break;

        case MQTT_EV_PUBLISH:
            /* QoS1 的下行报文必须回 PUBACK，否则 broker 会不断重发 */
            if (s_mqtt.publish_qos == MQTT_QOS1)
            {
                uint16_t n = mqtt_build_puback(s_tx_buf, NET_TX_BUF_SIZE, s_mqtt.packet_id);

                if (n > 0U)
                {
                    net_uart_write(s_tx_buf, n);
                }
            }

            net_handle_downlink(s_mqtt.topic, s_mqtt.payload);
            break;

        default:
            break;
    }
}

/**
 * @brief       把待发的 JSON 行用 QoS1 PUBLISH 发出去
 * @param       无
 * @retval      无
 */
static void net_do_publish(void)
{
    uint16_t n;
    uint16_t pid = 0;

    if ((uint8_t)MQTT_QOS_LEVEL > 0U)
    {
        pid = net_next_packet_id();
    }

    n = mqtt_build_publish(s_tx_buf, NET_TX_BUF_SIZE, MQTT_TOPIC_TELEMETRY,
                           s_tx_line, (uint8_t)MQTT_QOS_LEVEL, pid);

    if (n == 0U)
    {
        s_tx_pending = 0;
        s_tx_dropped++;
        return;
    }

    net_uart_write(s_tx_buf, n);

    /* 记下"这一条已经发出去了"，收到确认时用它做快照 */
    if (s_tx_data_valid)
    {
        s_sent_data  = s_tx_data;
        s_sent_valid = 1;
    }

    s_tx_pending = 0;

    if ((uint8_t)MQTT_QOS_LEVEL > 0U)
    {
        s_pub_pending = 1;
        s_pub_tick    = HAL_GetTick();
    }

#if (MQTT_USE_APP_ACK == 1)
    /* 需要后台应用回执：这一条还没算"确认"，等 ack 主题 */
    if (s_ack_got_valid && (s_ack_got_seq == s_tx_data.seq))
    {
        s_ack_waiting = 0;              /* 回执已经先到了（同一轮内）*/
    }
    else
    {
        s_ack_waiting = 1;
        s_ack_tick    = HAL_GetTick();
    }
#endif
}

/* ================================================================================================
 * 8. 状态机
 * ================================================================================================ */
static void net_enter_state(net_state_t st);

/**
 * @brief       进入退避（2/4/8/16/32/60 s，上限 NET_BACKOFF_MAX_S）
 */
static void net_enter_backoff(uint32_t now)
{
    s_backoff_ms = (1UL << s_backoff_exp) * 1000UL;

    if (s_backoff_ms > ((uint32_t)NET_BACKOFF_MAX_S * 1000UL))
    {
        s_backoff_ms = (uint32_t)NET_BACKOFF_MAX_S * 1000UL;
    }

    if (s_backoff_exp < 6U)
    {
        s_backoff_exp++;
    }

    /* 诊断：把"卡在哪一步"和"从 ESP 累计收到多少字节"一起打出来 ——
         rx=0  → 物理层完全不通：P5 跳线帽没拔 / TX-RX 没交叉接 / 没共地 / ESP 没供电
         rx>0  → 有数据回来但没匹配上预期响应：波特率不对（收到乱码），或模块还卡在透传模式 */
    printf("[NET] step=%s failed, rx=%luB, backoff %lums\r\n",
           net_state_name(s_state), (unsigned long)s_rx_total, (unsigned long)s_backoff_ms);

    /* AT 类步骤失败时，把 ESP 的原始响应尾部打出来 ——
       一眼就能看出它回的是 FAIL、是 "no AP found"、还是完全没匹配上预期关键字 */
    if (s_state < NET_ST_MQTT_CONNECT)
    {
        net_dump_tail();
    }

    s_state     = NET_ST_BACKOFF;
    s_step_tick = now;
}

/**
 * @brief       当前步骤失败：同一步骤连续失败 3 次 → 退避；否则立即重试该步骤
 */
static void net_step_failed(void)
{
    s_step_fail++;

    if (s_step_fail >= NET_STEP_FAIL_LIMIT)
    {
        s_step_fail = 0;
        net_enter_backoff(HAL_GetTick());
    }
    else
    {
        net_enter_state(s_state);
    }
}

/**
 * @brief       连接已确定失效（保活超时 / PUBACK 连续超时 / ACK 连续超时）
 * @note        透传模式下 AT 通道已消失，唯一可靠的退出方式是硬复位：
 *              第 1、2 次立即复位重连，第 3 次先退避再复位（防止 WiFi 长时间不可用时反复重启模组）。
 */
static void net_conn_lost(const char *why)
{
    printf("[NET] %s\r\n", why);

    s_mqtt_connected = 0;
    s_tx_pending     = 0;
    s_pub_pending    = 0;
    s_ping_pending   = 0;
#if (MQTT_USE_APP_ACK == 1)
    s_ack_waiting    = 0;
#endif

    s_step_fail++;

    if (s_step_fail >= NET_STEP_FAIL_LIMIT)
    {
        s_step_fail = 0;
        net_enter_backoff(HAL_GetTick());
    }
    else
    {
        net_enter_state(NET_ST_RESET);
    }
}

static void net_enter_state(net_state_t st)
{
    uint16_t n = 0;

    s_state     = st;
    s_step_tick = HAL_GetTick();
    resp_reset();

    switch (st)
    {
        case NET_ST_RESET:
            s_mqtt_connected = 0;
            s_reset_phase    = 0;
            HAL_GPIO_WritePin(ESP_RST_PORT, ESP_RST_PIN, GPIO_PIN_RESET);
            break;

        case NET_ST_AT:
            net_send_str("AT\r\n");
            break;

        case NET_ST_CWMODE:
            net_send_str("AT+CWMODE=1\r\n");
            break;

        case NET_ST_CWJAP:
            (void)snprintf(s_cmd, sizeof(s_cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI_SSID, WIFI_PWD);
            net_send_str(s_cmd);
            break;

        case NET_ST_CIPMUX:
            net_send_str("AT+CIPMUX=0\r\n");
            break;

        case NET_ST_CIPMODE:
            net_send_str("AT+CIPMODE=1\r\n");           /* 透传模式 */
            break;

        case NET_ST_CIPSTART:
            (void)snprintf(s_cmd, sizeof(s_cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",
                           MQTT_HOST, (unsigned)MQTT_PORT);
            net_send_str(s_cmd);
            break;

        case NET_ST_CIPSEND:
            net_send_str("AT+CIPSEND\r\n");             /* 透传模式下不带长度 */
            break;

        case NET_ST_MQTT_CONNECT:
            /* 进透传后先丢掉 AT 阶段的残留（"OK" / ">" 回显），免得污染 MQTT 报文流 */
            net_rx_clear();
            mqtt_parser_reset(&s_mqtt);
            s_packet_id = 0;

            n = mqtt_build_connect(s_tx_buf, NET_TX_BUF_SIZE, DEV_ID,
                                   MQTT_USERNAME, MQTT_PASSWORD, (uint16_t)MQTT_KEEPALIVE_S);
            if (n > 0U)
            {
                net_uart_write(s_tx_buf, n);
            }
            else
            {
                net_step_failed();
            }
            break;

        case NET_ST_MQTT_SUB_ACK:
            n = mqtt_build_subscribe(s_tx_buf, NET_TX_BUF_SIZE, MQTT_TOPIC_ACK,
                                     (uint8_t)MQTT_QOS_LEVEL, net_next_packet_id());
            if (n > 0U)
            {
                net_uart_write(s_tx_buf, n);
            }
            break;

        case NET_ST_MQTT_SUB_CMD:
            n = mqtt_build_subscribe(s_tx_buf, NET_TX_BUF_SIZE, MQTT_TOPIC_CMD,
                                     (uint8_t)MQTT_QOS_LEVEL, net_next_packet_id());
            if (n > 0U)
            {
                net_uart_write(s_tx_buf, n);
            }
            break;

        case NET_ST_ONLINE:
            s_step_fail      = 0;
            s_backoff_exp    = 1U;
            s_puback_fail    = 0;
            s_mqtt_connected = 1;
#if (MQTT_USE_APP_ACK == 1)
            s_ack_fail       = 0;
#endif
            s_ping_pending   = 0;
            s_ping_tick      = HAL_GetTick();
            printf("[NET] mqtt online %s:%u dev=%s\r\n", MQTT_HOST, (unsigned)MQTT_PORT, DEV_ID);
            break;

        default:
            break;
    }
}

/** @brief MQTT 三步（CONNECT / SUBSCRIBE ×2）的推进 */
static void net_mqtt_step(uint32_t now)
{
    uint32_t timeout = NET_MQTT_CONNACK_TIMEOUT_MS;

    if ((s_state == NET_ST_MQTT_SUB_ACK) || (s_state == NET_ST_MQTT_SUB_CMD))
    {
        timeout = NET_MQTT_SUBACK_TIMEOUT_MS;
    }

    if (s_mqtt_evt != MQTT_EV_NONE)
    {
        uint8_t ev = s_mqtt_evt;

        s_mqtt_evt = MQTT_EV_NONE;

        if ((s_state == NET_ST_MQTT_CONNECT) && (ev == MQTT_EV_CONNACK))
        {
            if (s_mqtt_connack_rc == 0U)
            {
                s_step_fail = 0;
                net_enter_state(NET_ST_MQTT_SUB_ACK);
            }
            else
            {
                /* 4 = 用户名/密码错误，5 = 未授权，1/2/3 = 协议/版本/客户端 ID 问题 */
                printf("[NET] CONNACK rc=%u (4=bad user/pass, 5=unauthorized)\r\n",
                       (unsigned)s_mqtt_connack_rc);
                net_step_failed();
            }
            return;
        }

        if (ev == MQTT_EV_SUBACK)
        {
            if (s_mqtt.suback_rc != 0x80U)
            {
                s_step_fail = 0;
                net_enter_state((s_state == NET_ST_MQTT_SUB_ACK) ? NET_ST_MQTT_SUB_CMD : NET_ST_ONLINE);
            }
            else
            {
                printf("[NET] SUBACK rejected\r\n");    /* 主题被 broker 拒绝 */
                net_step_failed();
            }
            return;
        }
    }

    if ((uint32_t)(now - s_step_tick) >= timeout)
    {
        net_step_failed();
    }
}

/** @brief 在线时空闲处理：保活 / 超时判定 / 推进待发数据 */
static void net_mqtt_idle(uint32_t now)
{
    /* ---- 1. MQTT 保活 ---- */
    if (s_ping_pending)
    {
        if ((uint32_t)(now - s_ping_tick) >= MQTT_PING_TIMEOUT_MS)
        {
            net_conn_lost("ping timeout, reconnect");
            return;
        }
    }
    else if ((uint32_t)(now - s_ping_tick) >= MQTT_PING_PERIOD_MS)
    {
        uint16_t n = mqtt_build_pingreq(s_tx_buf, NET_TX_BUF_SIZE);

        if (n > 0U)
        {
            net_uart_write(s_tx_buf, n);
        }

        s_ping_tick    = now;
        s_ping_pending = 1;
    }

    /* ---- 2. QoS1 PUBACK 超时 ---- */
    if (s_pub_pending && ((uint32_t)(now - s_pub_tick) >= MQTT_PUBACK_TIMEOUT_MS))
    {
        s_pub_pending = 0;
        s_puback_fail++;

        if (s_puback_fail >= MQTT_PUBACK_FAIL_LIMIT)
        {
            net_conn_lost("puback timeout, reconnect");
            return;
        }
    }

#if (MQTT_USE_APP_ACK == 1)
    /* ---- 3. 后台应用 ACK 超时 ---- */
    if (s_ack_waiting && ((uint32_t)(now - s_ack_tick) >= NET_ACK_TIMEOUT_MS))
    {
        s_ack_waiting = 0;
        s_ack_fail++;

        if (s_ack_fail >= NET_ACK_FAIL_LIMIT)
        {
            net_conn_lost("app ack timeout, reconnect");
            return;
        }
    }
#endif

    /* ---- 4. 有待发数据就发 ---- */
    if (s_tx_pending)
    {
        net_do_publish();
    }
}

/** @brief AT 阶段的 URC 处理（透传阶段的异常靠保活超时兜底） */
static void net_check_urc(void)
{
    if (s_resp_len == 0U)
    {
        return;
    }

    if (resp_has("WIFI DISCONNECT"))
    {
        if (s_state != NET_ST_CWJAP)            /* 已经在重连 WiFi 就不用再重置一次 */
        {
            printf("[NET] wifi disconnected\r\n");
            net_enter_state(NET_ST_CWJAP);
        }
    }
}

static void net_sm_run(void)
{
    uint32_t now     = HAL_GetTick();
    uint32_t timeout = NET_AT_TIMEOUT_MS;
    uint8_t  r       = 0;

    net_check_urc();

    switch (s_state)
    {
        case NET_ST_BACKOFF:
            if ((uint32_t)(now - s_step_tick) >= s_backoff_ms)
            {
                net_enter_state(NET_ST_RESET);
            }
            return;

        case NET_ST_RESET:
            if (s_reset_phase == 0U)
            {
                if ((uint32_t)(now - s_step_tick) >= NET_ESP_RESET_LOW_MS)
                {
                    HAL_GPIO_WritePin(ESP_RST_PORT, ESP_RST_PIN, GPIO_PIN_SET);
                    s_reset_phase = 1U;
                    s_step_tick   = now;        /* 从这里开始等模组启动 */
                }
            }
            else if ((uint32_t)(now - s_step_tick) >= NET_ESP_RESET_WAIT_MS)
            {
                net_enter_state(NET_ST_AT);
            }
            return;

        case NET_ST_ONLINE:
            net_mqtt_idle(now);
            return;

        case NET_ST_MQTT_CONNECT:
        case NET_ST_MQTT_SUB_ACK:
        case NET_ST_MQTT_SUB_CMD:
            net_mqtt_step(now);
            return;

        default:
            break;                          /* AT 文本类步骤，走下面统一处理 */
    }

    switch (s_state)
    {
        case NET_ST_AT:
            r = net_check_resp("OK", 0, 0);
            break;

        case NET_ST_CWMODE:
            r = net_check_resp("OK", 0, "ERROR");
            break;

        case NET_ST_CWJAP:
            r = net_check_resp("WIFI GOT IP", "OK", "FAIL");
            timeout = NET_JOIN_TIMEOUT_MS;
            break;

        case NET_ST_CIPMUX:
            r = net_check_resp("OK", 0, "ERROR");
            break;

        case NET_ST_CIPMODE:
            r = net_check_resp("OK", 0, "ERROR");
            break;

        case NET_ST_CIPSTART:
            r = net_check_resp("CONNECT", "ALREADY CONNECTED", "ERROR");
            timeout = NET_CIPSTART_TIMEOUT_MS;
            break;

        case NET_ST_CIPSEND:
            /* 透传模式下发 AT+CIPSEND，模块回 '>' 提示符（不带换行）*/
            r = net_check_resp(">", 0, "ERROR");
            timeout = NET_CIPSEND_TIMEOUT_MS;
            break;

        default:
            return;
    }

    if (r == 1U)
    {
        s_step_fail = 0;
        net_enter_state((net_state_t)(s_state + 1));
    }
    else if ((r == 2U) || ((uint32_t)(now - s_step_tick) >= timeout))
    {
        net_step_failed();
    }
}

/* ================================================================================================
 * 9. 对外接口（签名与《阶段一计划书》§5 完全一致，阶段二/三直接复用）
 * ================================================================================================ */

/**
 * @brief       初始化通信层（USART2 + EN/RST + 启动连接流程）
 * @note        非阻塞：本函数只做硬件初始化并把状态机置到"硬复位"，
 *              后续 WiFi/MQTT 连接由 net_poll() 一步步推进。
 * @param       无
 * @retval      无
 */
void net_init(void)
{
    s_mqtt_connected = 0;
    s_rx_head        = 0;
    s_rx_tail        = 0;
    s_resp_len       = 0;
    s_resp[0]        = '\0';
    s_at_line_len    = 0;
    s_mqtt_evt       = MQTT_EV_NONE;
    s_mqtt_connack_rc = 0;
    s_packet_id      = 0;
    s_tx_pending     = 0;
    s_tx_data_valid  = 0;
    s_sent_valid     = 0;
    s_pub_pending    = 0;
    s_puback_fail    = 0;
    s_puback_count   = 0;
    s_ack_count      = 0;
#if (MQTT_USE_APP_ACK == 1)
    s_ack_waiting    = 0;
    s_ack_fail       = 0;
    s_ack_got_valid  = 0;
#endif
    s_last_ack_valid = 0;
    s_ping_pending   = 0;
    s_ping_tick      = 0;
    s_step_fail      = 0;
    s_reset_phase    = 0;
    s_backoff_exp    = 1;
    s_backoff_ms     = 0;
    s_oversize_lines = 0;
    s_rx_overflow    = 0;
    s_tx_dropped     = 0;
    s_rx_total       = 0;

    mqtt_parser_reset(&s_mqtt);

    net_esp_pins_init();
    net_uart_init();

    net_enter_state(NET_ST_RESET);
}

/**
 * @brief       通信轮询（主循环每 10 ms 调用一次）
 * @note        单次耗时远小于 50 ms：只做"环形缓冲搬运（限量）+ 状态机推进一步"。
 * @param       无
 * @retval      无
 */
void net_poll(void)
{
    uint16_t n = 0;
    uint8_t  b;

    while ((n < NET_DRAIN_MAX_PER_POLL) && net_ring_pop(&b))
    {
        n++;
        s_rx_total++;                   /* 诊断用：从 ESP 收到的总字节数 */

        if ((s_state >= NET_ST_MQTT_CONNECT) && (s_state <= NET_ST_ONLINE))
        {
            uint8_t ev = mqtt_parser_feed(&s_mqtt, b);

            if (ev != MQTT_EV_NONE)
            {
                net_mqtt_event(ev);
            }
        }
        else
        {
            net_at_parser_feed(b);      /* AT 阶段（含透传前的所有 AT 交互）*/
        }
    }

    net_sm_run();
}

/**
 * @brief       MQTT 是否已连上 broker
 * @param       无
 * @retval      1 = 在线（CONNACK 成功且订阅完成），0 = 离线
 */
uint8_t net_is_online(void)
{
    return s_mqtt_connected;
}

/**
 * @brief       发送一条数据（组 JSON 行 → QoS1 PUBLISH 到 telemetry 主题）
 * @param       d: 数据快照
 * @retval      1 = 已受理；0 = 未受理（未连接 / 上一条还没确认完）
 * @note        本阶段不做落盘：返回 0 即表示这条没发出去（阶段三改成"先落盘再发送"）。
 * @param       d: 数据快照
 */
uint8_t net_send_data(const env_data_t *d)
{
    int n;

    if (d == 0)
    {
        return 0;
    }

    if (!s_mqtt_connected || (s_state != NET_ST_ONLINE))
    {
        s_tx_dropped++;
        return 0;
    }

    if (s_tx_pending || s_pub_pending)
    {
        s_tx_dropped++;
        return 0;                       /* 上一条还在发/还没确认，本阶段不排队 */
    }

    n = snprintf(s_tx_line, sizeof(s_tx_line),
                 "{\"type\":\"data\",\"dev\":\"%s\",\"seq\":%lu,\"ts\":%lu,"
                 "\"temp\":%d,\"humi\":%u,\"smoke\":%u,\"lux\":%u,\"alarm\":\"%s\"}",
                 DEV_ID,
                 (unsigned long)d->seq,
                 (unsigned long)d->ts,
                 (int)d->temp_x10,
                 (unsigned)d->humi_x10,
                 (unsigned)d->smoke_rel,
                 (unsigned)d->lux,
                 alarm_state_str((uint8_t)d->alarm));

    if ((n <= 0) || ((uint16_t)n >= sizeof(s_tx_line)))
    {
        s_tx_dropped++;
        return 0;                       /* 组包失败/超长：宁可不发也不发半条 */
    }

    s_tx_data       = *d;
    s_tx_data_valid = 1;
    s_tx_pending    = 1;

    return 1;
}

/**
 * @brief       累计的"上传成功"次数
 * @param       无
 * @retval      计数（LED4 靠它的变化来闪）
 * @note        MQTT_USE_APP_ACK=1 时统计后台应用在 ack 主题的回执；
 *              =0 时统计 broker 的 QoS1 PUBACK。
 */
uint32_t net_ack_count(void)
{
    return s_ack_count;
}

/**
 * @brief       取"最后一次被确认的数据快照"
 * @param       out: 输出快照
 * @retval      1 = 有效；0 = 还没有任何确认
 */
uint8_t net_get_last_ack(env_data_t *out)
{
    if ((out == 0) || (!s_last_ack_valid))
    {
        return 0;
    }

    *out = s_last_ack_data;

    return 1;
}

/**
 ****************************************************************************************************
 * @file        app_config.h
 * @brief       环境监测系统 —— 引脚 / 阈值 / 网络参数的【唯一配置入口】
 * @note        1. 引脚分配依据《项目计划书v2》§3.2；冻结区(§2.4)内的引脚一律不得使用。
 *              2. 除本文件外，任何 .c / .h 都不得出现硬编码的引脚号、IP、端口。
 *              3. 本文件只放宏定义，不得出现任何函数实现。
 ****************************************************************************************************
 */

#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "./SYSTEM/sys/sys.h"

/* ================================================================================================
 * 1. 指示灯    《项目计划书v2》§2.3
 * ================================================================================================ */
#define LED0_PORT               GPIOB               /* 板载 红 DS0 —— 程序运行心跳 1 Hz      */
#define LED0_PIN                GPIO_PIN_5          /* 低电平点亮                            */

#define LED1_PORT               GPIOE               /* 板载 绿 DS1 —— 烟雾报警              */
#define LED1_PIN                GPIO_PIN_5          /* 低电平点亮                            */

#define LED2_PORT               GPIOF               /* 外接        —— 温湿度报警            */
#define LED2_PIN                GPIO_PIN_0          /* 低电平点亮                            */

#define LED3_PORT               GPIOA               /* 外接 唯一预留灯 —— 光线强度调光      */
#define LED3_PIN                GPIO_PIN_6          /* TIM3_CH1，必须走 PWM（PF/PG 无定时器）*/

#define LED4_PORT               GPIOF               /* 外接        —— 上报成功指示          */
#define LED4_PIN                GPIO_PIN_1          /* 低电平点亮                            */

/* ================================================================================================
 * 2. 蜂鸣器    PB8（板载，同时对应 TIM4_CH3）
 * ================================================================================================ */
#define BEEP_PORT               GPIOB
#define BEEP_PIN                GPIO_PIN_8          /* 高电平响                              */

/* ================================================================================================
 * 3. ADC3 通道（一条 DMA2_CH5 扫描序列，顺序固定，勿改）
 *    依据《项目计划书v2》§3.3：ADC3_IN4~IN9 = PF6~PF11
 *    ⚠️ 2026-09-22 变更：摇杆已移除，模拟量只剩 烟雾 + 光敏 两路。
 *       原「摇杆X(PC0) / 摇杆Y(PC1)」两个 ADC 通道已删除，这两个脚改作「左 / 右」方向按键（见 §5）；
 *       摇杆 SW(PC2) 一并释放。ADC3 规则序列因此从 4 通道缩到 2 通道。
 * ================================================================================================ */
#define SMOKE_AO_CH             ADC_CHANNEL_4       /* PF6  烟雾 AO                         */
#define LSENS_AO_CH             ADC_CHANNEL_6       /* PF8  板载光敏 AO（固定不可改）       */

#define SMOKE_AO_PORT           GPIOF
#define SMOKE_AO_PIN            GPIO_PIN_6
#define LSENS_AO_PORT           GPIOF
#define LSENS_AO_PIN            GPIO_PIN_8

/* 烟雾 DO（数字量）备用硬报警输入，本阶段未使用，仅登记引脚占用 */
#define SMOKE_DO_PORT           GPIOE
#define SMOKE_DO_PIN            GPIO_PIN_6

/* ================================================================================================
 * 4. DHT11 温湿度（板载 U4 单总线口）
 * ================================================================================================ */
#define DHT11_PORT              GPIOG
#define DHT11_PIN               GPIO_PIN_11

/* ================================================================================================
 * 5. 按键（5 键：上 / 下 / 左 / 右 / 确认）
 *
 *    电气接法统一为「一端接 IO、另一端接 GND」→ **低电平有效 + MCU 内部上拉**：
 *      上   = 板载 KEY0 丝印  → PE4
 *      下   = 板载 KEY1 丝印  → PE3
 *      左   = 新接轻触键，插在原来摇杆 X 的位置 → PC0
 *      右   = 新接轻触键，插在原来摇杆 Y 的位置 → PC1
 *      确认 = 板载 WK_UP 丝印 → PA0
 *
 *    ⚠️ 两个容易搞混的点：
 *      1) 这里的「上」是**方向**，不是板载丝印上的 WK_UP。确认键的宏特意叫 KEY_OK_*（物理上是 PA0）。
 *      2) PA0(WK_UP 丝印) 在板子上是**高电平有效**（另一端接 3.3V），这是板子原有接法，不要改；
 *         key.c 的映射表里已按 active_level 区分，其余 4 个键都是低电平有效。
 * ================================================================================================ */
#define KEY_UP_PORT             GPIOE               /* 上   —— 板载 KEY0 丝印，低电平有效 */
#define KEY_UP_PIN              GPIO_PIN_4
#define KEY_DOWN_PORT           GPIOE               /* 下   —— 板载 KEY1 丝印，低电平有效 */
#define KEY_DOWN_PIN            GPIO_PIN_3
#define KEY_LEFT_PORT           GPIOC               /* 左   —— 新接，原摇杆 X 位置，低电平有效 */
#define KEY_LEFT_PIN            GPIO_PIN_0
#define KEY_RIGHT_PORT          GPIOC               /* 右   —— 新接，原摇杆 Y 位置，低电平有效 */
#define KEY_RIGHT_PIN           GPIO_PIN_1
#define KEY_OK_PORT             GPIOA               /* 确认 —— 板载 WK_UP 丝印，高电平有效 */
#define KEY_OK_PIN              GPIO_PIN_0

/* ================================================================================================
 * 6. ESP8266（ESP-01S，USART2）
 *    注意：接模组前必须先拔掉 P5 跳线帽，否则 USART2 与 RS485 芯片冲突。
 * ================================================================================================ */
#define ESP_UART                USART2
#define ESP_UART_IRQn           USART2_IRQn

#define ESP_TX_PORT             GPIOA               /* PA2 = USART2_TX */
#define ESP_TX_PIN              GPIO_PIN_2
#define ESP_RX_PORT             GPIOA               /* PA3 = USART2_RX */
#define ESP_RX_PIN              GPIO_PIN_3

#define ESP_EN_PORT             GPIOC               /* CH_PD，高电平使能 */
#define ESP_EN_PIN              GPIO_PIN_13
#define ESP_RST_PORT            GPIOF               /* 硬复位，低电平有效 */
#define ESP_RST_PIN             GPIO_PIN_12

/* ================================================================================================
 * 7. 网络与 MQTT 配置  ⚠️ 本段含凭据，请勿提交到公共仓库
 *
 *    【架构变更 · 2026-09-22 由用户拍板】设备侧【直连 MQTT broker】。
 *      原 v2.3 方案（设备只做 TCP 透传 + 电脑上的 Python 网关做 MQTT 桥接）作废，
 *      因此删除了 NET_PC_IP / NET_PC_PORT。ESP8266 只当"透传管道"（AT+CIPMODE=1），
 *      MQTT 3.1.1 报文的组包/解析由设备自己的 Drivers/BSP/MQTT 完成。
 *
 *    【没变的】ESP8266 的引脚一律不动，仍与《项目计划书v2》§3.2 一致：
 *      USART2 = PA2(TX)/PA3(RX)（记得拔 P5 跳线帽）、CH_PD = PC13、RST = PF12。
 *      —— 之所以不用 USART1：精英板 V2 的 PA9/PA10 默认跳线接 CH340（P3），
 *         拿它接 ESP 就必须拔 P3，等于丢掉 printf 调试串口，本项目大量验收都靠它。
 *
 *    TODO(烧录前必改)：WIFI_SSID / WIFI_PWD / MQTT_HOST / MQTT_USERNAME / MQTT_PASSWORD
 * ================================================================================================ */
#define DEV_ID                  "dev01"             /* 设备编号，同时用作 MQTT Client ID 与主题前缀 */

/* ⚠️⚠️ 注意：你的手机热点 SSID 末尾【真的带一个空格】，所以这里必须写成 "Hao " ——
 *      "Hao" 后面紧跟一个空格，**编辑时千万不要把那行尾空格删掉**（很多编辑器会"自动清理行尾空格"，
 *      一清理就会又连不上）。
 *
 *      依据：2026-09-27 用 AT+CWLAP 扫出来的原始结果 ——
 *          +CWLAP:(3,"Hao ",-19,"8e:3c:8c:e8:27:b8",11,-21,0)
 *                      ↑ 引号里 Hao 后面还有一个空格
 *      SSID 是逐字符精确匹配的，差一个空格就会一直卡在 step=CWJAP failed。
 *
 *      （不想被这个空格坑的话，也可以去手机热点设置里把名字改成不带空格的 "Hao"，
 *        然后把这里改回 "Hao"；代价是电脑要重连新名字、IP 可能变、MQTT_HOST 得跟着改。）
 */
#define WIFI_SSID               "Hao "              /* 末尾有空格，是故意的，别删 */
#define WIFI_PWD                "lyh123.."


#define MQTT_HOST               "172.29.32.221"
#define MQTT_PORT               1883U               /* broker 端口（明文 1883 / TLS 8883）*/

#define MQTT_USERNAME           "BABA"              /* EMQX 账号（与 WiFi 名无关，只是碰巧同名）*/
#define MQTT_PASSWORD           "lyh123.."          /* EMQX 密码                        */

/* 主题（沿用《项目计划书v2》§7.1 的命名，全部拼上 DEV_ID，换平台只改这里） */
#define MQTT_TOPIC_TELEMETRY    "env/" DEV_ID "/telemetry"      /* 上行：传感器数据 */
#define MQTT_TOPIC_ACK          "env/" DEV_ID "/ack"            /* 下行：数据确认   */
#define MQTT_TOPIC_CMD          "env/" DEV_ID "/cmd"            /* 下行：控制命令（阶段三用）*/

/* MQTT 行为参数 */
#define MQTT_KEEPALIVE_S        60U                 /* CONNECT 里的保活时间（秒）        */
#define MQTT_QOS_LEVEL          1U                  /* 上行 QoS：1 = 有 PUBACK 确认      */
#define MQTT_PING_PERIOD_MS     30000U              /* 每 30 s 发一次 PINGREQ            */
#define MQTT_PING_TIMEOUT_MS    15000U              /* 发 PINGREQ 后 15 s 无响应 → 判死  */
#define MQTT_PUBACK_TIMEOUT_MS  5000U               /* QoS1 PUBLISH 等 PUBACK 的超时     */
#define MQTT_PUBACK_FAIL_LIMIT  3U                  /* 连续 N 次拿不到 PUBACK → 重连     */

/* "上传成功"（= LED4 闪 / net_ack_count() 递增）的判定依据：
 *   1 = 等后台应用在 MQTT_TOPIC_ACK 回 {"type":"ack","seq":N}（与计划书语义一致）
 *   0 = 直接用 broker 的 QoS1 PUBACK（不需要任何后台程序，但只代表 broker 已收，不代表入库） */
#define MQTT_USE_APP_ACK        0

/* AT / 重连 参数 */
#define NET_AT_TIMEOUT_MS       2000U               /* 单条 AT 命令等待响应上限（≤2 s）  */
#define NET_JOIN_TIMEOUT_MS     20000U              /* AT+CWJAP 连 WiFi 的超时           */
#define NET_ACK_TIMEOUT_MS      3000U               /* 单条数据等后台 ACK 的超时          */
#define NET_ACK_FAIL_LIMIT      3U                  /* 连续 N 次 ACK 超时 → 重连         */
#define NET_BACKOFF_MIN_S       2U                  /* 指数退避起始                      */
#define NET_BACKOFF_MAX_S       60U                 /* 指数退避上限                      */
#define NET_LINE_MAX            256U                /* 单行 JSON 上限，超出即丢弃        */
#define NET_RX_RING_SIZE        1024U               /* USART2 接收环形缓冲               */

/* ================================================================================================
 * 8. 阈值默认值（24C02 无有效记录时使用；《项目计划书v2》§6.2）
 * ================================================================================================ */
#define TH_SMOKE_HI             700U                /* 烟雾相对上限（相对基线，标定后修改）*/
#define TH_TEMP_HI_X10          350                 /* 35.0 ℃                              */
#define TH_HUMI_LO_X10          300U                /* 30.0 %RH（下限报警）                */

#define HYST_SMOKE              70U                 /* 烟雾迟滞（约 10% 量程）             */
#define HYST_TEMP_X10           10U                 /* 1.0 ℃                               */
#define HYST_HUMI_X10           30U                 /* 3.0 %RH                             */

#define CONFIRM_CNT             3U                  /* 连续 N 次成立才翻转状态             */
#define SMOKE_WARMUP_MS         60000UL             /* 烟雾模块预热窗口：期内屏蔽烟雾报警  */

#endif /* __APP_CONFIG_H */

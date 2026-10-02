/**
 ****************************************************************************************************
 * @file        mqtt.h
 * @brief       MQTT 3.1.1 报文编解码（纯协议层，不碰串口 / 不碰 GPIO）
 * @note        【为什么自己封包】设备侧直连 broker 时有两个选择：
 *              ① 依赖 AT 固件的 MQTT 命令（AT+MQTTUSERCFG/AT+MQTTPUB…）—— 固件版本差异大、
 *                 行为不可控，这正是《项目计划书v2》v2.2 的 R2 风险；
 *              ② 自己按 MQTT 3.1.1 规范组报文，用最基础的 AT 只用它的"透传管道"。
 *              本文件走 ②：只用 AT+CIPMODE=1 透传，MQTT 协议在 MCU 里实现，行为完全可控。
 *
 *              本层只做编解码，不含任何状态机 / 重发 / 保活逻辑（那些在 net.c）。
 ****************************************************************************************************
 */

#ifndef __MQTT_H
#define __MQTT_H

#include "./SYSTEM/sys/sys.h"

/* ============================ 报文类型（高 4 位） ============================ */
#define MQTT_PKT_CONNECT        0x10U
#define MQTT_PKT_CONNACK        0x20U
#define MQTT_PKT_PUBLISH        0x30U
#define MQTT_PKT_PUBACK         0x40U
#define MQTT_PKT_SUBSCRIBE      0x80U
#define MQTT_PKT_SUBACK         0x90U
#define MQTT_PKT_PINGREQ        0xC0U
#define MQTT_PKT_PINGRESP       0xD0U
#define MQTT_PKT_DISCONNECT     0xE0U

#define MQTT_QOS0               0U
#define MQTT_QOS1               1U

/* ============================ 解析器容量 ============================ */
#define MQTT_TOPIC_MAX          64U     /* 主题最长（含结尾 '\0' 的余量） */
#define MQTT_PAYLOAD_MAX        192U    /* payload 最长（含结尾 '\0' 的余量）*/
#define MQTT_BODY_MAX           512U    /* 单条报文"剩余部分"缓冲上限，超出即丢弃该条 */

/* ============================ 解析出的事件 ============================ */
typedef enum
{
    MQTT_EV_NONE = 0,
    MQTT_EV_CONNACK,        /* 连接应答：connack_rc == 0 才算成功 */
    MQTT_EV_SUBACK,         /* 订阅应答：suback_rc */
    MQTT_EV_PUBLISH,        /* 收到的下行消息：topic + payload（QoS1 时带 packet_id） */
    MQTT_EV_PUBACK,         /* 我们发出的 QoS1 PUBLISH 被 broker 确认 */
    MQTT_EV_PINGRESP        /* 保活应答 */
} mqtt_event_t;

typedef struct
{
    /* ---- 内部解析状态 ---- */
    uint8_t  state;
    uint8_t  type;                  /* 报文类型（高 4 位已还原到低 4 位形式） */
    uint8_t  flags;
    uint32_t rem_len;               /* 剩余长度 */
    uint32_t shift;
    uint8_t  len_bytes;
    uint16_t pos;

    /* ---- 解析结果（事件有效时可读） ---- */
    uint8_t  connack_rc;            /* 0 = 接受；4 = 用户名密码错；5 = 未授权 */
    uint8_t  session_present;
    uint8_t  suback_rc;             /* 0/1/2 = 授予的 QoS；0x80 = 失败 */
    uint8_t  publish_qos;           /* 收到的 PUBLISH 的 QoS：QoS1 时必须回 PUBACK */
    uint16_t packet_id;
    char     topic[MQTT_TOPIC_MAX];
    char     payload[MQTT_PAYLOAD_MAX];

    uint8_t  body[MQTT_BODY_MAX];
} mqtt_parser_t;

/* ============================ 编包 ============================ */
/* 返回值：报文总字节数（含固定头）；0 = 失败（缓冲不够 / 参数非法） */
uint16_t mqtt_build_connect(uint8_t *buf, uint16_t cap,
                            const char *client_id,
                            const char *username,
                            const char *password,
                            uint16_t keepalive_s);
uint16_t mqtt_build_publish(uint8_t *buf, uint16_t cap,
                            const char *topic, const char *payload,
                            uint8_t qos, uint16_t packet_id);
uint16_t mqtt_build_subscribe(uint8_t *buf, uint16_t cap,
                              const char *topic, uint8_t qos, uint16_t packet_id);
uint16_t mqtt_build_puback(uint8_t *buf, uint16_t cap, uint16_t packet_id);
uint16_t mqtt_build_pingreq(uint8_t *buf, uint16_t cap);
uint16_t mqtt_build_disconnect(uint8_t *buf, uint16_t cap);

/* ============================ 解包（流式，逐字节喂） ============================ */
void    mqtt_parser_reset(mqtt_parser_t *p);
/* 返回值：MQTT_EV_* —— 非 0 表示本次喂入凑齐了一条完整报文，结果已写入 p */
uint8_t mqtt_parser_feed(mqtt_parser_t *p, uint8_t b);

#endif /* __MQTT_H */

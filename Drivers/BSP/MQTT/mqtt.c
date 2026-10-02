/**
 ****************************************************************************************************
 * @file        mqtt.c
 * @brief       MQTT 3.1.1 报文编解码
 * @note        只覆盖本项目用到的报文：CONNECT / CONNACK / PUBLISH / PUBACK / SUBSCRIBE / SUBACK /
 *              PINGREQ / PINGRESP / DISCONNECT。不做 QoS2、不做遗嘱（LWT）。
 *              规范要点：
 *                - 固定头第 1 字节 = 报文类型(高4位) | 标志(低4位)
 *                - 剩余长度 = 1~4 字节，"低 7 位有效 + 最高位表示还有后续字节"的小端变长编码
 *                - CONNECT 固定头标志必须为 0；SUBSCRIBE 必须为 0x02；PUBLISH 的 QoS 在 bit1~bit2
 ****************************************************************************************************
 */

#include "./BSP/MQTT/mqtt.h"
#include <string.h>

/* 解析器内部状态 */
#define MQTT_PS_TYPE        0U      /* 等固定头第 1 字节 */
#define MQTT_PS_RLEN        1U      /* 收剩余长度       */
#define MQTT_PS_BODY        2U      /* 收报文体         */
#define MQTT_PS_DRAIN       3U      /* 报文超长，丢弃   */

/**
 * @brief       写剩余长度（变长编码）
 * @param       buf: 输出
 * @param       len: 剩余长度
 * @retval      写入的字节数（1~4）
 */
static uint16_t mqtt_enc_remlen(uint8_t *buf, uint32_t len)
{
    uint16_t n = 0;

    do
    {
        uint8_t b = (uint8_t)(len % 128U);

        len /= 128U;

        if (len > 0U)
        {
            b |= 0x80U;         /* 最高位 = 1 表示还有后续字节 */
        }

        buf[n++] = b;
    } while ((len > 0U) && (n < 4U));

    return n;
}

/**
 * @brief       写 16 位大端整数
 * @param       buf: 输出
 * @param       v  : 数值
 * @retval      写入的字节数（固定 2）
 */
static uint16_t mqtt_put_u16(uint8_t *buf, uint16_t v)
{
    buf[0] = (uint8_t)(v >> 8);
    buf[1] = (uint8_t)(v & 0xFFU);

    return 2;
}

/**
 * @brief       组 CONNECT 报文
 * @param       username/password 为空字符串时对应标志位不置位（匿名连接）
 * @retval      报文总长度；0 = 失败
 */
uint16_t mqtt_build_connect(uint8_t *buf, uint16_t cap,
                            const char *client_id,
                            const char *username,
                            const char *password,
                            uint16_t keepalive_s)
{
    uint16_t idlen;
    uint16_t ulen = 0;
    uint16_t plen = 0;
    uint32_t rem;
    uint16_t n = 0;
    uint8_t  flags = 0x02U;                 /* bit1 = Clean Session */

    if ((buf == 0) || (client_id == 0))
    {
        return 0;
    }

    idlen = (uint16_t)strlen(client_id);

    if ((username != 0) && (username[0] != '\0'))
    {
        ulen = (uint16_t)strlen(username);
        flags |= 0x80U;                     /* bit7 = User Name Flag */

        if ((password != 0) && (password[0] != '\0'))
        {
            plen = (uint16_t)strlen(password);
            flags |= 0x40U;                 /* bit6 = Password Flag */
        }
    }

    rem = 10U + 2U + (uint32_t)idlen;
    if (ulen > 0U)
    {
        rem += 2U + (uint32_t)ulen;
    }
    if (plen > 0U)
    {
        rem += 2U + (uint32_t)plen;
    }

    if ((uint32_t)cap < (rem + 5U))
    {
        return 0;
    }

    buf[n++] = MQTT_PKT_CONNECT;            /* 固定头标志位必须为 0 */
    n = (uint16_t)(n + mqtt_enc_remlen(&buf[n], rem));

    /* ---- 可变头（10 字节）---- */
    n = (uint16_t)(n + mqtt_put_u16(&buf[n], 4U));      /* 协议名长度 */
    buf[n++] = 'M';
    buf[n++] = 'Q';
    buf[n++] = 'T';
    buf[n++] = 'T';
    buf[n++] = 4U;                                      /* 协议级别：4 = 3.1.1 */
    buf[n++] = flags;
    n = (uint16_t)(n + mqtt_put_u16(&buf[n], keepalive_s));

    /* ---- payload：客户端 ID → 用户名 → 密码 ---- */
    n = (uint16_t)(n + mqtt_put_u16(&buf[n], idlen));
    memcpy(&buf[n], client_id, idlen);
    n = (uint16_t)(n + idlen);

    if (ulen > 0U)
    {
        n = (uint16_t)(n + mqtt_put_u16(&buf[n], ulen));
        memcpy(&buf[n], username, ulen);
        n = (uint16_t)(n + ulen);
    }

    if (plen > 0U)
    {
        n = (uint16_t)(n + mqtt_put_u16(&buf[n], plen));
        memcpy(&buf[n], password, plen);
        n = (uint16_t)(n + plen);
    }

    return n;
}

/**
 * @brief       组 PUBLISH 报文
 * @retval      报文总长度；0 = 失败
 */
uint16_t mqtt_build_publish(uint8_t *buf, uint16_t cap,
                            const char *topic, const char *payload,
                            uint8_t qos, uint16_t packet_id)
{
    uint16_t tlen;
    uint16_t plen;
    uint32_t rem;
    uint16_t n = 0;

    if ((buf == 0) || (topic == 0) || (payload == 0))
    {
        return 0;
    }

    tlen = (uint16_t)strlen(topic);
    plen = (uint16_t)strlen(payload);

    rem = 2U + (uint32_t)tlen + ((qos > 0U) ? 2U : 0U) + (uint32_t)plen;

    if ((uint32_t)cap < (rem + 5U))
    {
        return 0;
    }

    buf[n++] = (uint8_t)(MQTT_PKT_PUBLISH | (uint8_t)((qos & 0x03U) << 1));
    n = (uint16_t)(n + mqtt_enc_remlen(&buf[n], rem));
    n = (uint16_t)(n + mqtt_put_u16(&buf[n], tlen));
    memcpy(&buf[n], topic, tlen);
    n = (uint16_t)(n + tlen);

    if (qos > 0U)
    {
        n = (uint16_t)(n + mqtt_put_u16(&buf[n], packet_id));
    }

    memcpy(&buf[n], payload, plen);
    n = (uint16_t)(n + plen);

    return n;
}

/**
 * @brief       组 SUBSCRIBE 报文（固定头标志必须为 0x02）
 * @retval      报文总长度；0 = 失败
 */
uint16_t mqtt_build_subscribe(uint8_t *buf, uint16_t cap,
                              const char *topic, uint8_t qos, uint16_t packet_id)
{
    uint16_t tlen;
    uint32_t rem;
    uint16_t n = 0;

    if ((buf == 0) || (topic == 0))
    {
        return 0;
    }

    tlen = (uint16_t)strlen(topic);
    rem  = 2U + 2U + (uint32_t)tlen + 1U;

    if ((uint32_t)cap < (rem + 5U))
    {
        return 0;
    }

    buf[n++] = (uint8_t)(MQTT_PKT_SUBSCRIBE | 0x02U);
    n = (uint16_t)(n + mqtt_enc_remlen(&buf[n], rem));
    n = (uint16_t)(n + mqtt_put_u16(&buf[n], packet_id));
    n = (uint16_t)(n + mqtt_put_u16(&buf[n], tlen));
    memcpy(&buf[n], topic, tlen);
    n = (uint16_t)(n + tlen);
    buf[n++] = (uint8_t)(qos & 0x03U);

    return n;
}

/**
 * @brief       组 PUBACK 报文
 * @retval      报文总长度（固定 4）；0 = 失败
 */
uint16_t mqtt_build_puback(uint8_t *buf, uint16_t cap, uint16_t packet_id)
{
    if ((buf == 0) || (cap < 4U))
    {
        return 0;
    }

    buf[0] = MQTT_PKT_PUBACK;
    buf[1] = 2U;
    buf[2] = (uint8_t)(packet_id >> 8);
    buf[3] = (uint8_t)(packet_id & 0xFFU);

    return 4;
}

/**
 * @brief       组 PINGREQ 报文
 * @retval      报文总长度（固定 2）；0 = 失败
 */
uint16_t mqtt_build_pingreq(uint8_t *buf, uint16_t cap)
{
    if ((buf == 0) || (cap < 2U))
    {
        return 0;
    }

    buf[0] = MQTT_PKT_PINGREQ;
    buf[1] = 0U;

    return 2;
}

/**
 * @brief       组 DISCONNECT 报文
 * @retval      报文总长度（固定 2）；0 = 失败
 */
uint16_t mqtt_build_disconnect(uint8_t *buf, uint16_t cap)
{
    if ((buf == 0) || (cap < 2U))
    {
        return 0;
    }

    buf[0] = MQTT_PKT_DISCONNECT;
    buf[1] = 0U;

    return 2;
}

/* ================================================================================================
 * 解析
 * ================================================================================================ */

/**
 * @brief       复位解析器
 * @param       p: 解析器
 * @retval      无
 */
void mqtt_parser_reset(mqtt_parser_t *p)
{
    if (p == 0)
    {
        return;
    }

    memset(p, 0, sizeof(mqtt_parser_t));
    p->state = MQTT_PS_TYPE;
}

/**
 * @brief       把一条完整报文体翻译成事件
 * @param       p: 解析器
 * @retval      MQTT_EV_*
 */
static uint8_t mqtt_dispatch(mqtt_parser_t *p)
{
    p->connack_rc      = 0;
    p->session_present = 0;
    p->suback_rc       = 0;
    p->publish_qos     = 0;
    p->packet_id       = 0;
    p->topic[0]        = '\0';
    p->payload[0]      = '\0';

    switch (p->type)
    {
        case MQTT_PKT_CONNACK:
            if (p->rem_len >= 2U)
            {
                p->session_present = (uint8_t)(p->body[0] & 0x01U);
                p->connack_rc      = p->body[1];

                return MQTT_EV_CONNACK;
            }
            break;

        case MQTT_PKT_PUBACK:
            if (p->rem_len >= 2U)
            {
                p->packet_id = (uint16_t)(((uint16_t)p->body[0] << 8) | p->body[1]);

                return MQTT_EV_PUBACK;
            }
            break;

        case MQTT_PKT_SUBACK:
            if (p->rem_len >= 3U)
            {
                p->packet_id = (uint16_t)(((uint16_t)p->body[0] << 8) | p->body[1]);
                p->suback_rc = p->body[2];

                return MQTT_EV_SUBACK;
            }
            break;

        case MQTT_PKT_PINGRESP:
            return MQTT_EV_PINGRESP;

        case MQTT_PKT_PUBLISH:
        {
            uint16_t tl;
            uint16_t idx;
            uint16_t i;
            uint16_t n;
            uint8_t  qos;

            if (p->rem_len < 2U)
            {
                break;
            }

            tl = (uint16_t)(((uint16_t)p->body[0] << 8) | p->body[1]);

            if (((uint32_t)tl + 2U) > p->rem_len)
            {
                break;
            }

            for (i = 0; (i < tl) && (i < (MQTT_TOPIC_MAX - 1U)); i++)
            {
                p->topic[i] = (char)p->body[2U + i];
            }
            p->topic[i] = '\0';

            idx = (uint16_t)(2U + tl);
            qos = (uint8_t)((p->flags >> 1) & 0x03U);

            if (qos > 0U)
            {
                if (((uint32_t)idx + 2U) > p->rem_len)
                {
                    break;
                }

                p->packet_id = (uint16_t)(((uint16_t)p->body[idx] << 8) | p->body[idx + 1U]);
                idx = (uint16_t)(idx + 2U);
            }

            n = 0;
            while ((idx < p->rem_len) && (n < (MQTT_PAYLOAD_MAX - 1U)))
            {
                p->payload[n++] = (char)p->body[idx++];
            }
            p->payload[n] = '\0';
            p->publish_qos = qos;

            return MQTT_EV_PUBLISH;
        }

        default:
            break;
    }

    return MQTT_EV_NONE;
}

/**
 * @brief       喂入一个字节
 * @param       p: 解析器
 * @param       b: 字节
 * @retval      MQTT_EV_* —— 非 0 表示凑齐了一条完整报文，结果在 p 里
 * @note        超长报文（剩余长度 > MQTT_BODY_MAX）会被整体丢弃，不影响后续报文同步。
 */
uint8_t mqtt_parser_feed(mqtt_parser_t *p, uint8_t b)
{
    if (p == 0)
    {
        return MQTT_EV_NONE;
    }

    switch (p->state)
    {
        case MQTT_PS_TYPE:
            p->type      = (uint8_t)(b & 0xF0U);
            p->flags     = (uint8_t)(b & 0x0FU);
            p->rem_len   = 0;
            p->shift     = 0;
            p->len_bytes = 0;
            p->pos       = 0;
            p->state     = MQTT_PS_RLEN;
            break;

        case MQTT_PS_RLEN:
            p->rem_len |= ((uint32_t)(b & 0x7FU)) << p->shift;
            p->shift += 7U;
            p->len_bytes++;

            if ((b & 0x80U) == 0U)              /* 剩余长度收完 */
            {
                if (p->rem_len > MQTT_BODY_MAX)
                {
                    p->pos   = 0;
                    p->state = MQTT_PS_DRAIN;   /* 太长：整条丢弃 */
                }
                else if (p->rem_len == 0U)
                {
                    p->state = MQTT_PS_TYPE;    /* PINGRESP / DISCONNECT 这类无报文体 */

                    return mqtt_dispatch(p);
                }
                else
                {
                    p->pos   = 0;
                    p->state = MQTT_PS_BODY;
                }
            }
            else if (p->len_bytes >= 4U)
            {
                p->state = MQTT_PS_TYPE;        /* 长度编码非法：重新同步 */
            }
            break;

        case MQTT_PS_BODY:
            p->body[p->pos++] = b;

            if (p->pos >= p->rem_len)
            {
                p->state = MQTT_PS_TYPE;

                return mqtt_dispatch(p);
            }
            break;

        case MQTT_PS_DRAIN:
        default:
            p->pos++;

            if ((uint32_t)p->pos >= p->rem_len)
            {
                p->state = MQTT_PS_TYPE;
            }
            break;
    }

    return MQTT_EV_NONE;
}

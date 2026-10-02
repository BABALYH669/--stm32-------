/**
 ****************************************************************************************************
 * @file        net.h
 * @brief       ESP8266 + MQTT 3.1.1 通信层  —— 阶段一 T1.11
 *
 * @note        【架构变更 · 2026-09-22 由用户拍板】设备侧【直连 MQTT broker】。
 *              旧的 v2.3 方案（设备只做 TCP 透传 + 电脑上 pc/gateway.py 做 MQTT 桥接）已作废。
 *              现在的链路：
 *                  STM32 ──USART2(AT+CIPMODE=1 透传)──▶ ESP8266 ──TCP──▶ broker ──▶ MQTT Explorer
 *
 *              ⚠️ 接口签名一个都没改：本组函数仍是《阶段一计划书》§5 / 《阶段三计划书》§5 冻结的
 *                 那 6 个。因此阶段二的 UI 取数（net_is_online / net_ack_count / net_get_last_ack）
 *                 和阶段三的"先落盘再发送"（net_send_data 内部改造）都不需要重写。
 *
 *              与旧方案相比，只有【语义】的两点变化（已在实现里注明）：
 *                1. net_is_online()  = "MQTT 已连上 broker（CONNACK 已收到）"，
 *                   而不是原来的"TCP 已连上电脑网关"；
 *                2. ACK 不再来自电脑网关的 TCP 行，而是来自 MQTT 主题 env/{dev}/ack
 *                   里的 {"type":"ack","seq":N}（见 app_config.h 的 MQTT_USE_APP_ACK，
 *                   也可切成为"直接用 QoS1 的 PUBACK"）。
 *
 *              阶段三仍会把 net.c 的内部实现改成"先落盘再发送"（25Q128 补传），签名保持不变。
 ****************************************************************************************************
 */

#ifndef __NET_H
#define __NET_H

#include "./SYSTEM/sys/sys.h"
#include "env_data.h"

void     net_init(void);                        /* 初始化 USART2 + ESP8266 EN/RST，启动 AT 序列 */
void     net_poll(void);                        /* 主循环每 10 ms 调用一次；单次耗时 < 50 ms */
uint8_t  net_is_online(void);                   /* 1 = TCP 已建立 */
uint8_t  net_send_data(const env_data_t *d);     /* 组 JSON 行 → 交给发送流程；1 = 已受理 */
uint32_t net_ack_count(void);                   /* 累计收到的 ACK 条数（LED4 用它判断"刚上传成功"）*/

/* 【新增】下面这个接口是《阶段二计划书》T2.7 明确要求、且《阶段三计划书》§5 冻结表已列入的：
 *   在 net.c 内部，每收到一条 ACK 就把"当时发送出去的那份 env_data_t"拷进快照。
 *   本阶段一并实现，阶段二的"上报回显页"可直接用，无需再改 net.c。 */
uint8_t  net_get_last_ack(env_data_t *out);     /* 1 = 有效，返回最后被网关确认的那条数据快照 */

#endif /* __NET_H */

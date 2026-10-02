#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gateway.py —— MQTT 侧的"数据消费者 + 中文显示器"

═══════════════════════════════════════════════════════════════════════════════
【为什么改成了这样】
  2026-09-22 架构变更：STM32 侧的 ESP8266 直连 broker（设备自己实现 MQTT 3.1.1）。
  所以本程序**不再监听 TCP**（原来那个 TCP 服务端 + MQTT 桥接的版本已作废），
  它现在只是一个普通的 MQTT 客户端：

      设备 ──publish──▶ env/{dev}/telemetry ──▶ 本程序（翻译成中文 + 去重）
      本程序 ─publish─▶ env/{dev}/display   ──▶ MQTTX / Dashboard 里也能看到中文
      本程序 ─publish─▶ env/{dev}/ack       ──▶ 设备（仅当 MQTT_USE_APP_ACK=1 时才用得上）

【职责】
  1. 订阅 env/+/telemetry：按 dev 去重，把英文 JSON **翻译成中文**打印出来
  2. 把中文结果再发布到 env/{dev}/display（由 config.ini 的 publish_cn 开关控制），
     这样 MQTTX / EMQX Dashboard 里订阅 env/dev01/display 也能看到人话
  3. 回 env/{dev}/ack（预留：app_config.h 的 MQTT_USE_APP_ACK 改成 1 时才需要；
              现在它是 0，设备改用 broker 的 QoS1 PUBACK 判"上传成功"）
  4. 订阅 env/+/ack、env/+/status：打印命令回执与在线状态

【为什么中文渲染放在这里，而不是让设备直接发中文】
  中文 UTF-8 一个字占 3 字节，会让报文明显变长；而且会破坏已定的协议
  （阶段三的后端、本程序都是按 temp/humi/smoke/lux 这些英文字段解析的）。
  所以：**设备发英文（机器友好）、电脑端翻译成中文（人友好）** —— 各司其职。
  注意 MQTTX / EMQX Dashboard 是通用工具，它们只会显示报文原文，没有翻译功能。

【启动顺序】
  1) 确保 EMQX 在跑（本机 1883）
  2) pip install paho-mqtt
  3) python gateway.py
  4) MQTTX / Dashboard 连 127.0.0.1:1883，订阅 env/# 看全部，
     或订阅 env/dev01/display 只看中文
═══════════════════════════════════════════════════════════════════════════════
"""

import configparser
import json
import os
import sys

import paho.mqtt.client as mqtt

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- 配置
cf = configparser.ConfigParser()
cf.read(os.path.join(HERE, "config.ini"), encoding="utf-8")

MQTT_HOST = cf.get("mqtt", "host", fallback="127.0.0.1")
MQTT_PORT = cf.getint("mqtt", "port", fallback=1883)
MQTT_USER = cf.get("mqtt", "username", fallback="").strip()
MQTT_PASS = cf.get("mqtt", "password", fallback="").strip()
PREFIX = cf.get("mqtt", "topic_prefix", fallback="env").strip("/")
KEEPALIVE = cf.getint("mqtt", "keepalive", fallback=60)

# 是否把中文渲染结果再发布回 MQTT 主题 {PREFIX}/{dev}/display
PUBLISH_CN = cf.getboolean("mqtt", "publish_cn", fallback=True)

# 每个设备的 last_seq（用于去重）
last_seq = {}

# ---------------------------------------------------------------- 中文显示
# 说明：设备上报的 JSON 必须保持英文字段（机器可读、报文小、协议稳定），
#       "翻译成中文"这一步放在这里做 —— 只有你自己的程序认识这些字段的含义。
#       MQTTX / EMQX Dashboard 是通用工具，它们永远只显示报文原文。
ALARM_CN = {
    "none":  "正常",
    "smoke": "烟雾报警",
    "temp":  "高温报警",
    "humi":  "低湿报警",
    "multi": "多重报警",
}


def lux_desc(lux: int) -> str:
    """把 0~100 的光强翻译成一句人话（0=最暗，100=最亮）。"""
    if lux <= 10:
        return "很暗"
    if lux <= 40:
        return "偏暗"
    if lux <= 70:
        return "适中"
    return "明亮"


def render_telemetry_cn(dev: str, body: dict) -> str:
    """把设备上报的 JSON 渲染成一行中文，方便直接看。"""
    seq   = body.get("seq", "?")
    ts    = int(body.get("ts", 0))
    temp  = int(body.get("temp", 0)) / 10.0      # 设备发的是 ×10 整数，避免浮点
    humi  = int(body.get("humi", 0)) / 10.0
    smoke = body.get("smoke", "?")
    lux   = int(body.get("lux", 0))
    alarm = ALARM_CN.get(body.get("alarm"), body.get("alarm"))

    return (f"[{dev}] 序号={seq}  "
            f"运行={ts // 60}分{ts % 60:02d}秒  "
            f"温度={temp:.1f}℃  湿度={humi:.1f}%RH  "
            f"烟雾={smoke}  光强={lux}/100({lux_desc(lux)})  "
            f"报警={alarm}")


def build_client(client_id: str):
    """兼容 paho-mqtt 2.x 与 1.x。

    - 2.x：优先用 VERSION2 回调（VERSION1 会打 DeprecationWarning）
    - 1.x：没有 CallbackAPIVersion 这个枚举，回退到老写法
    两个版本的 on_connect 参数个数不同，用 on_connect(..., rc, properties=None) 兜住。
    """
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:
        return mqtt.Client(client_id=client_id)


def on_connect(client, userdata, flags, rc, properties=None):
    # paho 2.x（VERSION2）传 5 个参数、rc 是 ReasonCode 对象；1.x 传 4 个、rc 是 int。
    # 直接原样打印最省事：2.x 会显示 rc=Success，1.x 会显示 rc=0。
    print(f"[MQTT] connected to {MQTT_HOST}:{MQTT_PORT} rc={rc}", flush=True)
    client.subscribe(f"{PREFIX}/+/telemetry", qos=1)
    client.subscribe(f"{PREFIX}/+/ack", qos=1)
    client.subscribe(f"{PREFIX}/+/status", qos=1)
    print(f"[MQTT] subscribed: {PREFIX}/+/telemetry , {PREFIX}/+/ack , {PREFIX}/+/status", flush=True)


def on_message(client, userdata, msg):
    parts = msg.topic.split("/")
    if len(parts) != 3:
        return

    _, dev, kind = parts

    try:
        body = json.loads(msg.payload.decode("utf-8", "replace"))
    except Exception:
        print(f"[!] bad json on {msg.topic}: {msg.payload[:80]!r}", flush=True)
        return

    # ---------------- 上行遥测 ----------------
    if kind == "telemetry":
        t = body.get("type")
        if t != "data":
            print(f"[!] unexpected telemetry type={t} dev={dev}", flush=True)
            return

        seq = int(body.get("seq", 0))
        prev = last_seq.get(dev, -1)

        if seq <= prev:
            print(f"[重复] {dev} 序号={seq}（这条之前已处理过）", flush=True)
        else:
            last_seq[dev] = seq

            text = render_telemetry_cn(dev, body)
            print(text, flush=True)

            # 把中文行再发回 broker：这样在 MQTTX / EMQX Dashboard 里订阅
            # {PREFIX}/{dev}/display 也能看到人话。
            # retain=True 让之后才连上的订阅者立刻就能拿到最后一条（当"当前状态"看）。
            if PUBLISH_CN:
                client.publish(f"{PREFIX}/{dev}/display", text, qos=0, retain=True)

        # 回 ACK。
        # 注意：app_config.h 里 MQTT_USE_APP_ACK 现在是 0，设备用 EMQX 的 QoS1 PUBACK
        #       判"上传成功"，所以这条 ACK 设备并不会用到 —— 保留它是为了将来把
        #       MQTT_USE_APP_ACK 改回 1 时无需改代码。
        ack = json.dumps({"type": "ack", "seq": seq}, separators=(",", ":"))
        client.publish(f"{PREFIX}/{dev}/ack", ack, qos=1)
        return

    # ---------------- 设备命令回执（阶段三）----------------
    if kind == "ack":
        if body.get("type") == "cmd_ack":
            print(f"[<] {dev} cmd_ack requestId={body.get('requestId')} result={body.get('result')}",
                  flush=True)
        return

    # ---------------- 在线状态 ----------------
    if kind == "status":
        print(f"[~] {dev} status={msg.payload.decode('utf-8', 'replace')}", flush=True)
        return


def main():
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass

    client = build_client("env-consumer")

    if MQTT_USER:
        client.username_pw_set(MQTT_USER, MQTT_PASS)

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(MQTT_HOST, MQTT_PORT, KEEPALIVE)
    print(f"[MQTT] connecting {MQTT_HOST}:{MQTT_PORT} ...", flush=True)

    client.loop_forever()


if __name__ == "__main__":
    main()

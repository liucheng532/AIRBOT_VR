#!/usr/bin/env python3
import json
import uuid
import threading
import time
import websocket

# =====================================================
#   独立的 WebSocketTwistClient（无需任何外部文件）
# =====================================================
class WebSocketTwistClient:
    def __init__(self, ws_url, accid, rate_hz=30):
        self.ws_url = ws_url
        self.accid = accid
        self.rate_hz = rate_hz

        self.ws = None
        self.ws_ready = False
        self._stop = threading.Event()

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self._lock = threading.Lock()

    def _guid(self):
        return str(uuid.uuid4())

    def _send_twist(self):
        msg = {
            "accid": self.accid,
            "title": "request_twist",
            "timestamp": int(time.time() * 1000),
            "guid": self._guid(),
            "data": {"x": self.x, "y": self.y, "z": self.z},
        }

        if self.ws and self.ws_ready:
            try:
                self.ws.send(json.dumps(msg))
            except:
                pass

    # WebSocket 回调 -------------------------
    def _on_open(self, ws):
        print("[WebSocket] 已连接。")
        self.ws_ready = True

    def _on_close(self, ws, *args):
        print("[WebSocket] 已断开连接。")
        self.ws_ready = False

    def _on_message(self, ws, msg):
        pass

    # 启动 WebSocket 发送线程 -----------------
    def start(self):
        def ws_thread():
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=self._on_open,
                on_message=self._on_message,
                on_close=self._on_close,
            )
            self.ws.run_forever()

        threading.Thread(target=ws_thread, daemon=True).start()

        def send_thread():
            period = 1.0 / self.rate_hz
            while not self._stop.is_set():
                with self._lock:
                    self._send_twist()
                time.sleep(period)

        threading.Thread(target=send_thread, daemon=True).start()

    # 停止 -------------------------
    def stop(self):
        self._stop.set()
        if self.ws:
            self.ws.close()

    # 更新速度 -------------------------
    def update_cmd(self, x, y, z):
        with self._lock:
            self.x = x
            self.y = y
            self.z = z


# =====================================================
#   自动测试 六个方向
# =====================================================
def send(ws, x, y, z, t, label):
    print(f"=== {label}，持续 {t} 秒 ===")
    ws.update_cmd(x, y, z)
    time.sleep(t)
    ws.update_cmd(0, 0, 0)
    time.sleep(0.5)


def main():
    ws = WebSocketTwistClient(
        ws_url="ws://10.192.1.2:5000",
        accid="SF_TRON1A_278",
        rate_hz=30
    )
    ws.start()

    print("""
======================================
   自动机器人坐标轴测试程序（独立版）
======================================

测试顺序，每段 1 秒：
1. x 正方向（预期：向前）
2. x 负方向（预期：向后）
3. y 正方向（预期：向左）
4. y 负方向（预期：向右）
5. z 正方向（预期：逆时针旋转）
6. z 负方向（预期：顺时针旋转）

请观察机器人实际动作是否一致！
""")

    time.sleep(2)  # 等 WebSocket 连接

    speed = 0.3
    turn = 0.6

    # 自动测试六个方向
    # send(ws,  speed, 0,     0,    1.0, "测试 x 正方向")
    # send(ws, -speed, 0,     0,    1.0, "测试 x 负方向")
    # send(ws, 0,      speed, 0,    2.0, "测试 y 正方向")
    # send(ws, 0,     -speed, 0,    2.0, "测试 y 负方向")
    # send(ws, 0,      0,     turn, 2.0, "测试 z 正方向")
    # send(ws, 0,      0,    -turn, 4.0, "测试 z 负方向")

    print("=== 测试完成，归零 ===")
    ws.update_cmd(0, 0, 0)
    time.sleep(0.5)
    ws.stop()


if __name__ == '__main__':
    main()

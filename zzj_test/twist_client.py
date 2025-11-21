# twist_ws_client.py
import json
import uuid
import threading
import time
import websocket


class WebSocketTwistClient:
    def __init__(self, ws_url, accid, rate_hz=50):
        self.ws_url = ws_url
        self.accid = accid
        self.rate_hz = rate_hz

        self.ws = None
        self.ws_ready = False
        self._stop = threading.Event()

        # ---------------------
        # 行走速度（滤波后的结果）
        # ---------------------
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # ---------------------
        # 控制参数
        # ---------------------
        self.alpha = 0.35  # 滤波平滑
        self.deadzone = 0.10  # 死区
        self.max_speed = 0.40  # 平移限幅
        self.max_turn = 0.80  # 旋转限幅
        self.stop_decay = 0.80  # 松开摇杆后的平滑停止

        # 坐标反转（必要时调）
        self.invert_x = 1.0
        self.invert_y = 1.0
        self.invert_z = 1.0

        self._lock = threading.Lock()

    # -------------------------------------------------------------
    # 工具函数
    # -------------------------------------------------------------
    def _filter(self, old, new):
        return self.alpha * new + (1 - self.alpha) * old

    def _deadzone(self, v):
        return 0.0 if abs(v) < self.deadzone else v

    def _limit(self, v, lim):
        return max(min(v, lim), -lim)

    def _smooth_stop(self, v):
        return 0.0 if abs(v) < 1e-3 else v * self.stop_decay

    # -------------------------------------------------------------
    # 处理 VR 摇杆数据 -> 自动发送 Twist
    # -------------------------------------------------------------
    def process_vr_stick(self, data):
        """输入 VR 控制数组，自动生成底盘移动命令"""

        if len(data) < 6:
            return

        # ------------------ 摇杆读取 ------------------
        lx = self._deadzone(data[2])  # 左摇杆 X
        ly = self._deadzone(data[3])  # 左摇杆 Y
        rx = self._deadzone(data[4])  # 右摇杆 X

        # ------------------ 映射 ------------------
        target_x = self.invert_x * (-ly)  # 前后
        target_y = self.invert_y * (lx)  # 左右
        target_z = self.invert_z * (rx)  # 旋转

        # ------------------ 限速 ------------------
        target_x = self._limit(target_x, self.max_speed)
        target_y = self._limit(target_y, self.max_speed)
        target_z = self._limit(target_z, self.max_turn)

        # ------------------ 避免斜方向过快 ------------------
        if abs(target_x) > abs(target_y):
            target_y *= 0.25
        else:
            target_x *= 0.25

        # ------------------ 滤波 / 停止衰减 ------------------
        self.vx = self._filter(self.vx, target_x) if target_x != 0 else self._smooth_stop(self.vx)
        self.vy = self._filter(self.vy, target_y) if target_y != 0 else self._smooth_stop(self.vy)
        self.wz = self._filter(self.wz, target_z) if target_z != 0 else self._smooth_stop(self.wz)

        # ------------------ 自动发送 ------------------
        self.update_cmd(self.vx, self.vy, self.wz)

    # -------------------------------------------------------------
    # WebSocket 部分
    # -------------------------------------------------------------
    def _guid(self):
        return str(uuid.uuid4())

    def _send_twist(self, x, y, z):
        msg = {
            "accid": self.accid,
            "title": "request_twist",
            "timestamp": int(time.time() * 1000),
            "guid": self._guid(),
            "data": {"x": x, "y": y, "z": z},
        }

        if self.ws and self.ws_ready:
            try:
                self.ws.send(json.dumps(msg))
            except:
                pass

    def update_cmd(self, x, y, z):
        with self._lock:
            self._send_twist(x, y, z)

    def _on_open(self, ws):
        print("[WebSocket] Connected.")
        self.ws_ready = True

    def _on_close(self, ws, *args):
        print("[WebSocket] Closed.")
        self.ws_ready = False

    def _on_message(self, ws, msg):
        pass

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

    def stop(self):
        self._stop.set()
        if self.ws:
            self.ws.close()

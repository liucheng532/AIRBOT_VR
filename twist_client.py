import json
import uuid
import threading
import time
import websocket

class WebSocketTwistClient:
    def __init__(self, ws_url, accid, rate_hz=30):
        self.ws_url = ws_url
        self.accid = accid
        self.rate_hz = rate_hz

        self.ws = None
        self.ws_ready = False
        self._stop = threading.Event()

        # 最新速度指令
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # 保护锁
        self._lock = threading.Lock()

        # 参数调整区 ⭐（你可以调这里）
        self.alpha = 0.35  # 靠近目标速度的滤波系数
        self.deadzone = 0.10  # 摇杆死区（双足建议略大）
        self.max_speed = 0.40  # 平移 max
        self.max_turn = 0.60  # 旋转 max
        self.stop_decay = 0.80  # ⭐ 平滑停止，每帧乘以 0.8，约 0.15 s 停

        # 若方向反，改成 -1
        self.invert_x = 1.0
        self.invert_y = -1.0
        self.invert_z = -1.0


    def _guid(self):
        return str(uuid.uuid4())

    def _send_twist(self):
        # if (self.x!=0)|(self.y!=0)|(self.z!=0):
        #     print(self.x,self.y,self.z)
        #     print('--------')
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
        
    # ---- WebSocket 回调函数 ----
    def _on_open(self, ws):
        print("[WebSocket] 连接成功.")
        self.ws_ready = True

    def _on_close(self, ws, *args):
        print("[WebSocket] 连接关闭.")
        self.ws_ready = False

    def _on_message(self, ws, msg):
        pass  # 不需要处理返回消息

    def start(self):
        # WebSocket 线程
        def ws_thread():
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=self._on_open,
                on_message=self._on_message,
                on_close=self._on_close,
            )
            self.ws.run_forever()

        threading.Thread(target=ws_thread, daemon=True).start()

        # 命令发送线程
        def send_thread():
            period = 1.0 / self.rate_hz
            while not self._stop.is_set():
                with self._lock:
                    self._send_twist()
                time.sleep(period)

        threading.Thread(target=send_thread, daemon=True).start()

    def stop(self):
        self._stop.set()
        if self.ws:
            self.ws.close()

    def update_cmd(self, x, y, z):
        with self._lock:
            self.x = x
            self.y = y
            self.z = z

    def update_cmd_from_vr(self, lx,ly,rx):
        """处理 VR 控制器的数据"""
        # if len(data) < 6:
        #     return

        # -------------------------
        # 读取摇杆数据
        # -------------------------
        # lx = self.apply_deadzone(data[3])  # 左摇杆 x
        # ly = self.apply_deadzone(data[2])  # 左摇杆 y
        # rx = self.apply_deadzone(data[4])  # 右摇杆 x
        # if (lx!=0)|(ly!=0)|(rx!=0):
        #     print(lx,ly,rx)
        # -------------------------
        # 速度映射（坐标系转换 + 方向反转）
        # -------------------------
        target_x = self.invert_x * (lx)  # 前后
        target_y = self.invert_y * (ly)  # 左右
        target_z = self.invert_z * (rx)  # 旋转（增强旋转速度）

        # -------------------------
        # 限速
        # -------------------------
        target_x = self.limit(target_x, self.max_speed)
        target_y = self.limit(target_y, self.max_speed)
        target_z = self.limit(target_z, self.max_turn)

        # -------------------------
        # 避免同时 x 和 y 太大
        # 比如 x=0.5, y=0.4, 则把 y 缩小
        # -------------------------
        if abs(target_x) > abs(target_y):
            target_y = target_y * 0.25  # 缩小 y 的速度
        else:
            target_x = target_x * 0.25  # 缩小 x 的速度

        # -------------------------
        # 平滑停止逻辑
        # 摇杆有输入 → 滤波跟随
        # 摇杆回到 0 → smooth_stop()
        # -------------------------
        if target_x != 0.0:
            self.x = self.filter(self.x, target_x)
        else:
            self.x = self.smooth_stop(self.x)

        if target_y != 0.0:
            self.y = self.filter(self.y, target_y)
        else:
            self.y = self.smooth_stop(self.y)

        if target_z != 0.0:
            self.z = self.filter(self.z, target_z)
        else:
            self.z = self.smooth_stop(self.z)

        # 更新发送的速度指令
        self._send_twist()
        return {"x": self.x, "y": self.y, "z": self.z}

    # -----------------------------
    # 工具函数
    # -----------------------------
    def filter(self, old, new):
        """一阶低通滤波"""
        return self.alpha * new + (1 - self.alpha) * old

    def apply_deadzone(self, v):
        """摇杆死区"""
        return 0.0 if abs(v) < self.deadzone else v

    def limit(self, v, limit_val):
        """限速"""
        return max(min(v, limit_val), -limit_val)

    def smooth_stop(self, v):
        """平滑停止（指数衰减）"""
        if abs(v) < 1e-3:
            return 0.0
        return v * self.stop_decay

# imu_manager.py
import json
import uuid
import threading
import time
import websocket


class IMUManager:

    def __init__(self, ws_url, accid=None):
        self.ws_url = ws_url
        self.accid = accid  # optional
        self.ws = None

        # 最新 IMU 数据（线程安全，可被主程序读取）
        self.latest_imu = {
            "euler": None,
            "acc": None,
            "gyro": None,
            "quat": None,
            "timestamp": 0,
        }

        self._lock = threading.Lock()
        self._stop = threading.Event()

    def _generate_guid(self):
        return str(uuid.uuid4())

    def _send_request(self, title, data=None):
        if data is None:
            data = {}
        msg = {
            "accid": self.accid,
            "title": title,
            "timestamp": int(time.time() * 1000),
            "guid": self._generate_guid(),
            "data": data,
        }
        if self.ws:
            self.ws.send(json.dumps(msg))

    # ========== WebSocket callbacks ==========
    def _on_open(self, ws):
        print("[IMU] WebSocket connected")
        # 开启 IMU 推送
        self._send_request("request_enable_imu", {"enable": True})

    def _on_message(self, ws, message):
        root = json.loads(message)
        if root.get("title") == "notify_imu":
            imu = root["data"]
            with self._lock:
                self.latest_imu = {
                    "euler": imu["euler"],
                    "acc": imu["acc"],
                    "gyro": imu["gyro"],
                    "quat": imu["quat"],
                    "timestamp": time.time(),
                }

    def _on_close(self, ws, *args):
        print("[IMU] WebSocket closed")

    # ==========================================

    def start(self):
        """启动独立线程并连接 WebSocket"""

        def run():
            self.ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=self._on_open,
                on_message=self._on_message,
                on_close=self._on_close,
            )
            self.ws.run_forever()

        threading.Thread(target=run, daemon=True).start()
        print("[IMU] Manager started")

    def stop(self):
        self._stop.set()
        if self.ws:
            self.ws.close()

    def get_latest_imu(self):
        """主线程从这里获取最新 IMU（非阻塞）"""
        with self._lock:
            return dict(self.latest_imu)

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import threading
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from twist_client import WebSocketTwistClient  # ← 你封装好的版本


class TwistTester(Node):
    def __init__(self):
        super().__init__("twist_tester")

        # ------------------------------------
        # 初始化 WebSocket twist 客户端
        # ------------------------------------
        self.ws = WebSocketTwistClient(
            ws_url="ws://127.0.0.1:5000",  # 可改为你的服务器
            accid="TEST_VR",
            rate_hz=50,
        )
        self.ws.start()

        # ------------------------------------
        # 订阅 VR 控制器
        # ------------------------------------
        self.create_subscription(
            Float32MultiArray,
            "/vr_controller",
            self.vr_callback,
            10,
        )

        self.get_logger().info("TwistTester started. Waiting for VR...")

        # ------------------------------------
        # 存储实时曲线数据
        # ------------------------------------
        self.history_len = 200
        self.vx_hist = [0] * self.history_len
        self.vy_hist = [0] * self.history_len
        self.wz_hist = [0] * self.history_len

        # 创建线程，运行 matplotlib 动画
        threading.Thread(target=self.plot_thread, daemon=True).start()

    # ==========================================================
    # 接收 VR 数据（摇杆） → 交给 WebSocketTwistClient 计算
    # ==========================================================
    def vr_callback(self, msg):
        data = msg.data
        if len(data) < 6:
            return

        # 将整个 vr_cmd 数组传入你的客户端处理
        self.ws.process_vr_stick(data)
        vx, vy, wz = self.ws.vx, self.ws.vy, self.ws.wz

        # 终端实时打印
        print(f"[VR] vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")

        # 加入历史曲线
        self.vx_hist.append(vx)
        self.vx_hist.pop(0)
        self.vy_hist.append(vy)
        self.vy_hist.pop(0)
        self.wz_hist.append(wz)
        self.wz_hist.pop(0)

    # ==========================================================
    # Matplotlib 实时绘图线程
    # ==========================================================
    def plot_thread(self):
        plt.style.use("ggplot")
        fig, ax = plt.subplots(figsize=(8, 4))

        (line_vx,) = ax.plot([], [], label="vx", linewidth=2)
        (line_vy,) = ax.plot([], [], label="vy", linewidth=2)
        (line_wz,) = ax.plot([], [], label="wz", linewidth=2)

        ax.set_ylim(-1.0, 1.0)
        ax.set_xlim(0, self.history_len)
        ax.legend()
        ax.set_title("Real-time Twist from VR")
        ax.set_xlabel("Frame")
        ax.set_ylabel("Velocity")

        def update(frame):
            line_vx.set_data(range(self.history_len), self.vx_hist)
            line_vy.set_data(range(self.history_len), self.vy_hist)
            line_wz.set_data(range(self.history_len), self.wz_hist)
            return line_vx, line_vy, line_wz

        ani = animation.FuncAnimation(fig, update, interval=50, blit=True)
        plt.show()


def main():
    rclpy.init()
    node = TwistTester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

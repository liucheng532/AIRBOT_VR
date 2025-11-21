#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import numpy as np


class FakeVRPublisher(Node):
    def __init__(self):
        super().__init__('fake_vr_publisher')
        self.pub = self.create_publisher(Float32MultiArray, '/vr_controller', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz 发布频率

        self.start_time = time.time()
        self.data = np.zeros(12, dtype=np.float32)  # 模拟手柄 12 通道
        self.THRESHOLD = 0.8

        self.get_logger().info("Fake VR publisher started — 5s trigger version")

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        msg = Float32MultiArray()
        self.data[:] = 0.0  # 每帧清零

        # === 模拟时序 ===
        # A键 -> 扳机(5s) -> B键 -> X键 -> A键 -> 扳机(5s) -> B键
        # 通道定义：
        # A 键: data[0]
        # B 键: data[1]
        # 扳机（左前扳机）: data[6]
        # X 键: data[10]

        # 0~1s: 按 A 键（开始同步）
        if 0 <= elapsed < 1:
            self.data[0] = 1.0
        # 1~6s: 扳机按下（持续 5 秒）
        elif 1 <= elapsed < 6:
            self.data[6] = 1.0
        # 6~7s: 按 B 键（停止）
        elif 6 <= elapsed < 7:
            self.data[1] = 1.0
        # 7~9s: 按 X 键（复位，持续两秒）
        elif 7 <= elapsed < 9:
            self.data[10] = 1.0
        # 9~10s: 再按 A 键（重新同步）
        elif 9 <= elapsed < 10:
            self.data[0] = 1.0
        # 10~15s: 扳机按下（持续 5 秒）
        elif 10 <= elapsed < 15:
            self.data[6] = 1.0
        # 15~16s: 按 B 键（停止）
        elif 15 <= elapsed < 16:
            self.data[1] = 1.0

        msg.data = self.data.tolist()
        self.pub.publish(msg)

        # 打印当前状态（每 0.5 秒一次）
        if abs(elapsed * 2 - round(elapsed * 2)) < 0.02:
            a, b, trig, x = self.data[0], self.data[1], self.data[6], self.data[10]
            self.get_logger().info(
                f"t={elapsed:5.1f}s | A={a:.1f}, B={b:.1f}, Trig={trig:.1f}, X={x:.1f}"
            )

        if elapsed > 17.0:
            self.get_logger().info("✅ Test sequence finished.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FakeVRPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

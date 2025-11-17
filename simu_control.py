#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import math


class FakeVRPublisher(Node):
    def __init__(self):
        super().__init__("fake_vr_publisher")

        self.pub_ctrl = self.create_publisher(Float32MultiArray, "/vr_controller", 10)
        self.pub_left = self.create_publisher(Float32MultiArray, "/leftInfo", 10)
        self.pub_right = self.create_publisher(Float32MultiArray, "/rightInfo", 10)
        self.pub_head = self.create_publisher(Float32MultiArray, "/headInfo", 10)

        self.get_logger().info("Fake VR Publisher started")

    # -----------------------
    # 生成 VR 控制数组（12维）
    # -----------------------
    def make_ctrl(self, A=0.0, B=0.0, X=0.0, Y=0.0, Trig=0.0, Grip=0.0):
        msg = Float32MultiArray()
        data = [0.0] * 12

        # 映射与 VRArm 完全一致
        # [0] = A
        # [1] = B
        # [6] = 左手前扳机（手臂）
        # [8] = 左手侧扳机（夹爪）
        # [10] = X
        # [11] = Y
        data[0] = float(A)
        data[1] = float(B)
        data[6] = float(Trig)
        data[8] = float(Grip)
        data[10] = float(X)
        data[11] = float(Y)

        msg.data = data
        return msg

    # -----------------------
    # 手部信息（位置 + 四元数）
    # -----------------------
    def make_hand_info(self, moving=False, t=0.0):
        msg = Float32MultiArray()

        if moving:
            # 小范围来回移动一下，方便看机械臂动
            x = 0.05 * math.sin(2 * math.pi * 0.5 * t)
            y = 0.05 * math.cos(2 * math.pi * 0.5 * t)
            z = 0.25
        else:
            x, y, z = 0.0, 0.0, 0.25

        # 固定朝向
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        msg.data = [float(x), float(y), float(z), float(qx), float(qy), float(qz), float(qw)]
        return msg

    # -----------------------
    # 发布一帧所有 topic
    # -----------------------
    def publish_all(self, ctrl_msg, moving=False, t=0.0):
        self.pub_ctrl.publish(ctrl_msg)
        self.pub_left.publish(self.make_hand_info(moving, t))
        self.pub_right.publish(self.make_hand_info(moving, t))
        self.pub_head.publish(self.make_hand_info(False, t))


def main():
    rclpy.init()
    node = FakeVRPublisher()

    def step(ctrl_msg, duration_sec=1.0, moving=False):
        """以 30Hz 频率，持续 duration_sec 时间"""
        t0 = time.time()
        frame = 0
        while time.time() - t0 < duration_sec and rclpy.ok():
            now = time.time()
            node.publish_all(ctrl_msg, moving=moving, t=now - t0)
            frame += 1
            time.sleep(1.0 / 30.0)
        print(f"  -> 发送完 {frame} 帧")

    print("======= Fake VR 测试开始 =======")

    # 1. 按下 A 开启遥操
    print("→ 按下 A（开启遥操）")
    step(node.make_ctrl(A=1.0), duration_sec=0.3, moving=False)

    # 2. 按下 B 开始录制
    print("→ 按下 B 开始录制")
    step(node.make_ctrl(B=1.0), duration_sec=0.3, moving=False)

    # 3. 扳机移动 2 秒（此时应该在录制）
    print("→ 扳机（手部移动 2 秒）")
    step(node.make_ctrl(Trig=1.0), duration_sec=2.0, moving=True)

    # 4. <5s 内按 B，应该被忽略
    print("→ 尝试提前按 B（应该被忽略）")
    step(node.make_ctrl(B=1.0), duration_sec=0.3, moving=False)

    # 5. 再等 3s，让总时间超过 5s
    print("→ 等待 3 秒（不按键）")
    step(node.make_ctrl(), duration_sec=3.0, moving=False)

    # 6. 再按 B，应该真正停止录制
    print("→ 再按 B（应该停止录制）")
    step(node.make_ctrl(B=1.0), duration_sec=0.3, moving=False)

    print("======= Fake VR 测试结束 =======")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

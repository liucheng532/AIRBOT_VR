# fake_vr_pub.py —— 修正版，不使用 create_rate()，可正常循环打印
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FakeVRPublisher(Node):
    def __init__(self, freq=20):
        super().__init__("fake_vr_pub")
        self.pub_ctrl = self.create_publisher(Float32MultiArray, "/vr_controller", 10)
        self.pub_left = self.create_publisher(Float32MultiArray, "/leftInfo", 10)
        self.pub_right = self.create_publisher(Float32MultiArray, "/rightInfo", 10)
        self.freq = freq
        self.time0 = time.time()
        self.vr_cmd = np.zeros(12, dtype=np.float32)

        print("🎮 Fake VR publisher started.")
        print("✅ 正在模拟发布 /vr_controller, /leftInfo, /rightInfo")

    def run(self):
        last_status = None
        while rclpy.ok():
            t = time.time() - self.time0
            msg_ctrl = Float32MultiArray()
            msg_left = Float32MultiArray()
            msg_right = Float32MultiArray()

            # ===== 模拟控制逻辑 =====
            self.vr_cmd[:] = 0.0
            status = "idle"
            if 3.0 <= t < 18.0:
                self.vr_cmd[6] = 1.0
                status = "recording"
            elif 18.0 <= t < 20.0:
                self.vr_cmd[1] = 1.0
                status = "stopping"
            elif t >= 20.0:
                status = "done"

            msg_ctrl.data = self.vr_cmd.tolist()

            # 模拟左手位姿（简单往前移动一点）
            pos = [0.2 + 0.05 * np.sin(t), 0.0, 0.3 + 0.05 * np.cos(t)]
            quat = [0, 0, 0, 1]
            msg_left.data = pos + quat
            msg_right.data = [0, 0, 0, 0, 0, 0, 0]

            # 发布
            self.pub_ctrl.publish(msg_ctrl)
            self.pub_left.publish(msg_left)
            self.pub_right.publish(msg_right)

            # 每秒打印一次状态
            if int(t) != int(t - 1) and status != last_status:
                print(f"[{t:5.2f}s] 状态: {status}")
                print(f"   /vr_controller data = {np.round(self.vr_cmd, 2)}")
                last_status = status

            if status == "done":
                print("✅ 模拟完毕，保持发布空闲信号。Ctrl+C可退出。")

            time.sleep(1.0 / self.freq)


if __name__ == "__main__":
    rclpy.init()
    node = FakeVRPublisher(freq=20)
    try:
        node.run()
    except KeyboardInterrupt:
        print("👋 手动中断，退出 Fake VR Publisher.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class VRRawViewer(Node):
    def __init__(self):
        super().__init__('vr_raw_viewer')

        self.create_subscription(
            Float32MultiArray,
            '/vr_controller',
            self.callback,
            10
        )

        self.get_logger().info("VR 原始数据测试程序已启动")

    def callback(self, msg):
        data = msg.data
        print("=== VR 原始数据 ===")
        for i, v in enumerate(data):
            print(f"data[{i}] = {v:.3f}")
        print("------------------")

def main(args=None):
    rclpy.init(args=args)
    node = VRRawViewer()
    rclpy.spin(node)
    node.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

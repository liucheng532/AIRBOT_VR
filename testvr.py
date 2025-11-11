#!/usr/bin/env python3
"""
VR 控制器映射测试节点（带阈值触发逻辑）
用于验证 Quest3 -> ROS2 的按键映射是否正确。
当数据超过阈值时执行特定操作（模拟真实 teleop 行为）。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class VRMappingTester(Node):
    def __init__(self):
        super().__init__('vr_mapping_tester')

        # 设置触发阈值，与 teleop 中保持一致
        self.THRESHOLD = 0.5

        # 订阅 /vr_controller 话题
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/vr_controller',
            self.callback,
            10
        )

        # 状态变量
        self.startflag = False

        # 按键说明表（与原映射一致）
        self.button_names = {
            0: "右手柄A键 -> 开始控制",
            1: "右手柄B键 -> 停止控制",
            6: "左手柄前扳机 -> LTrig",
            7: "右手柄前扳机 -> RTrig",
            8: "左手柄侧方中指扳机 -> LGrip（夹爪控制）",
            9: "右手柄侧方中指扳机（未使用）",
            10: "左手柄X键 -> PauseL（重置）",
            11: "左手柄Y键 -> EXIT（退出）"
        }

        self.get_logger().info("✅ VR映射测试器已启动，监听 /vr_controller")

    def callback(self, msg):
        data = msg.data
        if len(data) < 12:
            return

        # ====== 模拟 teleop.vr_arm 的触发逻辑 ======
        # 右手柄 A键 -> 开始控制
        if data[0] > self.THRESHOLD:
            if not self.startflag:
                print("\033[93mstart VR control\033[0m")  # 黄色输出
                self.startflag = True

        # 右手柄 B键 -> 停止控制
        if data[1] > self.THRESHOLD:
            if self.startflag:
                print("\033[91mstop Arm control\033[0m")  # 红色输出
                self.startflag = False

        # 左手柄 X键 -> 重置
        if data[10] > self.THRESHOLD:
            print("\033[94mstart reset, please wait...\033[0m")  # 蓝色输出
            self.startflag = False
            self.reset()

        # 左手柄 Y键 -> 退出
        if data[11] > self.THRESHOLD:
            print("\033[95mEXIT triggered\033[0m")

        # 其他键测试输出（方便调试）
        for i in [6, 7, 8, 9]:
            if data[i] > self.THRESHOLD:
                print(f"[{i}] {self.button_names.get(i, f'按钮{i}')} → 按下")
        
    def reset(self):
        """模拟 reset 行为"""
        print("\033[96m[Reset] 位置和状态已重置！\033[0m")


def main(args=None):
    rclpy.init(args=args)
    node = VRMappingTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 已退出 VR 按键测试。")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import logging
import threading
import numpy as np
from scipy.spatial.transform import Rotation
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

from airbot_py.arm import AIRBOTPlay

from .control.servo import Arm
from .vr import CoordinateConverter, VRTeleopRos2
from .utils import print_green, print_yellow, print_red

UPDATE_INTERVAL = 1 / 250


class VRArm(Node):

    INIT_POSE = np.array(
        [
            [1, 0, 0, 0.28628],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.21368],
            [0, 0, 0, 1],
        ],
        dtype=float,
    )

    init_joint_ctrl = np.array(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )

    arm_len = 0
    scale = 80.0 / 60

    target_control = np.zeros(12)
    startflag = False
    arm_init = False
    joints_left = np.array([0, 0, 0, 0, 0, 0])

    trans_init = None
    quat_init = None
    arm_init_pose = None

    def __init__(self, freq):
        Node.__init__(self, "vr_node")
        self.fps = freq
        self.tctr_arm = self.target_control[:6]
        self.teleop = VRTeleopRos2()
        self.init_gripper = 0.07

        self._init_sub()

        self.arm = Arm()
        self.arm.init()
        self.pose = self.INIT_POSE.copy()

    def _init_sub(self):
        self.sub = self.create_subscription(
            Float32MultiArray, "/vr_controller", self.teleop.vr_callback, 10
        )
        self.head_sub = self.create_subscription(
            Float32MultiArray, "/headInfo", self.teleop.head_info_callback, 10
        )
        self.left_sub = self.create_subscription(
            Float32MultiArray, "/leftInfo", self.teleop.left_info_callback, 10
        )
        self.right_sub = self.create_subscription(
            Float32MultiArray, "/rightInfo", self.teleop.right_info_callback, 10
        )
        print("Ros2VRCtl node is up")

    def reset(self):
        print_yellow("Robot reset to zero position")
        print_yellow(f"reset target: {self.INIT_POSE[:3, 3]}")
        self.pose = self.INIT_POSE.copy()
        self.arm.update_arm(self.pose)

        # 重置初始化标志
        self.arm_init = False
        self.trans_init = None
        self.quat_init = None
        self.arm_init_pose = None

    def teleopProcess(self):
        # 左手柄X键 重置位置
        if self.teleop.vr_cmd.data[10] > self.teleop.THRESHOLD:
            self.reset()
            self.startflag = True

        # 右手柄 A键 开始同步
        if self.teleop.vr_cmd.data[0] > self.teleop.THRESHOLD:
            self.startflag = True
            print_yellow("start VR control")

        # 右手柄 B键 结束同步
        if self.teleop.vr_cmd.data[1] > self.teleop.THRESHOLD:
            self.startflag = False
            self.arm_init = False
            self.trans_init = None
            self.quat_init = None
            self.arm_init_pose = None
            print_yellow("stop Arm control")

        if self.startflag:
            # 左臂控制  左手柄前扳机
            if self.teleop.vr_cmd.data[6] > self.teleop.THRESHOLD:

                # 左手柄侧方中指扳机 控制 夹爪
                self.tctr_gripper = (
                    self.init_gripper
                    - self.init_gripper * self.teleop.vr_cmd.data[8] ** 2
                )
                self.arm.update_eef(self.tctr_gripper)

                try:
                    trans = self.teleop.left_info.data[:3]
                    quat = self.teleop.left_info.data[3:7]
                    (
                        vr_trans,
                        vr_quat,
                    ) = CoordinateConverter.convert_left_to_right_handed(trans, quat)

                    # 初始化偏置
                    if not self.arm_init:
                        self.trans_init = np.array(vr_trans)
                        self.quat_init = np.array(vr_quat)
                        self.arm_init_pose = self.arm.get_end_pose()

                        print_green(f"VR初始位置: {self.trans_init}")
                        print_green(f"VR初始朝向: {self.quat_init}")
                        print_green(f"机械臂末端初始位姿: {self.arm_init_pose}\n")
                        self.arm_init = True

                    # 增量更新位姿
                    trans_delta = np.array(vr_trans) - self.trans_init
                    print_yellow(f"trans_delta: {trans_delta}")
                    quat_delta = (
                        Rotation.from_quat(self.quat_init)
                        * Rotation.from_quat(vr_quat).inv()
                    )
                    target_trans = self.arm_init_pose[0] + trans_delta
                    target_quat = (
                        (Rotation.from_quat(self.arm_init_pose[1]) * quat_delta)
                        .inv()
                        .as_quat()
                    )
                    T_left = np.eye(4)
                    T_left[:3, :3] = Rotation.from_quat(target_quat).as_matrix()
                    T_left[:3, 3] = target_trans
                    # print_yellow(f"T_left: {T_left}")

                    self.arm.update_arm(T_left)

                except Exception as e:
                    print_red(f"臂控制出错: {str(e)}\n")

            else:
                # 停止同步后重新计算偏置
                self.arm_init = False
                # 左手柄侧方中指扳机 控制 夹爪
                if self.teleop.vr_cmd.data[8] > self.teleop.THRESHOLD:
                    self.tctr_gripper = (
                        self.init_gripper
                        - self.init_gripper * self.teleop.vr_cmd.data[8] ** 2
                    )
                    print_yellow(
                        f"[夹爪控制调试] "
                        f"扳机值: {self.teleop.vr_cmd.data[6]}, "
                        f"输入轴值: {self.teleop.vr_cmd.data[8]}, "
                        f"目标位置: {[self.tctr_gripper]}\n"
                    )
                    self.arm.update_eef(self.tctr_gripper)

    def printMessage(self):
        if self.teleop.vrCmdRecv:
            print("VR Controller Status:")
            for i, value in enumerate(self.teleop.vr_cmd.data):
                print(f"    Control[{i}] = {value:.4f}")

        if self.teleop.headInfoRecv:
            print("Head Info Status:")
            for i, value in enumerate(self.teleop.head_info.data):
                print(f"    Head[{i}] = {value:.4f}")

        if self.teleop.leftInfoRecv:
            print("Left Hand Info Status:")
            for i, value in enumerate(self.teleop.left_info.data):
                print(f"    Left[{i}] = {value:.4f}")

        if self.teleop.rightInfoRecv:
            print("Right Hand Info Status:")
            for i, value in enumerate(self.teleop.right_info.data):
                print(f"    Right[{i}] = {value:.4f}")


if __name__ == "__main__":
    rclpy.init()

    pub_freq = 100
    vr_arm_node = VRArm(pub_freq)

    spin_thread = threading.Thread(target=lambda: rclpy.spin(vr_arm_node))
    spin_thread.start()

    rate = vr_arm_node.create_rate(pub_freq)
    while rclpy.ok():
        vr_arm_node.teleopProcess()
        rate.sleep()

    vr_arm_node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

# ============================================================
# vr_arm_rs.py  —— VR 控制 + RealSense 录像  (支持 mock_arm 模式)
# ============================================================

import sys, time, threading, numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Airbot SDK 路径
sys.path.append('/airbot_vr_py/airbot_vr_python_sdk')

# --------- 外部依赖模块 ----------
from airbot_vr.control.servo import Arm
from airbot_vr.vr import CoordinateConverter, VRTeleopRos2
from airbot_vr.utils import print_green, print_yellow, print_red
from recorder_serial_threads import VRDataRecorder


# --------- Mock 机械臂 ----------
class MockArm:
    """无机械臂时的轻量替身，实现 Arm 同名接口"""
    def __init__(self, init_T=None):
        self._T = np.eye(4) if init_T is None else init_T.copy()
        self._gripper = 0.07
        self.solve_fail = False  # 保持与真实臂一致的标志

    def init(self):
        pass

    def get_end_pose(self):
        pos = self._T[:3, 3].copy()
        quat = Rotation.from_matrix(self._T[:3, :3]).as_quat()
        return (pos, quat)

    def update_arm(self, T):
        self._T = T.copy()

    def update_eef(self, width):
        self._gripper = float(width)

    def get_joint_state(self):
        return None


# --------- 主类 ----------
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

    def __init__(self, freq, main_serial, wrist_serial, data_dir, mock_arm=False):
        Node.__init__(self, "vr_node")
        self.fps = freq
        self.teleop = VRTeleopRos2()

        # ---------- Arm 初始化 ----------
        if mock_arm:
            print_yellow("mock_arm=True，使用 MockArm（不连接真实机械臂）")
            self.arm = MockArm(init_T=self.INIT_POSE.copy())
        else:
            try:
                self.arm = Arm()
                self.arm.init()
                print_green("✅ 机械臂已连接")
            except Exception as e:
                print_red(f"⚠️ 机械臂初始化失败: {e}")
                print_yellow("自动切换到 MockArm（仅录像仿真）")
                self.arm = MockArm(init_T=self.INIT_POSE.copy())

        self.pose = self.INIT_POSE.copy()
        self.close_width, self.open_width = 0.0, 0.07
        self.startflag, self.arm_init = False, False

        # ---------- 录像 ----------
        self.recorder = VRDataRecorder(
            data_dir=data_dir,
            task="teleop",
            frequency=freq,
            main_serial=main_serial,
            wrist_serial=wrist_serial,
            depth_scale_mm=True
        )

        # ---------- 订阅 ----------
        self._init_sub()

    def _init_sub(self):
        self.create_subscription(Float32MultiArray, "/vr_controller", self.teleop.vr_callback, 10)
        self.create_subscription(Float32MultiArray, "/leftInfo", self.teleop.left_info_callback, 10)
        self.create_subscription(Float32MultiArray, "/rightInfo", self.teleop.right_info_callback, 10)
        print_green("✅ ROS2 VRArm node ready.")

    # ---------- 主控制循环 ----------
    # ---------- 主控制循环 ----------
    def teleopProcess(self):
        trigger = self.teleop.vr_cmd.data[6] > self.teleop.THRESHOLD   # 左扳机（录制）
        stop = self.teleop.vr_cmd.data[1] > self.teleop.THRESHOLD      # 右B键（停止）
        reset = self.teleop.vr_cmd.data[10] > self.teleop.THRESHOLD    # 左X键（重置）

        # ---------- 重置机械臂 ----------
        if reset:
            self.arm_init = False
            print_yellow("🔄 Robot reset.")
            return

        # ---------- 停止录像 ----------
        if stop and self.recorder._episode_started:
            time.sleep(0.5)  # 等相机线程写完缓存
            self.recorder.stop_episode()
            print_yellow("🟥 Stop arm & recording.")
            return

        # ---------- 开始录像 ----------
        if trigger and not self.recorder._episode_started:
            self.recorder.start_episode()
            print_green("🎥 扳机触发开始录像")

        # ---------- 若正在录像，持续更新姿态并记录 ----------
        if self.recorder._episode_started:
            try:
                # 获取VR左手位置和姿态
                trans = self.teleop.left_info.data[:3]
                quat = self.teleop.left_info.data[3:7]
                vr_t, vr_q = CoordinateConverter.convert_left_to_right_handed(trans, quat)

                # 初始化参考位姿
                if not self.arm_init:
                    self.trans_init, self.quat_init = np.array(vr_t), np.array(vr_q)
                    self.arm_init_pose = self.arm.get_end_pose()
                    self.arm_init = True

                # 计算相对位姿
                trans_delta = np.array(vr_t) - self.trans_init
                quat_delta = Rotation.from_quat(self.quat_init) * Rotation.from_quat(vr_q).inv()
                target_t = self.arm_init_pose[0] + trans_delta
                target_q = (Rotation.from_quat(self.arm_init_pose[1]) * quat_delta).inv().as_quat()

                # 构造目标矩阵
                T = np.eye(4)
                T[:3, :3] = Rotation.from_quat(target_q).as_matrix()
                T[:3, 3] = target_t

                # 更新机械臂 / 虚拟臂
                close_gripper = bool(self.teleop.vr_cmd.data[8])
                self.arm.update_arm(T)
                self.arm.update_eef(self.close_width if close_gripper else self.open_width)

                # ---------- 记录帧 ----------
                eef_pose = self.arm.get_end_pose()
                robot_data = {
                    "timestamp": float(time.time()),
                    "states": {
                        "eef_pose": {
                            "pos": eef_pose[0].tolist(),
                            "quat": eef_pose[1].tolist(),
                        }
                    },
                    "actions": {"grip": close_gripper},
                }
                self.recorder.record_step(robot_data)

            except Exception as e:
                print_red(f"控制错误: {e}")

        # ---------- 若未触发且未录像 ----------
        else:
            self.arm_init = False



# ---------- 主程序 ----------
if __name__ == "__main__":
    import argparse
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument("--main_serial", type=str, default="135122077225")
    parser.add_argument("--wrist_serial", type=str, default="943222073615")
    parser.add_argument("--data_dir", type=str, default="data_teleop")
    parser.add_argument("--freq", type=int, default=20)
    parser.add_argument("--mock_arm", action="store_true", help="使用虚拟机械臂（不连接服务器）")
    args = parser.parse_args()

    node = VRArm(args.freq, args.main_serial, args.wrist_serial, args.data_dir, mock_arm=args.mock_arm)
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node))
    spin_thread.start()

    rate = node.create_rate(args.freq)
    try:
        while rclpy.ok():
            node.teleopProcess()
            rate.sleep()
    finally:
        if node.recorder._episode_started:
            node.recorder.stop_episode()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

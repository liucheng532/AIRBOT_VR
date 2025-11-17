#!/usr/bin/env python3
import logging
import threading
import time
import sys

import numpy as np
from scipy.spatial.transform import Rotation
# from imu_socket import IMUManager
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray  # 目前没用到，但保留不动

import pyrealsense2 as rs
import cv2

from async_episode_writer import AsyncEpisodeWriter

# from Armsim import DummyArm

sys.path.append("/home/krystal/AIRBOT_VR/airbot_vr_python_sdk")
from airbot_vr.control.servo import Arm  # type: ignore
from airbot_vr.vr import CoordinateConverter, VRTeleopRos2  # type: ignore
from airbot_vr.utils import print_green, print_yellow, print_red  # type: ignore


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

    init_joint_ctrl = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    arm_len = 0
    scale = 80.0 / 60

    target_control = np.zeros(12)
    startflag = False
    arm_init = False
    joints_left = np.array([0, 0, 0, 0, 0, 0])

    trans_init = None
    quat_init = None
    arm_init_pose = None

    def __init__(self, freq: int, record_freq: int):
        super().__init__("vr_node")

        # ----------------- 原有遥操初始化 ----------------- #
        self.fps = freq
        self.tctr_arm = self.target_control[:6]
        self.teleop = VRTeleopRos2()
        self.init_gripper = 0.07
        self.tctr_gripper = self.init_gripper

        self._init_sub()

        # self.arm = DummyArm()
        self.arm = Arm()
        self.arm.init()
        self.pose = self.INIT_POSE.copy()

        # self.imu_manager = IMUManager("ws://10.192.1.2:5000", "SF_TRON1A_278")
        # self.imu_manager.start()

        # ----------------- 数据录制相关初始化 ----------------- #
        self.recording = False
        self.record_start_time = 0.0
        self.segment_events = []
        self._last_B_value = 0.0

        # 录制节流：目标 30Hz
        self._last_record_time = 0.0
        self.record_interval = 1.0 / record_freq  # 30FPS

        # EpisodeWriter，两个相机
        self.writer = AsyncEpisodeWriter(
            data_root="./dataset",
            camera_config=[
                {"name": "main", "has_rgb": True, "has_depth": True, "width": 640, "height": 480},
                {"name": "wrist", "has_rgb": True, "has_depth": False, "width": 640, "height": 480},
            ],
            freq=record_freq,
            gripper_open=0.07,
            gripper_close=0.0,
        )

        # 相机帧缓冲
        self.main_color = None  # A: 主相机 RGB
        self.main_depth = None  # A: 主相机 Depth
        self.wrist_color = None  # B: 腕部 RGB

        # 相机线程控制
        self._cam_stop = threading.Event()
        self._cam_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self._cam_thread.start()
        print_green("RealSense camera thread started.")

    # ----------------- 相机线程 ----------------- #
    def camera_loop(self):
        MAIN_SERIAL = "943222073615"  # 主相机 A（RGB + Depth）
        WRIST_SERIAL = "317222075228"  # 腕相机 B（RGB）

        main_pipe = None
        wrist_pipe = None

        try:
            # 主相机 A
            main_pipe = rs.pipeline()
            main_cfg = rs.config()
            main_cfg.enable_device(MAIN_SERIAL)
            main_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            main_cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            align = rs.align(rs.stream.color)
            main_pipe.start(main_cfg)

            # 腕相机 B
            wrist_pipe = rs.pipeline()
            wrist_cfg = rs.config()
            wrist_cfg.enable_device(WRIST_SERIAL)
            wrist_cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            wrist_pipe.start(wrist_cfg)

            print_green("[Camera] 两个 RealSense 相机已启动")

            while not self._cam_stop.is_set():
                # 主视角（阻塞等待）
                main_frames = main_pipe.wait_for_frames()
                aligned = align.process(main_frames)
                c = aligned.get_color_frame()
                d = aligned.get_depth_frame()

                if c and d:
                    self.main_color = np.asanyarray(c.get_data()).copy()
                    self.main_depth = np.asanyarray(d.get_data()).copy()

                # 腕部（非阻塞）
                wrist_frames = wrist_pipe.poll_for_frames()
                if wrist_frames:
                    wc = wrist_frames.get_color_frame()
                    if wc:
                        self.wrist_color = np.asanyarray(wc.get_data()).copy()

        except Exception as e:
            print_red(f"[Camera] 相机线程异常: {e}")

        finally:
            try:
                if main_pipe is not None:
                    main_pipe.stop()
                if wrist_pipe is not None:
                    wrist_pipe.stop()
            except Exception as e:
                print_red(f"[Camera] 相机关闭异常: {e}")

            print_green("[Camera] 相机线程结束")

    # ----------------- 打印限流 ----------------- #
    def throttled_print(self, msg, interval=2.0, color_fn=print_yellow):
        now = time.time()
        if not hasattr(self, "_last_print_time"):
            self._last_print_time = {}
        last_time = self._last_print_time.get(msg, 0)
        if now - last_time > interval:
            color_fn(msg)
            self._last_print_time[msg] = now

    # ----------------- ROS 订阅 ----------------- #
    def _init_sub(self):
        self.sub = self.create_subscription(Float32MultiArray, "/vr_controller", self.teleop.vr_callback, 10)
        self.head_sub = self.create_subscription(Float32MultiArray, "/headInfo", self.teleop.head_info_callback, 10)
        self.left_sub = self.create_subscription(Float32MultiArray, "/leftInfo", self.teleop.left_info_callback, 10)
        self.right_sub = self.create_subscription(Float32MultiArray, "/rightInfo", self.teleop.right_info_callback, 10)
        print("Ros2VRCtl node is up")

    # ----------------- 复位机械臂 ----------------- #
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
        # ==========================================================
        # 录制控制：B 键（右手 B 按钮）开始 / 结束（5 秒保护）
        # 做成「按一下触发一次」，而不是按住一直触发
        # ==========================================================
        B_value = self.teleop.vr_cmd.data[1]
        TH = self.teleop.THRESHOLD

        # 只检测从 0 → 1 的上升沿
        B_edge_pressed = (B_value > TH) and (self._last_B_value <= TH)

        if B_edge_pressed:
            # ★ 当前没录制 → 开始录制
            if not self.recording:
                print_green("开始录制 Episode（B）")
                self.writer.start()
                self.recording = True
                self.record_start_time = time.time()
                # 开始录制时重置节流计时
                self._last_record_time = 0.0
            else:
                # ★ 当前正在录制 → 尝试结束
                elapsed = time.time() - self.record_start_time
                if elapsed < 5.0:
                    # 少于 5 秒，不允许结束
                    print_yellow(f"录制仅 {elapsed:.1f} 秒，少于 5 秒，忽略这次 B 键结束请求")
                else:
                    print_yellow("停止录制 Episode（B，已超过 5 秒）")
                    self.writer.stop()
                    self.recording = False

        # ==========================================================
        # 录制状态：写帧（30Hz 节流）
        # ==========================================================
        if self.recording:
            now = time.time()
            # 没到下一帧时间就直接跳过，不写盘
            if now - self._last_record_time >= self.record_interval:
                self._last_record_time = now

                if self.main_color is not None and self.main_depth is not None and self.wrist_color is not None:
                    camera_data = {
                        "main": {
                            "rgb": self.main_color,
                            "depth": self.main_depth,
                        },
                        "wrist": {"rgb": self.wrist_color},
                    }

                    # 获取机械臂末端位姿
                    # imu = self.imu_manager.get_latest_imu()
                    robot_state = {
                        "joint_pos": self.arm.robot.get_joint_pos(),
                        "eef_pos": self.arm.robot.get_end_pose()[0],
                        "eef_quat": self.arm.robot.get_end_pose()[1],
                        "gripper": float(self.tctr_gripper),
                    }

                    # robot_state = {
                    #     # "imu_euler": imu["euler"],
                    #     # "imu_acc": imu["acc"],
                    #     # "imu_gyro": imu["gyro"],
                    #     # "imu_quat": imu["quat"],
                    # }

                    self.writer.add_item(camera_data, robot_state)

        # --- 这句很重要：更新上一帧 B 的值 ---
        self._last_B_value = B_value

        # 左手柄X键 重置位置
        if self.teleop.vr_cmd.data[10] > self.teleop.THRESHOLD:
            print_yellow("start reset, please wait...")
            self.reset()
            self.startflag = True

        # 右手柄 A键 开始同步
        if self.teleop.vr_cmd.data[0] > self.teleop.THRESHOLD:
            self.startflag = True
            print_yellow("start VR control")

        # 右手柄 Y键 结束同步
        if self.teleop.vr_cmd.data[11] > self.teleop.THRESHOLD:
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
                self.tctr_gripper = self.init_gripper - self.init_gripper * self.teleop.vr_cmd.data[8] ** 2
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

                    # print_yellow(f"trans_delta: {trans_delta}")
                    self.throttled_print(f"trans_delta: {trans_delta}", interval=1.0, color_fn=print_yellow)

                    quat_delta = Rotation.from_quat(self.quat_init) * Rotation.from_quat(vr_quat).inv()
                    target_trans = self.arm_init_pose[0] + trans_delta
                    target_quat = (Rotation.from_quat(self.arm_init_pose[1]) * quat_delta).inv().as_quat()
                    T_left = np.eye(4)
                    T_left[:3, :3] = Rotation.from_quat(target_quat).as_matrix()
                    T_left[:3, 3] = target_trans
                    # print_yellow(f"T_left: {T_left}")

                    self.arm.update_arm(T_left)

                except Exception as e:
                    # print_red(f"臂控制出错: {str(e)}\n")
                    self.throttled_print(f"臂控制出错: {str(e)}\n", interval=1.0, color_fn=print_red)

            else:
                # 停止同步后重新计算偏置
                self.arm_init = False
                # 左手柄侧方中指扳机 控制 夹爪
                if self.teleop.vr_cmd.data[8] > self.teleop.THRESHOLD:
                    self.tctr_gripper = self.init_gripper - self.init_gripper * self.teleop.vr_cmd.data[8] ** 2
                    print_yellow(
                        f"[夹爪控制调试] "
                        f"扳机值: {self.teleop.vr_cmd.data[6]}, "
                        f"输入轴值: {self.teleop.vr_cmd.data[8]}, "
                        f"目标位置: {[self.tctr_gripper]}\n"
                    )
                    self.arm.update_eef(self.tctr_gripper)

    # ----------------- 调试输出 ----------------- #
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

    # ----------------- 析构：确保相机线程退出 ----------------- #
    def __del__(self):
        try:
            self._cam_stop.set()
            if hasattr(self, "_cam_thread"):
                self._cam_thread.join(timeout=1)
        except Exception:
            pass


if __name__ == "__main__":
    rclpy.init()

    pub_freq = 100
    record_freq = 30
    vr_arm_node = VRArm(pub_freq, record_freq)

    # ROS spin 放到子线程
    spin_thread = threading.Thread(target=lambda: rclpy.spin(vr_arm_node), daemon=True)
    spin_thread.start()

    rate = vr_arm_node.create_rate(pub_freq)

    try:
        while rclpy.ok():
            vr_arm_node.teleopProcess()
            rate.sleep()
    finally:
        # 如果还在录制，先结束本次 episode
        if vr_arm_node.recording:
            print_yellow("Recording still active, stopping writer...")
            vr_arm_node.writer.stop()

        print_yellow("Stopping camera thread...")
        vr_arm_node._cam_stop.set()
        if hasattr(vr_arm_node, "_cam_thread"):
            vr_arm_node._cam_thread.join(timeout=2)

        vr_arm_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2)
        print_green("✅ 程序已安全退出。")

#!/usr/bin/env python3
import logging
import threading
import time
import sys
import select
import tty
import termios

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

sys.path.append("airbot_vr_python_sdk")
from airbot_vr.control.servo import Arm  # type: ignore
from airbot_vr.utils import print_green, print_yellow, print_red  # type: ignore

import os
import json
import collections
from utils import quat2axisangle, axisangle2quat


UPDATE_INTERVAL = 1 / 250

# 从robot_state.json获取机器人状态序列
def load_robot_states(ep_path):
    json_data = json.load(open(os.path.join(ep_path, "robot_state.json")))
    states = json_data['states']
    list_joint_pos = []
    list_eef_pose = []
    list_grip = []
    for state in states:
        joint_pos = np.array(state['joint_pos'])
        list_joint_pos.append(joint_pos)

        eef_pos = np.array(state['eef_pos'])
        eef_quat = np.array(state['eef_quat'])
        eef_rot = quat2axisangle(eef_quat)
        eef_pose = np.concatenate([eef_pos, eef_rot])
        list_eef_pose.append(eef_pose)

        gripper = state['gripper']
        assert gripper in [0.07, 0]
        grip = np.array([0]) if gripper == 0.07 else np.array([1])
        list_grip.append(grip)

    # 6D joint + 6D eef + 1D lift
    robot_states = np.concatenate([list_joint_pos, list_eef_pose, list_grip], axis=-1)
    return robot_states


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

    src_data_dir = "./dataset/episode_2025-11-19_22-45-56"

    def __init__(self, freq: int, record_freq: int, speedup=1, use_eef_act=True, reset=True, save=True):
        super().__init__("vr_node")

        # ----------------- 原有遥操初始化 ----------------- #
        self.fps = freq
        self.init_gripper = 0.07
        self.tctr_gripper = self.init_gripper

        # self._init_sub()

        # self.arm = DummyArm()
        self.arm = Arm()
        self.arm.init()
        self.pose = self.INIT_POSE.copy()

        # self.imu_manager = IMUManager("ws://10.192.1.2:5000", "SF_TRON1A_278")
        # self.imu_manager.start()

        # 录制节流：目标 30Hz
        self._last_record_time = 0.0
        self.record_interval = 1.0 / record_freq  # 30FPS

        # EpisodeWriter，两个相机
        self.writer = AsyncEpisodeWriter(
            data_root="./rollout",
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

        if reset:
            self.reset()

        self.speedup = speedup
        self.use_eef_act = use_eef_act
        self.action_plan = collections.deque()
        data_to_replay = load_robot_states(self.src_data_dir)
        self.action_plan.extend(data_to_replay)

        # 键盘监听控制
        self._should_stop = threading.Event()
        self._keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        self._keyboard_thread.start()
        print_green("键盘监听已启动，按 's' 键停止程序并保存数据")

        self.writer.start()
        self.recording = True
        self.record_start_time = time.time()
        # 开始录制时重置节流计时
        self._last_record_time = 0.0

    # ----------------- 相机线程 ----------------- #
    def camera_loop(self):
        MAIN_SERIAL = "317222075228"  # 主相机 A（RGB + Depth）
        WRIST_SERIAL = "943222073615"  # 腕相机 B（RGB）

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

    # ----------------- 键盘监听线程 ----------------- #
    def _keyboard_listener(self):
        """监听键盘输入，按 's' 键停止程序"""
        # 保存原始终端设置
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            # 设置为原始模式，可以立即读取按键
            tty.setraw(sys.stdin.fileno())
            
            while not self._should_stop.is_set():
                # 非阻塞检查是否有输入
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 's':
                        print_yellow("\n检测到 's' 键，正在停止程序并保存数据...")
                        self._should_stop.set()
                        break
        except Exception as e:
            print_red(f"[Keyboard] 键盘监听异常: {e}")
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # ----------------- 打印限流 ----------------- #
    def throttled_print(self, msg, interval=2.0, color_fn=print_yellow):
        now = time.time()
        if not hasattr(self, "_last_print_time"):
            self._last_print_time = {}
        last_time = self._last_print_time.get(msg, 0)
        if now - last_time > interval:
            color_fn(msg)
            self._last_print_time[msg] = now



    # ----------------- 复位机械臂 ----------------- #
    def reset(self):
        print_yellow("Robot reset to zero position")
        print_yellow(f"reset target: {self.INIT_POSE[:3, 3]}")
        self.pose = self.INIT_POSE.copy()
        self.arm.update_arm(self.pose)


    def rolloutProcess(self):

        now = time.time()
        # 没到下一帧时间就直接跳过
        if now - self._last_record_time >= self.record_interval:
            self._last_record_time = now

            if self.main_color is None or self.main_depth is None or self.wrist_color is None:
                return

            # 获取机械臂末端位姿
            # imu = self.imu_manager.get_latest_imu()
            robot_state = {
                "joint_pos": self.arm.robot.get_joint_pos(),
                "eef_pos": self.arm.robot.get_end_pose()[0],
                "eef_quat": self.arm.robot.get_end_pose()[1],
                "gripper": float(self.tctr_gripper),
                # "imu_euler": imu["euler"],
                # "imu_acc": imu["acc"],
                # "imu_gyro": imu["gyro"],
                # "imu_quat": imu["quat"],
            }

            camera_data = {
                "main": {
                    "rgb": self.main_color,
                    "depth": self.main_depth,
                },
                "wrist": {"rgb": self.wrist_color},
            }
            print_green(f"self.main_color: {self.main_color.shape}")

            if len(self.action_plan) == 0:
                return

            # get action
            next_act_step_size = min(self.speedup, len(self.action_plan))
            for _ in range(next_act_step_size):
                action = self.action_plan.popleft()

            joint_act = action[:6]
            eef_pos = action[6:9]
            eef_rot = action[9:12]
            eef_quat = axisangle2quat(eef_rot)
            eef_act = np.eye(4)
            eef_act[:3, :3] = Rotation.from_quat(eef_quat).as_matrix()
            eef_act[:3, 3] = eef_pos
            grip_act = action[12]
            grip_act = 0 if grip_act==1 else 0.07

            self.arm.update_eef(grip_act)

            try:
                self.arm.update_arm(eef_act)
            except Exception as e:
                # print_red(f"臂控制出错: {str(e)}\n")
                self.throttled_print(f"臂控制出错: {str(e)}\n", interval=1.0, color_fn=print_red)
            
            self.writer.add_item(camera_data, robot_state)


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
    reset = True
    save = True
    vr_arm_node = VRArm(pub_freq, record_freq, reset=reset, save=save)

    # ROS spin 放到子线程
    spin_thread = threading.Thread(target=lambda: rclpy.spin(vr_arm_node), daemon=True)
    spin_thread.start()

    rate = vr_arm_node.create_rate(pub_freq)

    try:
        while rclpy.ok() and not vr_arm_node._should_stop.is_set():
            vr_arm_node.rolloutProcess()
            rate.sleep()
    finally:
        vr_arm_node.reset()
        # 如果还在录制，先结束本次 episode
        if vr_arm_node.recording:
            print_yellow("Recording still active, stopping writer...")
            vr_arm_node.writer.stop()
            vr_arm_node.recording = False

        print_yellow("Stopping camera thread...")
        vr_arm_node._cam_stop.set()
        if hasattr(vr_arm_node, "_cam_thread"):
            vr_arm_node._cam_thread.join(timeout=2)

        # 停止键盘监听线程
        vr_arm_node._should_stop.set()
        if hasattr(vr_arm_node, "_keyboard_thread"):
            vr_arm_node._keyboard_thread.join(timeout=1)

        vr_arm_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2)
        print_green("✅ 程序已安全退出。")
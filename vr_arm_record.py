import logging
import threading
import queue
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

# Optional imports used only for data recording
try:
    from pyorbbecsdk import Config, Pipeline, OBSensorType, OBFormat, OBAlignMode, FrameSet
except Exception:
    pass

try:
    import cv2
except Exception:
    pass

from single_episode_writer import EpisodeWriter


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

    def __init__(self, freq, close_width=0, open_width=0.07, save=False, data_dir="debug", task="play", wrist_cam_indices=()):
        Node.__init__(self, "vr_node")
        self.fps = freq
        self.tctr_arm = self.target_control[:6]
        self.teleop = VRTeleopRos2()
        self.close_width = close_width
        self.open_width = open_width

        self._init_sub()

        self.arm = Arm()
        self.arm.init()
        self.pose = self.INIT_POSE.copy()

        # Recording related
        self.save_enabled = bool(save)
        self._stop_recording = threading.Event()
        self._recorder = None
        self._main_cam_thread = None
        self._wrist_cam_thread = None
        self._main_vision_stack = queue.LifoQueue()
        self._wrist_vision_stack = queue.LifoQueue()
        self._wrist_cam_indices = wrist_cam_indices

        if self.save_enabled:
            try:
                self._recorder = EpisodeWriter(
                    data_dir=data_dir,
                    task=task,
                    close_width=close_width,
                    open_width=open_width,
                    frequency=freq,
                )
                self._recorder.create_episode()
                self._start_camera_threads()
                print_green("Recording enabled: episode created and camera threads started")
            except Exception as e:
                print_red(f"Failed to initialize recorder/cameras: {e}")
                self.save_enabled = False

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

        eef_pose = self.arm.get_end_pose()
        close_gripper = bool(self.teleop.vr_cmd.data[8])

        if self.startflag:
            # 左臂控制  左手柄前扳机
            if self.teleop.vr_cmd.data[6] > self.teleop.THRESHOLD:

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
                        self.arm_init_pose = eef_pose

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

                    # main_vision_data: (color_data, depth_data)
                    main_vision_data = self._main_vision_stack.get()
                    # wrist_vision_data: dict of frames {"rgb_wrist_i": frame}
                    wrist_vision_data = self._wrist_vision_stack.get()

                    self.arm.update_arm(T_left)
                    self.arm.update_eef(self.close_width if close_gripper else self.open_width)

                    # Data recording hook (optional)
                    if not self.arm.solve_fail and self.save_enabled:
                        robot_data = {
                            "states": {
                                "eef_pose_l": eef_pose,  # expected as (trans, quat)
                            },
                            "actions": {
                                "act_grip_l": close_gripper, # binary
                            },
                        }
                        self._recorder.add_item(
                            main_vision_data=main_vision_data,
                            wrist_vision_data=wrist_vision_data,
                            robot_data=robot_data,
                        )

                except Exception as e:
                    print_red(f"臂控制出错: {str(e)}\n")

            else:
                # 停止同步后重新计算偏置
                self.arm_init = False

        

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

    # ----------------------- Cameras & Recording -----------------------
    def _start_camera_threads(self):

        def main_camera_stream():
            try:
                config = Config()
                pipeline = Pipeline()
                profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                color_profile = profile_list.get_default_video_stream_profile()
                config.enable_stream(color_profile)
                profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
                depth_profile = profile_list.get_default_video_stream_profile()
                config.enable_stream(depth_profile)
                config.set_align_mode(OBAlignMode.HW_MODE)
                pipeline.enable_frame_sync()
                pipeline.start(config)

                while not self._stop_recording.is_set():
                    frames = pipeline.wait_for_frames(100)
                    if frames is None:
                        continue
                    color_frame = frames.get_color_frame()
                    depth_frame = frames.get_depth_frame()
                    if color_frame is None or depth_frame is None:
                        continue
                    color_data = np.asanyarray(color_frame.get_data())
                    depth_data = depth_frame.get_data()
                    self._main_vision_stack.put((color_data, depth_data))
            except Exception as e:
                print_red(f"Main camera stream error: {e}")

        def wrist_camera_stream(indices):
            try:
                caps = []
                for device_index in indices:
                    cap = cv2.VideoCapture(device_index)
                    caps.append(cap)
                while not self._stop_recording.is_set():
                    content = {}
                    valid = True
                    for cap in caps:
                        ret = cap.grab()
                        if not ret:
                            valid = False
                    if not valid:
                        continue
                    for i, cap in enumerate(caps):
                        _, frame = cap.retrieve()
                        content[f"rgb_wrist_{i}"] = frame
                    self._wrist_vision_stack.put(content)
                for cap in caps:
                    cap.release()
            except Exception as e:
                print_red(f"Wrist camera stream error: {e}")

        self._main_cam_thread = threading.Thread(target=main_camera_stream, daemon=True)
        self._main_cam_thread.start()

        self._wrist_cam_thread = threading.Thread(
            target=wrist_camera_stream, args=(self._wrist_cam_indices,), daemon=True
        )
        self._wrist_cam_thread.start()

    def _stop_camera_threads(self):
        if self._main_cam_thread or self._wrist_cam_thread:
            self._stop_recording.set()
        if self._main_cam_thread:
            self._main_cam_thread.join(timeout=2)
        if self._wrist_cam_thread:
            self._wrist_cam_thread.join(timeout=2)

    def close_recording(self):
        if self.save_enabled:
            try:
                self._stop_camera_threads()
                self._recorder.save_episode()
                print_green("Episode saved")
            except Exception as e:
                print_red(f"Failed to close recording: {e}")


if __name__ == "__main__":
    import argparse
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument('--pub_freq', type=int, default=20)
    parser.add_argument('--close_width', type=float, default=0.0)
    parser.add_argument('--open_width', type=float, default=0.07)
    parser.add_argument('--save', action='store_true', default=False)
    parser.add_argument('--data_dir', type=str, default='debug')
    parser.add_argument('--task', type=str, default='play')
    parser.add_argument('--wrist_cam_ids', type=int, nargs='*', default=())
    args = parser.parse_args()

    pub_freq = args.pub_freq

    vr_arm_node = VRArm(
        pub_freq,
        args.close_width,
        args.open_width,
        save=args.save,
        data_dir=args.data_dir,
        task=args.task,
        wrist_cam_indices=args.wrist_cam_ids,
        frequency=args.frequency,
    )

    spin_thread = threading.Thread(target=lambda: rclpy.spin(vr_arm_node))
    spin_thread.start()

    rate = vr_arm_node.create_rate(pub_freq)
    try:
        while rclpy.ok():
            vr_arm_node.teleopProcess()
            rate.sleep()
    finally:
        try:
            vr_arm_node.close_recording()
        except Exception:
            pass
        vr_arm_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

import threading
import queue
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from airbot_py.arm import AIRBOTPlay

import sys
sys.path.append('/airbot_vr_py/airbot_vr_python_sdk')
from airbot_vr.control.servo import Arm
from airbot_vr.vr import CoordinateConverter, VRTeleopRos2
from airbot_vr.utils import print_green, print_yellow, print_red

import pyrealsense2 as rs
import cv2
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

    def __init__(self, freq, close_width=0, open_width=0.07, save=False, data_dir="debug", task="play"):
        Node.__init__(self, "vr_node")
        
        # Initialize parameters
        self.fps = freq
        self.close_width = close_width
        self.open_width = open_width
        self.init_gripper = 0.07
        self.recording = False
        self.trigger_prev = 0.0


        # VR control setup
        self.teleop = VRTeleopRos2()
        self._init_sub()

        print_green("✅ VRTeleopRos2 node up")

        self.arm = Arm()
        self.arm.init()

        # # Arm initialization
        # class DummyArm:
        #     def __init__(self):
        #         self._count = 0
        #     def init(self):
        #         print_yellow("[DummyArm] 模拟机械臂已初始化")
        #     def update_arm(self, pose):
        #         self._count += 1
        #         if self._count % 100 == 0:
        #             print_yellow(f"[DummyArm] 更新位姿 {self._count} 次")
        #     def update_eef(self, width):
        #         pass
        #     def get_end_pose(self):
        #         trans = np.array([0.3, 0.0, 0.2])
        #         quat = np.array([0, 0, 0, 1])
        #         return (trans, quat)

        # self.arm = DummyArm()

        # Initial pose of the arm
        self.pose = self.INIT_POSE.copy()

        # Recording setup
        self.save_enabled = bool(save)
        self._stop_recording = threading.Event()
        self._main_cam_thread = None
        self._main_vision_stack = queue.LifoQueue(maxsize=5)
        self._wrist_vision_stack = queue.LifoQueue(maxsize=5)
        self._closed = False

        # If saving enabled, create episode for recording
        if self.save_enabled:
            self._recorder = EpisodeWriter(data_dir=data_dir, task=task,
                                           close_width=close_width,
                                           open_width=open_width,
                                           frequency=freq)
            self._recorder.create_episode()
            self._recorder.main_img_size = (640, 480)
            self._recorder.wrist_img_size = (640, 480)
            self._start_camera_threads()  # Start the camera threads for capturing video
        else:
            self._recorder = None


    # ROS2订阅
    def _init_sub(self):
        self.create_subscription(Float32MultiArray, "/vr_controller",
                                 self.teleop.vr_callback, 10)
        self.create_subscription(Float32MultiArray, "/headInfo",
                                 self.teleop.head_info_callback, 10)
        self.create_subscription(Float32MultiArray, "/leftInfo",
                                 self.teleop.left_info_callback, 10)
        self.create_subscription(Float32MultiArray, "/rightInfo",
                                 self.teleop.right_info_callback, 10)

    # 相机线程
    def _start_camera_threads(self):
        MAIN_SERIAL = "317222075228"
        WRIST_SERIAL = "943222073615"

        def camera_thread():
            pipeline_main, pipeline_wrist = None, None
            try:
                pipeline_main = rs.pipeline()
                config_main = rs.config()
                config_main.enable_device(MAIN_SERIAL)
                config_main.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                config_main.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                align = rs.align(rs.stream.color)
                pipeline_main.start(config_main)
                print_green(f"✅ 主相机启动成功：{MAIN_SERIAL}")
                time.sleep(2.0)

                pipeline_wrist = rs.pipeline()
                config_wrist = rs.config()
                config_wrist.enable_device(WRIST_SERIAL)
                config_wrist.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                pipeline_wrist.start(config_wrist)
                print_green(f"✅ 腕部相机启动成功：{WRIST_SERIAL}")

                while not self._stop_recording.is_set():
                    frames_main = pipeline_main.wait_for_frames(timeout_ms=1000)
                    aligned = align.process(frames_main)
                    color_frame = aligned.get_color_frame()
                    depth_frame = aligned.get_depth_frame()
                    if color_frame and depth_frame:
                        color_img = np.asanyarray(color_frame.get_data()).copy()
                        depth_img = np.asanyarray(depth_frame.get_data()).copy()
                        if not self._main_vision_stack.full():
                            self._main_vision_stack.put((color_img, depth_img))

                    frames_wrist = pipeline_wrist.poll_for_frames()
                    if frames_wrist:
                        color_wrist = frames_wrist.get_color_frame()
                        if color_wrist:
                            wrist_np = np.asanyarray(color_wrist.get_data()).copy()
                            if not self._wrist_vision_stack.full():
                                self._wrist_vision_stack.put({"rgb_wrist_0": wrist_np})
            except Exception as e:
                print_red(f"相机线程出错: {e}")
            finally:
                try:
                    if pipeline_main: pipeline_main.stop()
                    if pipeline_wrist: pipeline_wrist.stop()
                except Exception as e:
                    print_red(f"关闭相机异常: {e}")

        self._main_cam_thread = threading.Thread(target=camera_thread, daemon=True)
        self._main_cam_thread.start()

    def _stop_camera_threads(self):
        self._stop_recording.set()
        if self._main_cam_thread and self._main_cam_thread.is_alive():
            self._main_cam_thread.join(timeout=5)
        print_green("✅ Camera threads stopped.")

    def close_recording(self):
        if not self.save_enabled or self._closed:
            return
        self._closed = True
        try:
            self._stop_camera_threads()
            self._recorder.save_episode()
            print_green("💾 Episode saved.")
        except Exception as e:
            print_red(f"❌ Failed to close recording: {e}")

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

    # 主逻辑
    def teleopProcess(self):
        try:
            # B 键停止录制并保存
            if self.teleop.vr_cmd.data[1] > self.teleop.THRESHOLD:
                if self.recording:
                    print_yellow("停止录制 (B)")
                self.recording = False
                if self.save_enabled and self._recorder:
                    self.close_recording()  # 停止录制并保存
                    print_green("录制已保存。")

            # 扳机按钮触发录制
            trigger_val = self.teleop.vr_cmd.data[6]
            if (not self.trigger_prev > self.teleop.THRESHOLD) and (trigger_val > self.teleop.THRESHOLD):
                if not self.recording:
                    self.recording = True
                    self.arm_init = False
                    print_green("开始录制 (扳机按钮触发)")
            self.trigger_prev = trigger_val


            # A 键启动同步
            if self.teleop.vr_cmd.data[0] > self.teleop.THRESHOLD:
                self.startflag = True
                print_yellow("开始同步控制 (A 键)")

            # B 键停止同步
            if self.teleop.vr_cmd.data[1] > self.teleop.THRESHOLD:
                self.startflag = False
                self.arm_init = False
                self.trans_init = None
                self.quat_init = None
                self.arm_init_pose = None
                print_yellow("停止同步控制 (B 键)")

            # X 键复位机械臂
            if self.teleop.vr_cmd.data[10] > self.teleop.THRESHOLD:
                self.reset()
                self.startflag = True

            if self.startflag:
                # 左手柄前扳机控制左臂
                if self.teleop.vr_cmd.data[6] > self.teleop.THRESHOLD:
                    # 左手柄侧方中指扳机控制夹爪
                    self.tctr_gripper = (
                        self.init_gripper
                        - self.init_gripper * self.teleop.vr_cmd.data[8] ** 2
                    )
                    self.arm.update_eef(self.tctr_gripper)

                    try:
                        trans = self.teleop.left_info.data[:3]
                        quat = self.teleop.left_info.data[3:7]
                        vr_trans, vr_quat = CoordinateConverter.convert_left_to_right_handed(trans, quat)

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

                # 数据采集（如果启用录制且正在录制）
                if self.save_enabled and self.recording:
                    if self._main_vision_stack.empty():
                        print_yellow("主相机数据为空，等待数据")
                    elif self._wrist_vision_stack.empty():
                        print_yellow("腕部相机数据为空，等待数据")
                    else:
                        # print_green("获取主相机和腕部相机数据")
                        color_img, depth_img = self._main_vision_stack.get()
                        wrist_data = self._wrist_vision_stack.get()
                        _, color_encoded = cv2.imencode('.jpg', color_img)
                        depth_bytes = depth_img.tobytes()
                        main_data = (color_encoded, depth_bytes)
                        robot_data = {
                            "states": {"eef_pose_l": self.arm.get_end_pose()[0]},
                            "actions": {"act_grip_l": self.tctr_gripper},
                        }
                        self._recorder.add_item(main_data, wrist_data, robot_data)

                        if len(self._recorder.episode_data) % 20 == 0:
                            print_green(f"[录制中] 当前帧数: {len(self._recorder.episode_data)}")

        except Exception as e:
            print_red(f"控制出错: {e}")




# 主入口
if __name__ == "__main__":
    import argparse
    rclpy.init()

    parser = argparse.ArgumentParser()
    parser.add_argument('--pub_freq', type=int, default=20)
    parser.add_argument('--close_width', type=float, default=0.0)
    parser.add_argument('--open_width', type=float, default=0.07)
    parser.add_argument('--save', action='store_true', default=True)
    parser.add_argument('--data_dir', type=str, default='/airbot_vr_py/zzj_test')
    parser.add_argument('--task', type=str, default='vr_play')
    args = parser.parse_args()

    node = VRArm(args.pub_freq, args.close_width, args.open_width,
                 save=args.save, data_dir=args.data_dir, task=args.task)

    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        print_green("✅ VR 模式启动成功，等待手柄输入...")
        period = 1.0 / max(1, args.pub_freq)
        while rclpy.ok():
            node.teleopProcess()
            time.sleep(period)
    finally:
        node.close_recording()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2)
        print("✅ 程序已安全退出。")

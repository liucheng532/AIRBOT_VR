import math
import random
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation

from airbot_py.arm import AIRBOTPlay, RobotMode, SpeedProfile
from mmk2_kdl_py import ArmKdlNumerical
from .lpf import OnlineVariableStepLPF
from ..utils import print_yellow, timed


def distance(pos: np.ndarray, target: np.ndarray) -> float:
    return np.linalg.norm(pos - target)


def limit_pos(current: np.ndarray, target: np.ndarray, max_diff: float) -> np.ndarray:
    diff = target - current
    diff = np.clip(diff, -max_diff, max_diff)
    return current + diff


def limit_pose(pose: np.ndarray, target: np.ndarray, max_diff: float) -> np.ndarray:
    diff = target[:3, 3] - pose[:3, 3]
    diff = np.clip(diff, -max_diff, max_diff)
    pose[:3, 3] = pose[:3, 3] + diff
    pose[:3, :3] = target[:3, :3]
    return pose


class Arm:
    def __init__(self):
        self.robot = AIRBOTPlay(port=50051)
        self.robot.connect()
        self.robot.switch_mode(RobotMode.PLANNING_POS)
        self.robot.set_speed_profile(SpeedProfile.DEFAULT)
        self.robot.move_eef_pos(0.07)
        self.robot.move_to_joint_pos([0, 0, 0, math.pi / 2, 0, -math.pi / 2])

        self.robot.set_params({"dm_motor_can0_4.normal_current_thres": 7.5})
        self.robot.set_params({"dm_motor_can0_5.normal_current_thres": 7.5})
        self.robot.set_params({"dm_motor_can0_6.normal_current_thres": 7.5})

        self.robot.switch_mode(RobotMode.SERVO_JOINT_POS)
        speed_params = {
            "servo_node.moveit_servo.scale.linear": 1.0,
            "servo_node.moveit_servo.scale.rotational": 1.0,
            "servo_node.moveit_servo.scale.joint": 1.0,
            "sdk_server.max_velocity_scaling_factor": 0.5,
            "sdk_server.max_acceleration_scaling_factor": 0.5,
        }
        self.robot.set_params(speed_params)

        self.arm_kdl = ArmKdlNumerical(eef_type="G2")
        self.lpfs = [
            OnlineVariableStepLPF(fc=100, kp=10, kd=0.2, v_max=5000, a_max=100)
            for _ in range(6)
        ]
        # TODO pending parameter tuning
        self.eef_dist = 0.07
        self.running = threading.Event()
        self.running.set()

        # Initialize pose servoing parameters and state
        self.max_translation_step_m = 0.001
        self.max_rotation_step_rad = math.radians(3.0)
        end_pose = self.robot.get_end_pose()
        self.command_pose_mat = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        self.command_pose_mat[:3, :3] = Rotation.from_quat(end_pose[1]).as_matrix()
        self.command_pose_mat[:3, 3] = end_pose[0]
        self.target_pose_mat = self.command_pose_mat.copy()

        self.position_bias = np.array([0, 0, 0], dtype=float)

        self.solve_fail = False
        self.allow_flip = True

    def __enter__(self):
        self.init()
        return self

    def init(self):
        self.t = threading.Thread(target=self.run)
        # self.t.daemon = True
        self.t.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.release()

    def get_end_pose(self):
        return self.robot.get_end_pose()

    # @timed
    def update_arm(self, pose: np.ndarray):
        target_pose_mat = np.array(pose, dtype=float)
        if target_pose_mat.shape != (4, 4):
            raise ValueError("pose must be a 4x4 homogeneous transform matrix")
        # Update target and step commanded pose toward it for smooth motion
        self.target_pose_mat = target_pose_mat

        joint_pos = self.robot.get_joint_pos()
        result = self.arm_kdl.inverse_kinematics(target_pose_mat, joint_pos)
        # print_yellow(f"arm_kdl result: {result}")
        if len(result) == 0:
            self.solve_fail = True
            print(f"Inverse kinematics failed. Given pose: {target_pose_mat}")
            return
        elif np.all(
            np.abs(
                np.array([joint_pos[3], joint_pos[5]]) - [result[0][3], result[0][5]]
            )
            > math.pi * 2 / 3
        ):
            # print_yellow(f"current: {joint_pos[3]:.3f}, {joint_pos[5]:.3f}")
            # print_yellow(f"current: {result[0][3]:.3f}, {result[0][5]:.3f}")
            if not self.allow_flip:
                self.solve_fail = True
                return
        else:
            self.solve_fail = False
        pos = result[0]
        print_yellow(f"pos: {pos}")
        # pos = limit_pos(self.robot.get_joint_pos(), pos, math.pi / 12)
        now = time.time()
        for i in range(6):
            self.lpfs[i].update(now, pos[i])

    def update_joints(self, joints: list[float]):
        now = time.time()
        for i in range(6):
            self.lpfs[i].update(now, joints[i])

    def reset(self):
        now = time.time()
        # [0, 0, 0, math.pi / 2, 0, -math.pi / 2]
        self.lpfs[0].update(now, 0)
        self.lpfs[1].update(now, 0)
        self.lpfs[2].update(now, 0)
        self.lpfs[3].update(now, math.pi / 2)
        self.lpfs[4].update(now, 0)
        self.lpfs[5].update(now, -math.pi / 2)

    def update_eef(self, eef_dist: float):
        self.eef_dist = eef_dist

    def run(self):
        while self.running.is_set():
            pos = self.robot.get_joint_pos()
            now = time.time()
            for i in range(6):
                pos[i] = self.lpfs[i].sample(now)
            self.robot.servo_joint_pos(pos)
            self.robot.servo_eef_pos(self.eef_dist)
            time.sleep(1 / 250)

    def release(self):
        self.running.clear()
        self.t.join()
        self.robot.disconnect()


if __name__ == "__main__":
    with Arm() as arm:
        start = time.time()
        pose = np.array(
            [
                [1.0, 0.0, 0.0, 0.606],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.213],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        while time.time() - start < 20:
            pose[1][3] = 0.05 * math.sin(time.time() - start)
            pose[2][3] = 0.213 + 0.05 * (1 - math.cos(time.time() - start))

            arm.update_arm(pose)
            arm.update_eef(0.07 * math.sin(time.time()))
            time.sleep(random.random() * 0.1)
            # time.sleep(0.1)

    print("finished")

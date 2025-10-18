import math
import numpy as np
from std_msgs.msg import Float32MultiArray


class VRTeleopRos2:
    THRESHOLD = 0.1
    NUM_CONTROL = 12

    def __init__(self):
        self.vr_cmd = Float32MultiArray()
        self.vr_cmd.data = [0.0] * self.NUM_CONTROL
        self.button_cooldown = 0.8  # 设置冷却时间为1秒
        self.last_buttons = np.zeros(self.NUM_CONTROL, np.float32)
        self.raising_sig = np.zeros(self.NUM_CONTROL, np.bool_)
        self.falling_sig = np.zeros(self.NUM_CONTROL, np.bool_)
        self.vrCmdRecv = False

        self.head_info = Float32MultiArray()
        self.head_info.data = []
        self.left_info = Float32MultiArray()
        self.left_info.data = []
        self.right_info = Float32MultiArray()
        self.right_info.data = []

        self.headInfoRecv = False
        self.leftInfoRecv = False
        self.rightInfoRecv = False

        print("META Quest VRTeleopRos2 node is up")

    def reset(self):
        self.vr_cmd.data = [0.0] * self.NUM_CONTROL
        self.raising_sig[:] = False
        self.falling_sig[:] = False
        self.vrCmdRecv = False

    def get_raising_edge(self, i):
        if i < len(self.raising_sig):
            if self.raising_sig[i]:
                self.raising_sig[i] = False
                return True
            else:
                return False
        else:
            return None

    def get_falling_edge(self, i):
        if i < len(self.falling_sig):
            if self.falling_sig[i]:
                self.falling_sig[i] = False
                return True
            else:
                return False
        else:
            return None

    def vr_callback(self, msg: Float32MultiArray):
        self.vr_cmd = msg
        current_buttons = np.array(msg.data) > 0.1
        self.raising_sig = np.logical_or(
            self.raising_sig,
            np.logical_and(current_buttons, np.logical_not(self.last_buttons)),
        )
        self.falling_sig = np.logical_or(
            self.falling_sig,
            np.logical_and(np.logical_not(current_buttons), self.last_buttons),
        )

        self.last_buttons = current_buttons
        self.vrCmdRecv = True

    def head_info_callback(self, msg: Float32MultiArray):
        self.head_info = msg
        self.headInfoRecv = True
        # print("Head Info Received:", np.array2string(np.array(msg.data), precision=3, suppress_small=True))

    def left_info_callback(self, msg: Float32MultiArray):
        self.left_info = msg
        self.leftInfoRecv = True
        # print("Left Hand Info Received:", np.array2string(np.array(msg.data), precision=3, suppress_small=True))

    def right_info_callback(self, msg: Float32MultiArray):
        self.right_info = msg
        self.rightInfoRecv = True
        # print("Right Hand Info Received:", np.array2string(np.array(msg.data), precision=3, suppress_small=True))

    def joint_info_callback(self, msg: Float32MultiArray):
        self.Unity_joint_info = msg
        # self.rightInfoRecv = True
        # print("Right Hand Info Received:", np.array2string(np.array(msg.data), precision=3, suppress_small=True))


class CoordinateConverter:
    """坐标系转换类：将左手坐标系(Z前)转换为右手坐标系(X前,Y左,Z上)"""

    @staticmethod
    def normalize_quaternion(x, y, z, w):
        magnitude = math.sqrt(x * x + y * y + z * z + w * w)
        if magnitude < 1e-10:
            return [0.0, 0.0, 0.0, 1.0]
        return [x / magnitude, y / magnitude, z / magnitude, w / magnitude]

    @staticmethod
    def quaternion_to_axis_angle(q):
        q = CoordinateConverter.normalize_quaternion(q[0], q[1], q[2], q[3])
        angle = 2 * math.acos(q[3])

        if math.isclose(angle, 0.0, abs_tol=1e-10):
            return [0.0, 0.0, 1.0, 0.0]

        s = math.sqrt(1 - q[3] * q[3])
        if s < 1e-10:
            axis = [1.0, 0.0, 0.0]
        else:
            axis = [q[0] / s, q[1] / s, q[2] / s]

        return [*axis, angle]

    @staticmethod
    def axis_angle_to_quaternion(axis, angle):
        magnitude = math.sqrt(sum(x * x for x in axis))
        if magnitude < 1e-10:
            return [0.0, 0.0, 0.0, 1.0]

        normalized_axis = [x / magnitude for x in axis]

        half_angle = angle / 2
        sin_half = math.sin(half_angle)

        q = [
            normalized_axis[0] * sin_half,
            normalized_axis[1] * sin_half,
            normalized_axis[2] * sin_half,
            math.cos(half_angle),
        ]

        return q

    @staticmethod
    def convert_left_to_right_handed(position, rotation):
        """
        将左手坐标系(X右, Y上, Z前 )转换为右手坐标系(X前,Y左,Z上)
        参数:
            position: [x, y, z] 原始左手坐标系位置
            rotation: [x, y, z, w] 原始左手坐标系四元数
        返回:
            transformed_position: [x, y, z] 转换后的右手坐标系位置
            transformed_rotation: [x, y, z, w] 转换后的右手坐标系四元数
        """
        transformed_position = [
            position[2],  # 新X = 原Z (前向)
            -position[0],  # 新Y = -原X (左向)
            position[1],  # 新Z = 原Y (上向)
        ]

        original_axis_angle = CoordinateConverter.quaternion_to_axis_angle(rotation)

        original_axis = original_axis_angle[0:3]
        angle = original_axis_angle[3]

        transformed_axis = [
            original_axis[2],  # 新X = 原Z
            -original_axis[0],  # 新Y = -原X
            original_axis[1],  # 新Z = 原Y
        ]

        # 从转换后的轴和角度创建新的四元数
        # 由于从左手系到右手系的转换，需要反转旋转方向（取反角度）
        transformed_rotation = CoordinateConverter.axis_angle_to_quaternion(
            transformed_axis, -angle
        )
        transformed_rotation = CoordinateConverter.normalize_quaternion(
            *transformed_rotation
        )

        return transformed_position, transformed_rotation

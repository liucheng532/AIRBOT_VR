#!/usr/bin/env python3
"""
TCP到ROS2桥接器
将您的Quest3应用（ControllerPoseSender.cs）的TCP数据转换为ROS2话题
兼容原有的airbot_vr.vr_arm系统
"""

import socket
import threading
import re
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.spatial.transform import Rotation


class TCPToROS2Bridge(Node):
    def __init__(self):
        super().__init__('tcp_to_ros2_bridge')
        
        # TCP服务器配置
        self.HOST = "0.0.0.0"
        self.PORT = 8000
        
        # ROS2发布器（与原有系统兼容）
        self.vr_controller_pub = self.create_publisher(Float32MultiArray, '/vr_controller', 10)
        self.left_info_pub = self.create_publisher(Float32MultiArray, '/leftInfo', 10)
        self.right_info_pub = self.create_publisher(Float32MultiArray, '/rightInfo', 10)
        self.head_info_pub = self.create_publisher(Float32MultiArray, '/headInfo', 10)
        
        # 状态变量
        self.button_state = {
            "LGrip": False,
            "RGrip": False,
            "LTrig": False,
            "RTrig": False,
            "PauseL": False,
            "PauseR": False,
            "EXIT": False
        }
        
        # 位置和旋转数据
        self.left_pos = [0.0, 0.0, 0.0]
        self.left_quat = [0.0, 0.0, 0.0, 1.0]
        self.right_pos = [0.0, 0.0, 0.0]
        self.right_quat = [0.0, 0.0, 0.0, 1.0]
        
        # 正则表达式
        self.pos_pattern = re.compile(r'([LR])Pos:\s*\(([^)]+)\)')
        self.rot_pattern = re.compile(r'([LR])Rot:\s*\(([^)]+)\)')
        
        # 启动TCP服务器
        self.start_tcp_server()
        
        # 定时发布ROS2数据
        self.timer = self.create_timer(0.05, self.publish_ros_data)  # 20Hz
        
        self.get_logger().info(f"TCP到ROS2桥接器已启动，监听端口: {self.PORT}")

    def start_tcp_server(self):
        """启动TCP服务器"""
        self.server_thread = threading.Thread(target=self.tcp_server_loop, daemon=True)
        self.server_thread.start()

    def tcp_server_loop(self):
        """TCP服务器主循环"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.HOST, self.PORT))
        server_socket.listen(5)
        
        self.get_logger().info(f"TCP服务器监听 {self.HOST}:{self.PORT}")
        
        while True:
            try:
                client_socket, client_address = server_socket.accept()
                self.get_logger().info(f"Quest3客户端连接: {client_address}")
                
                client_thread = threading.Thread(
                    target=self.handle_client,
                    args=(client_socket, client_address),
                    daemon=True
                )
                client_thread.start()
                
            except Exception as e:
                self.get_logger().error(f"TCP服务器错误: {e}")

    def handle_client(self, client_socket, client_address):
        """处理客户端连接"""
        buffer = b""
        
        try:
            while True:
                data = client_socket.recv(1024)
                if not data:
                    break
                    
                buffer += data
                
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        message = line.decode('ascii', errors='ignore').strip()
                        if message:
                            self.parse_quest3_data(message)
                    except Exception as e:
                        self.get_logger().warn(f"解析数据错误: {e}")
                        
        except Exception as e:
            self.get_logger().error(f"客户端 {client_address} 错误: {e}")
        finally:
            client_socket.close()
            self.get_logger().info(f"Quest3客户端 {client_address} 断开连接")

    def parse_quest3_data(self, message):
        """解析Quest3应用发送的数据"""
        # 解析按键状态
        if "=" in message:
            parts = message.split(';')
            for part in parts:
                if '=' in part:
                    key, value = part.split('=', 1)
                    key = key.strip()
                    value = value.strip()
                    
                    if key == "LGrip":
                        self.button_state["LGrip"] = (value == 'T')
                    elif key == "RGrip":
                        self.button_state["RGrip"] = (value == 'T')
                    elif key == "LTrig":
                        self.button_state["LTrig"] = (value == 'T')
                    elif key == "RTrig":
                        self.button_state["RTrig"] = (value == 'T')
                    elif key == "PauseL":
                        self.button_state["PauseL"] = (value == 'T')
                    elif key == "PauseR":
                        self.button_state["PauseR"] = (value == 'T')
                    elif key == "EXIT":
                        self.button_state["EXIT"] = (value == 'T')
        
        # 解析位置数据
        pos_matches = self.pos_pattern.findall(message)
        for hand, pos_str in pos_matches:
            try:
                coords = [float(x.strip()) for x in pos_str.split(',')]
                if len(coords) == 3:
                    if hand == 'L':
                        self.left_pos = coords
                    elif hand == 'R':
                        self.right_pos = coords
            except ValueError:
                pass
        
        # 解析旋转数据（欧拉角转四元数）
        rot_matches = self.rot_pattern.findall(message)
        for hand, rot_str in rot_matches:
            try:
                euler_angles = [float(x.strip()) for x in rot_str.split(',')]
                if len(euler_angles) == 3:
                    # Unity欧拉角（度）转四元数
                    euler_rad = np.radians(euler_angles)
                    rotation = Rotation.from_euler('xyz', euler_rad)
                    quat = rotation.as_quat()  # [x, y, z, w]
                    
                    if hand == 'L':
                        self.left_quat = quat.tolist()
                    elif hand == 'R':
                        self.right_quat = quat.tolist()
            except ValueError:
                pass

    def publish_ros_data(self):
        """发布ROS2数据（兼容原有vr_arm.py的格式）"""
        
        # 构建/vr_controller话题数据（12个浮点数）
        # 按照原有系统的按键映射：
        # [0] 右手柄A键 -> 用RGrip代替（开始控制）
        # [1] 右手柄B键 -> 用EXIT代替（停止控制）
        # [2-5] 摇杆数据（暂未使用）
        # [6] 左手柄前扳机 -> LTrig
        # [7] 右手柄前扳机 -> RTrig
        # [8] 左手柄侧方中指扳机 -> LGrip（夹爪控制）
        # [9] 右手柄侧方中指扳机（暂未使用）
        # [10] 左手柄X键 -> PauseL（重置）
        # [11] 左手柄Y键 -> EXIT
        
        vr_controller_msg = Float32MultiArray()
        vr_controller_data = [
            1.0 if self.button_state["RGrip"] else 0.0,   # [0] 开始控制
            1.0 if self.button_state["EXIT"] else 0.0,    # [1] 停止控制
            0.0, 0.0, 0.0, 0.0,                           # [2-5] 摇杆
            1.0 if self.button_state["LTrig"] else 0.0,   # [6] 左手前扳机
            1.0 if self.button_state["RTrig"] else 0.0,   # [7] 右手前扳机
            1.0 if self.button_state["LGrip"] else 0.0,   # [8] 夹爪控制
            0.0,                                           # [9] 保留
            1.0 if self.button_state["PauseL"] else 0.0,  # [10] 重置
            1.0 if self.button_state["EXIT"] else 0.0,    # [11] 退出
        ]
        vr_controller_msg.data = vr_controller_data
        self.vr_controller_pub.publish(vr_controller_msg)
        
        # 发布左手信息: [x, y, z, qx, qy, qz, qw]
        left_info_msg = Float32MultiArray()
        left_info_msg.data = self.left_pos + self.left_quat
        self.left_info_pub.publish(left_info_msg)
        
        # 发布右手信息: [x, y, z, qx, qy, qz, qw]
        right_info_msg = Float32MultiArray()
        right_info_msg.data = self.right_pos + self.right_quat
        self.right_info_pub.publish(right_info_msg)
        
        # 发布头部信息（使用默认值）
        head_info_msg = Float32MultiArray()
        head_info_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.head_info_pub.publish(head_info_msg)


def main(args=None):
    rclpy.init(args=args)
    
    bridge = TCPToROS2Bridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("收到中断信号，正在关闭...")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from twist_client import WebSocketTwistClient

class TwistFromVR(Node):
    def __init__(self):
        super().__init__("twist_from_vr")

        # -------------------------
        # WebSocket 初始化
        # -------------------------
        self.ws = WebSocketTwistClient(
            ws_url="ws://10.192.1.2:5000",
            accid="SF_TRON1A_278",
            rate_hz=50,  # 提高发送频率到50Hz
        )
        self.ws.start()

        # -------------------------
        # 订阅 VR 控制器数据
        # -------------------------
        self.create_subscription(
            Float32MultiArray,
            "/vr_controller",
            self.vr_callback,
            10,
        )

        self.get_logger().info("TwistFromVR 已启动并连接 WebSocket.")

    # -----------------------------
    # 主逻辑
    # -----------------------------
    def vr_callback(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 6:
            return

        # 直接将摇杆数据传递给 WebSocketTwistClient 进行处理
        self.ws.update_cmd_from_vr(data)


        # print(f"[VR] Data sent to WebSocket: {data}")

    # -----------------------------
    # 结束
    # -----------------------------
    def destroy(self):
        self.ws.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TwistFromVR()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

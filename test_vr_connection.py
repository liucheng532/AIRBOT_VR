#!/usr/bin/env python3
"""
VR连接测试脚本
用于测试Quest3应用与系统的连接状态
"""

import socket
import threading
import time
import re

class VRConnectionTester:
    def __init__(self, port=8000):
        self.port = port
        self.host = "0.0.0.0"
        self.running = False
        self.client_count = 0
        
        # 数据统计
        self.message_count = 0
        self.last_message_time = 0
        
        # 解析器
        self.pos_pattern = re.compile(r'([LR])Pos:\s*\(([^)]+)\)')
        self.rot_pattern = re.compile(r'([LR])Rot:\s*\(([^)]+)\)')

    def start_server(self):
        """启动测试服务器"""
        self.running = True
        server_thread = threading.Thread(target=self._server_loop, daemon=True)
        server_thread.start()
        
        print(f"🚀 VR连接测试服务器启动")
        print(f"📡 监听端口: {self.port}")
        print(f"🔗 等待Quest3应用连接...")
        print("=" * 50)

    def _server_loop(self):
        """服务器主循环"""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(5)
        
        while self.running:
            try:
                client_socket, client_address = server_socket.accept()
                self.client_count += 1
                print(f"✅ 客户端连接: {client_address} (总连接数: {self.client_count})")
                
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, client_address),
                    daemon=True
                )
                client_thread.start()
                
            except Exception as e:
                if self.running:
                    print(f"❌ 服务器错误: {e}")

    def _handle_client(self, client_socket, client_address):
        """处理客户端连接"""
        buffer = b""
        
        try:
            while self.running:
                data = client_socket.recv(1024)
                if not data:
                    break
                    
                buffer += data
                
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        message = line.decode('ascii', errors='ignore').strip()
                        if message:
                            self._process_message(message, client_address)
                    except Exception as e:
                        print(f"⚠️  解析错误: {e}")
                        
        except Exception as e:
            print(f"❌ 客户端 {client_address} 错误: {e}")
        finally:
            client_socket.close()
            print(f"🔌 客户端 {client_address} 断开连接")

    def _process_message(self, message, client_address):
        """处理接收到的消息"""
        self.message_count += 1
        self.last_message_time = time.time()
        
        # 每100条消息显示一次统计
        if self.message_count % 100 == 0:
            print(f"📊 已接收 {self.message_count} 条消息")
        
        # 解析并显示关键信息
        if "LGrip=T" in message or "RGrip=T" in message:
            print(f"🎮 按键触发: {message}")
        
        if "EXIT=T" in message:
            print(f"🛑 收到退出信号: {client_address}")
        
        # 解析位置数据
        pos_matches = self.pos_pattern.findall(message)
        if pos_matches and self.message_count % 50 == 0:  # 每50条消息显示一次位置
            for hand, pos_str in pos_matches:
                try:
                    coords = [float(x.strip()) for x in pos_str.split(',')]
                    if len(coords) == 3:
                        print(f"📍 {hand}手位置: ({coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f})")
                except ValueError:
                    pass

    def show_status(self):
        """显示连接状态"""
        while self.running:
            time.sleep(5)
            current_time = time.time()
            
            if self.last_message_time > 0:
                time_since_last = current_time - self.last_message_time
                if time_since_last < 1.0:
                    status = "🟢 活跃"
                elif time_since_last < 5.0:
                    status = "🟡 空闲"
                else:
                    status = "🔴 无数据"
            else:
                status = "⚪ 等待连接"
            
            print(f"📈 状态: {status} | 消息总数: {self.message_count} | 连接数: {self.client_count}")

    def run(self):
        """运行测试器"""
        self.start_server()
        
        # 启动状态显示线程
        status_thread = threading.Thread(target=self.show_status, daemon=True)
        status_thread.start()
        
        try:
            print("\n📝 使用说明:")
            print("1. 在Quest3应用中设置IP地址为本机IP")
            print("2. 确保Quest3和PC在同一网络")
            print("3. 在Quest3应用中点击连接")
            print("4. 观察此窗口的连接状态和数据流")
            print("\n按 Ctrl+C 停止测试")
            print("=" * 50)
            
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n🛑 停止测试...")
            self.running = False

def main():
    print("🔧 VR连接测试工具")
    print("=" * 50)
    
    tester = VRConnectionTester()
    tester.run()

if __name__ == "__main__":
    main()

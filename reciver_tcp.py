# visualize_poses_terminal.py
import socket
import threading
import collections
import re
import time

# 配置
HOST = "0.0.0.0"
PORT = 8000
BUFFER = 5000  # 缓存最近 N 个点（可选）

# 缓存（可用于后续分析，当前仅作记录）
L_buf = collections.deque(maxlen=BUFFER)  # 左手位置缓存
R_buf = collections.deque(maxlen=BUFFER)  # 右手位置缓存

# 状态字典
state = {
    "LGrip": "F", "RGrip": "F",
    "LTrig": "F", "RTrig": "F",
    "PauseL": "F", "PauseR": "F",
    "EXIT":   "F"
}

# 正则匹配 LPos/RPos: (x, y, z)
POS_RE = re.compile(r"""
    (?P<tag>LPos|RPos)\s*:\s*
    \(\s*(?P<x>[-+]?[\d\.eE]+)\s*,\s*
       (?P<y>[-+]?[\d\.eE]+)\s*,\s*
       (?P<z>[-+]?[\d\.eE]+)\s*\)
""", re.VERBOSE)

def parse_line(line: str):
    """解析一行数据并更新状态和缓存"""
    line = line.strip()
    if not line:
        return

    # 处理单个状态指令（如 PauseL=T）
    for key in state:
        if line.startswith(key + "="):
            _, value = line.split("=", 1)
            state[key] = value.strip()
            return

    # 分号分割 KV 对（如 LGrip=T;RTrig=F）
    parts = [p.strip() for p in line.split(";") if p.strip()]
    for p in parts:
        if "=" in p:
            k, v = p.split("=", 1)
            k, v = k.strip(), v.strip()
            if k in state:
                state[k] = v

    # 解析位置数据
    for m in POS_RE.finditer(line):
        tag = m.group("tag")
        try:
            x = float(m.group("x"))
            y = float(m.group("y"))
            z = float(m.group("z"))
        except ValueError:
            continue

        if tag == "LPos":
            L_buf.append((x, y, z))
            print(f"⬅️  LEFT  : x={x:8.3f}, y={y:8.3f}, z={z:8.3f}")
        elif tag == "RPos":
            R_buf.append((x, y, z))
            print(f"➡️  RIGHT : x={x:8.3f}, y={y:8.3f}, z={z:8.3f}")


def client_thread(conn, addr):
    """处理单个客户端连接"""
    print(f"🔗 连接来自 {addr}")
    try:
        data = b""
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            data += chunk
            # 按换行拆分处理
            while b"\n" in data:
                line, data = data.split(b"\n", 1)
                try:
                    s = line.decode("ascii", errors="ignore").strip()
                except:
                    s = ""
                if s:
                    parse_line(s)
    except Exception as e:
        print(f"❌ 客户端 {addr} 出错: {e}")
    finally:
        print(f"🔌 客户端 {addr} 断开")


def tcp_server():
    """启动 TCP 服务器"""
    print(f"📡 开始监听 TCP {HOST}:{PORT}")
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(32)
    
    while True:
        conn, addr = srv.accept()
        t = threading.Thread(target=client_thread, args=(conn, addr), daemon=True)
        t.start()


def print_status_periodically():
    """每秒打印一次状态（非阻塞）"""
    while True:
        status_str = (
            f" | Grip: L={state['LGrip']} R={state['RGrip']} "
            f"| Trig: L={state['LTrig']} R={state['RTrig']} "
            f"| Pause: L={state['PauseL']} R={state['PauseR']} "
            f"| EXIT={state['EXIT']}"
        )
        print(f"📌 状态更新: {status_str}")
        
        if state["EXIT"] == "T":
            print("🛑 收到退出指令，程序即将结束...")
            break
            
        time.sleep(1.0)  # 每秒刷新一次状态


def main():
    # 启动 TCP 服务器线程
    server_thread = threading.Thread(target=tcp_server, daemon=True)
    server_thread.start()

    # 打印提示
    print("="*80)
    print("✅ 已启动 - 等待 TCP 数据...")
    print("💡 数据格式示例: LPos: (0.123, 0.456, 0.789); LGrip=T; RTrig=F")
    print("📌 按 Ctrl+C 退出")
    print("="*80)

    # 启动状态打印线程
    status_thread = threading.Thread(target=print_status_periodically, daemon=True)
    status_thread.start()

    # 保持主线程运行
    try:
        while True:
            time.sleep(0.1)
            # 如果收到 EXIT=T，退出
            if state["EXIT"] == "T":
                print("🎉 已收到退出指令，关闭程序。")
                break
    except KeyboardInterrupt:
        print("\n👋 被用户中断，正在退出...")


if __name__ == "__main__":
    main()
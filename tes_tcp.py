import socket

HOST = '0.0.0.0'     # 监听所有网卡（必须这样）
PORT = 8000          # 你 Unity 里设置的端口

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(5)

print(f"Server listening on port {PORT} ...")

while True:
    conn, addr = server.accept()
    print("Connected by:", addr)
    
    data = conn.recv(4096)
    if not data:
        conn.close()
        continue

    print("Received:", data.decode(errors="ignore"))

    conn.close()

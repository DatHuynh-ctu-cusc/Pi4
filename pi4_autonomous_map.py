# Pi4: nhận bản đồ mini từ Pi5 và dùng trong scan_callback
import socket
import threading
import json

# Biến toàn cục lưu bản đồ mini 1m x 1m quanh robot (10x10 với resolution 0.1m)
local_map = [[0 for _ in range(10)] for _ in range(10)]

# Hàm lắng nghe Pi5 gửi bản đồ nhỏ
def receive_local_map():
    global local_map
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", 9000))
    server.listen(1)
    print("[Pi4] 🟢 Đang chờ bản đồ nhỏ từ Pi5...")
    while True:
        conn, addr = server.accept()
        print(f"[Pi4] ✅ Nhận bản đồ mini từ {addr}")
        with conn:
            buffer = ""
            while True:
                try:
                    chunk = conn.recv(4096).decode()
                    if not chunk:
                        break
                    buffer += chunk
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        data = json.loads(line.strip())
                        if "local_map" in data:
                            local_map = data["local_map"]
                except:
                    break

# Hàm gợi ý: kiểm tra nếu vùng trung tâm phía trước đã quét

def center_front_clear():
    for y in range(0, 3):
        for x in range(3, 7):
            if local_map[y][x] == 1:
                return False
    return True

def left_front_clear():
    for y in range(0, 3):
        for x in range(0, 3):
            if local_map[y][x] == 1:
                return False
    return True

def right_front_clear():
    for y in range(0, 3):
        for x in range(7, 10):
            if local_map[y][x] == 1:
                return False
    return True
# Có thể gọi hàm này trong scan_callback của Pi4:
# if mean_c > THRESH_CLEAR and center_front_clear():
#     self.safe_move("forward")
# else: chọn hướng khác

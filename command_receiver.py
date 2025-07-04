# command_receiver.py
import socket
import json
import threading
import time
from motor_control import move_vehicle, stop_all_motors
from receiver import counts, lock

# lưu lại số encoder ban đầu và số cần đi
target_counts = 0
start_left = 0
start_right = 0
moving = False

def wait_for_valid_encoder():
    print("[Pi4] ⏳ Đợi encoder từ Pi5...")
    while True:
        with lock:
            if all(isinstance(v, int) for v in counts.values()):
                return
        time.sleep(0.05)

def handle_command(conn):
    global target_counts, start_left, start_right, moving
    try:
        data = conn.recv(1024).decode().strip()
        cmd = json.loads(data)

        if cmd.get("command") == "go":
            direction = cmd.get("direction", "forward")
            target_counts = cmd.get("target_counts", 0)
            print(f"[Pi4] 🟢 Lệnh GO: {direction} {target_counts:.0f} counts")
            wait_for_valid_encoder()
            with lock:
                start_left = (counts['E1'] + counts['E2']) / 2
                start_right = (counts['E3'] + counts['E4']) / 2
            moving = True
            move_vehicle(direction, 0.6)

        elif cmd.get("command") == "stop":
            print(f"[Pi4] 🔴 Lệnh STOP: dừng xe")
            moving = False
            stop_all_motors()

    except Exception as e:
        print(f"[Pi4] ❌ Lỗi xử lý lệnh: {e}")
    finally:
        conn.close()

def monitor_encoder():
    global moving
    while True:
        time.sleep(0.05)
        if not moving:
            continue
        with lock:
            left_now = (counts['E1'] + counts['E2']) / 2
            right_now = (counts['E3'] + counts['E4']) / 2
        moved = ((abs(left_now - start_left) + abs(right_now - start_right)) / 2)
        if moved >= target_counts:
            print("[Pi4] ✅ Đã đạt đủ counts, dừng xe.")
            stop_all_motors()
            moving = False

def start_command_listener():
    PORT = 9002
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', PORT))
    server.listen(1)
    print(f"[Pi4] 📡 Chờ lệnh GO/STOP (port {PORT})...")
    threading.Thread(target=monitor_encoder, daemon=True).start()
    while True:
        conn, _ = server.accept()
        threading.Thread(target=handle_command, args=(conn,), daemon=True).start()

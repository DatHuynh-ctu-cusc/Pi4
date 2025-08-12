import os, math, json, socket, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from motor_control import move_vehicle, stop_all
import shared_state

SHOW_DEBUG = False  # Bật debug bản đồ ASCII

class LiDARNode(Node):
    def __init__(self, counts):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sock = None
        self.counts = counts
        self.moving = False
        self.last_action_time = self.get_clock().now()
        self.connect_to_pi5()
        self.ping_thread = threading.Thread(target=self.ping_loop, daemon=True)
        self.ping_thread.start()

    def connect_to_pi5(self):
        while True:
            try:
                print("[Pi4] 🔄 Đang kết nối Pi5...")
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect(("192.168.100.2", 8899))
                self.sock.settimeout(0.1)
                print("[Pi4] ✅ Kết nối thành công.")
                break
            except:
                time.sleep(2)

    def ping_loop(self):
        while True:
            try:
                now = time.time()
                if now - self.last_action_time.nanoseconds / 1e9 >= 1.0:
                    self.sock.sendall(b"PING\n")
                try:
                    response = self.sock.recv(1024).decode()
                    if "PONG" in response:
                        self.last_action_time = self.get_clock().now()
                except socket.timeout:
                    pass
            except:
                time.sleep(2)
                self.connect_to_pi5()

    def scan_callback(self, msg):
        # 1. Gửi dữ liệu LiDAR sang Pi5
        try:
            self.sock.sendall((json.dumps({
                "ranges": list(msg.ranges),
                "angle_min": msg.angle_min,
                "angle_increment": msg.angle_increment
            }) + "\n").encode())
        except:
            self.sock.close()
            self.connect_to_pi5()

        # 2. Bỏ qua khi đang chạy path
        if shared_state.running_path:
            return

        # 3. Kiểm tra trạng thái tự hành
        if not shared_state.running_scan:
            stop_all()
            return

        # Nếu công tắc đang giữ và không yêu cầu né, thì bỏ qua
        if shared_state.limit_active and not shared_state.escape_required:
            return

        # 4. Tạo bản đồ quét để điều hướng
        size, scale, center = 40, 0.1, 20
        map_array = np.full((size, size), '.', dtype=str)
        THRESH_CLEAR = 1.3

        free_l, free_c, free_r, valid_points = [], [], [], 0

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance): continue
            angle = msg.angle_min + i * msg.angle_increment
            deg = math.degrees(angle)
            x = int(center + (distance * math.sin(angle)) / scale)
            y = int(center - (distance * math.cos(angle)) / scale)
            if 0 <= x < size and 0 <= y < size:
                map_array[y][x] = '#'
            if 60 <= deg <= 120:
                free_l.append(distance)
            elif -50 <= deg <= 50:
                free_c.append(distance)
            elif -120 <= deg <= -60:
                free_r.append(distance)
            valid_points += 1

        if valid_points == 0:
            stop_all()
            self.moving = False
            return

        mean_l = np.mean([d for d in free_l if d < 3.0]) if free_l else 0
        mean_c = np.mean([d for d in free_c if d < 3.0]) if free_c else 0
        mean_r = np.mean([d for d in free_r if d < 3.0]) if free_r else 0

        # ✅ Cập nhật vào shared_state để receiver.py dùng
        shared_state.mean_l = mean_l
        shared_state.mean_r = mean_r

        if SHOW_DEBUG:
            os.system("clear")
            for row in map_array[::-1]:
                print(" ".join(row))
            print(f"[DEBUG] Trái={mean_l:.2f} | Giữa={mean_c:.2f} | Phải={mean_r:.2f}")

        # 5. Điều hướng robot
        now = self.get_clock().now()
        if (now - self.last_action_time).nanoseconds / 1e9 > 0.5 and not self.moving:
            if mean_c > THRESH_CLEAR:
                self.safe_move("forward")
            elif mean_l > THRESH_CLEAR / 2 or mean_r > THRESH_CLEAR / 2:
                if mean_l > mean_r:
                    self.safe_move("left")
                else:
                    self.safe_move("right")
            else:
                print("[AI] 🛑 Cả 3 hướng bị chặn")
                stop_all()
                self.moving = False
            self.last_action_time = now

            # Nếu đang né vật (từ công tắc L3/L4), thì cho phép tiếp tục
            if shared_state.escape_required:
                print("[AI] ✅ Đã tránh công tắc xong – tiếp tục tự hành")
                shared_state.escape_required = False

    def safe_move(self, direction):
        if self.moving:
            return
        def worker():
            self.moving = True
            move_vehicle(direction, 0.02, 1.0, self.counts)
            self.moving = False
        threading.Thread(target=worker, daemon=True).start()

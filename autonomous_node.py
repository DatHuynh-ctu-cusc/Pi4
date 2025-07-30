import os, math, json, socket, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from motor_control import move_vehicle, stop_all
import shared_state  # ✅ Sử dụng trạng thái dùng chung

SHOW_DEBUG = False  # ✅ Bật = True nếu muốn xem bản đồ ASCII và debug

class LiDARNode(Node):
    def __init__(self, counts):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sock = None
        self.ping_time = time.time()
        self.last_pong_time = time.time()
        self.counts = counts
        self.moving = False
        self.last_action_time = self.get_clock().now()

        self.connect_to_pi5()
        self.ping_thread = threading.Thread(target=self.ping_loop, daemon=True)
        self.ping_thread.start()

    def connect_to_pi5(self):
        while True:
            try:
                print("[Pi4] 🔄 Đang cố kết nối tới Pi5...")
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect(("192.168.100.2", 8899))
                self.sock.settimeout(0.1)
                print("[Pi4] ✅ Kết nối lại thành công – Gửi LiDAR đến Pi5")
                break
            except:
                time.sleep(2)

    def ping_loop(self):
        while True:
            try:
                now = time.time()
                if now - self.ping_time >= 1.0:
                    self.sock.sendall(b"PING\n")
                    self.ping_time = now
                try:
                    response = self.sock.recv(1024).decode()
                    if "PONG" in response:
                        self.last_pong_time = now
                except socket.timeout:
                    pass
                if now - self.last_pong_time > 3:
                    print("[Pi4] ❌ Mất kết nối với Pi5 – Dừng xe")
                    stop_all()
                    self.sock.close()
                    self.connect_to_pi5()
                    self.last_pong_time = time.time()
            except Exception as e:
                print(f"[Pi4] 🔌 Lỗi ping: {e}")
                time.sleep(2)

    def scan_callback(self, msg):
        # 1. Gửi LiDAR sang Pi5
        try:
            self.sock.sendall((json.dumps({
                "ranges": list(msg.ranges),
                "angle_min": msg.angle_min,
                "angle_increment": msg.angle_increment
            }) + "\n").encode())
        except:
            self.sock.close()
            self.connect_to_pi5()

        # 2. Kiểm tra chế độ quét
        if not shared_state.running_scan:
            stop_all()
            return
        if shared_state.limit_active and not shared_state.escape_required:
            return

        # 3. Phân tích LiDAR để điều khiển tự hành
        size, scale, center = 40, 0.1, 20
        map_array = np.full((size, size), '.', dtype=str)
        THRESH_CLEAR = 1.4

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

        if SHOW_DEBUG:
            os.system("clear")
            for row in map_array[::-1]:
                print(" ".join(row))
            print(f"[DEBUG] Trái={mean_l:.2f} | Giữa={mean_c:.2f} | Phải={mean_r:.2f}")

        # 4. Xử lý thoát va chạm
        if shared_state.escape_required:
            print("[AI] 🔄 Đang xử lý hướng thoát sau va chạm L3/L4")
            if mean_l > mean_r:
                self.safe_move("left")
            else:
                self.safe_move("right")
            shared_state.escape_required = False
            shared_state.limit_active = False
            return

        # 5. Điều hướng tự động
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
                print("[AI] 🛑 Cả 3 hướng đều bị chặn – DỪNG HẲN")
                stop_all()
                self.moving = False
            self.last_action_time = now

    def safe_move(self, direction):
        if self.moving:
            return
        def worker():
            self.moving = True
            move_vehicle(direction, 0.02, 1.0, self.counts)
            self.moving = False
        threading.Thread(target=worker, daemon=True).start()

def main(args=None):
    rclpy.init(args=args)
    counts = {'E1': 0, 'E2': 0, 'E3': 0, 'E4': 0}
    node = LiDARNode(counts)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

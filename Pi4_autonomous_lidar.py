# Pi4_autonomous_lidar.py

import os, math, time, json, threading, socket, subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from gpiozero import Motor, PWMOutputDevice
from time import sleep
from bluetooth_rfcomm_server import BluetoothServer
import shared_state  # Dùng biến toàn cục chia sẻ

# ==== GPIO cấu hình động cơ ====
motor1 = Motor(forward=24, backward=25, pwm=True)
motor1_pwm = PWMOutputDevice(17)
motor2 = Motor(forward=22, backward=23, pwm=True)
motor2_pwm = PWMOutputDevice(12)
motor3 = Motor(forward=21, backward=27, pwm=True)
motor3_pwm = PWMOutputDevice(4)
motor4 = Motor(forward=19, backward=20, pwm=True)
motor4_pwm = PWMOutputDevice(16)
pwm_r1 = motor3_pwm
pwm_r2 = motor4_pwm
pwm_l1 = motor1_pwm
pwm_l2 = motor2_pwm

# ==== Biến encoder nhận từ Pi5 ====
count_l1 = count_l2 = count_r1 = count_r2 = 0
CPR = 200

# ==== PID controller ====
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0

    def compute(self, current):
        error = self.setpoint - current
        self.integral += error
        self.integral = max(min(self.integral, 100), -100)
        derivative = error - self.last_error
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

def stop_all():
    print("[DEBUG] stop_all() được gọi.")
    motor1.stop(); motor2.stop(); motor3.stop(); motor4.stop()
    pwm_l1.value = pwm_l2.value = pwm_r1.value = pwm_r2.value = 0

def set_motor(forward_l, forward_r):
    print(f"[DEBUG] set_motor(forward_l={forward_l}, forward_r={forward_r})")
    if forward_l:
        motor1.forward(); motor2.forward()
    else:
        motor1.backward(); motor2.backward()
    if forward_r:
        motor3.forward(); motor4.forward()
    else:
        motor3.backward(); motor4.backward()

def move_vehicle(direction="forward", target_rps=0.15, duration=1.0):
    global count_r1, count_r2, count_l1, count_l2
    print(f"[DEBUG] move_vehicle(direction={direction}, running_scan={shared_state.running_scan})")
    # --- KHÔNG BAO GIỜ cho phép chạy nếu chưa start_scan! ---
    if not shared_state.running_scan:
        print("[SECURITY] move_vehicle bị chặn vì chưa start_scan.")
        stop_all()
        return

    count_r1 = count_r2 = count_l1 = count_l2 = 0

    pid_r = PID(Kp=0.002, Ki=0.00001, Kd=0.0001, setpoint=target_rps)
    pid_l = PID(Kp=0.002, Ki=0.00001, Kd=0.0001, setpoint=target_rps)

    pwm_r_val = pwm_l_val = 0.18
    PWM_MIN = 0.10
    PWM_MAX = 0.6

    if direction == "forward":
        set_motor(True, True)
    elif direction == "backward":
        set_motor(False, False)
    elif direction == "left":
        set_motor(True, False)
    elif direction == "right":
        set_motor(False, True)
    else:
        print("[DEBUG] move_vehicle: direction không hợp lệ.")
        return

    start = last = time.time()
    try:
        while time.time() - start < duration:
            sleep(0.05)
            now = time.time()
            dt = now - last
            last = now

            rps_r = ((count_r1 + count_r2) / 2) / dt / CPR
            rps_l = ((count_l1 + count_l2) / 2) / dt / CPR
            count_r1 = count_r2 = count_l1 = count_l2 = 0

            pwm_r_val += pid_r.compute(rps_r)
            pwm_l_val += pid_l.compute(rps_l)

            pwm_r_val = max(PWM_MIN, min(pwm_r_val, PWM_MAX))
            pwm_l_val = max(PWM_MIN, min(pwm_l_val, PWM_MAX))

            pwm_r1.value = pwm_r2.value = pwm_r_val
            pwm_l1.value = pwm_l2.value = pwm_l_val

    except Exception as e:
        print(f"[ERROR] Movement error: {e}")
    finally:
        stop_all()

def receive_from_pi5():
    global count_r1, count_r2, count_l1, count_l2
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('', 9999))
    server.listen(1)
    print("[Pi4] 🟢 Chờ Pi5 gửi encoder + limit switch...")

    while True:
        conn, addr = server.accept()
        print(f"[Pi4] ✅ Kết nối Pi5: {addr}")
        with conn:
            while True:
                try:
                    data = conn.recv(1024).decode().strip()
                    if not data: break

                    if "ENC{" in data:
                        enc_block = data.split("ENC{")[1].split("}")[0]
                        enc_data = {kv.split(':')[0]: int(kv.split(':')[1]) for kv in enc_block.split(';') if ':' in kv}
                        count_l1 = enc_data.get('E1', 0)
                        count_l2 = enc_data.get('E2', 0)
                        count_r1 = enc_data.get('E3', 0)
                        count_r2 = enc_data.get('E4', 0)
                        print(f"[ENCODER] L1={count_l1} | L2={count_l2} | R1={count_r1} | R2={count_r2}")

                    if "LIMITS{" in data:
                        sw_block = data.split("LIMITS{")[1].split("}")[0]
                        sw_data = {kv.split(':')[0]: int(kv.split(':')[1]) for kv in sw_block.split(';') if ':' in kv}
                        l1 = sw_data.get('L1', 0)
                        l2 = sw_data.get('L2', 0)
                        l3 = sw_data.get('L3', 0)
                        l4 = sw_data.get('L4', 0)
                        print(f"[LIMIT] L1={l1}, L2={l2}, L3={l3}, L4={l4}")
                        print(f"[DEBUG] receive_from_pi5: running_scan={shared_state.running_scan}")

                        # --- Ngăn không tránh vật khi chưa start_scan ---
                        if not shared_state.running_scan:
                            stop_all()
                            continue

                        if l1 or l2:
                            print("[⚠️] L1 hoặc L2 được nhấn – Tiến 2s")
                            move_vehicle("forward", target_rps=0.1, duration=2.0)
                        elif l3 or l4:
                            print("[⚠️] L3 hoặc L4 được nhấn – Lùi 2s")
                            move_vehicle("backward", target_rps=0.1, duration=2.0)
                except Exception as e:
                    print(f"[ERROR] Lỗi khi nhận từ Pi5: {e}")
                    break

# ==== Nhận lệnh từ Pi5 qua Bluetooth ====
def on_bt_receive(msg):
    msg = msg.strip().lower()
    print("[BT] Nhận lệnh:", msg)
    if msg == "start_scan":
        shared_state.running_scan = True
        print("[BT] ▶️ Cho phép tự hành!")
    elif msg == "stop":
        shared_state.running_scan = False
        stop_all()
        print("[BT] ⏹️ Dừng tự hành!")

# ==== ROS2 Node tự hành ====
class LiDARNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sock = None
        self.connect_to_pi5()
        self.moving = False
        self.last_action_time = self.get_clock().now()

    def connect_to_pi5(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect(("192.168.100.2", 8899))
                print("[Pi4] ✅ Gửi LiDAR đến Pi5")
                break
            except:
                print("[Pi4] 🔁 Chờ Pi5 mở cổng nhận LiDAR...")
                time.sleep(2)

    def scan_callback(self, msg):
        print(f"[DEBUG] scan_callback: running_scan={shared_state.running_scan}")
        # --- Ngăn tự hành khi chưa start_scan ---
        if not shared_state.running_scan:
            stop_all()
            return

        # Autonomous routine bên dưới
        try:
            data = {
                "ranges": list(msg.ranges),
                "angle_min": msg.angle_min,
                "angle_increment": msg.angle_increment
            }
            self.sock.sendall((json.dumps(data) + "\n").encode())
        except:
            self.sock.close()
            self.connect_to_pi5()

        # === Xử lý tự hành tránh vật ===
        size = 40
        scale = 0.1
        map_array = np.full((size, size), '.', dtype=str)
        center = size // 2
        THRESH_CLEAR = 1.5

        valid_points = 0
        free_l, free_c, free_r = [], [], []

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance): continue
            valid_points += 1
            angle = msg.angle_min + i * msg.angle_increment + math.pi
            deg = math.degrees(angle)
            x = int(center + (distance * math.sin(angle)) / scale)
            y = int(center - (distance * math.cos(angle)) / scale)
            if 0 <= x < size and 0 <= y < size:
                map_array[y][x] = '#'
            if 45 <= deg <= 135:
                free_l.append(distance)
            elif -30 <= deg <= 30:
                free_c.append(distance)
            elif -135 <= deg <= -45:
                free_r.append(distance)

        if valid_points == 0:
            print("[AI] ❌ Không có dữ liệu LiDAR – Dừng khẩn")
            stop_all()
            self.moving = False
            return

        mean_l = np.mean([d for d in free_l if d < 3.0]) if free_l else 0
        mean_c = np.mean([d for d in free_c if d < 3.0]) if free_c else 0
        mean_r = np.mean([d for d in free_r if d < 3.0]) if free_r else 0

        os.system('clear')
        for row in map_array[::-1]:
            print(' '.join(row))
        print(f"[DEBUG] Rỗng: Trái={mean_l:.2f} | Giữa={mean_c:.2f} | Phải={mean_r:.2f}")

        now = self.get_clock().now()
        if (now - self.last_action_time).nanoseconds / 1e9 > 0.5 and not self.moving:
            print(f"[DEBUG] scan_callback: Chuẩn bị gọi safe_move (running_scan={shared_state.running_scan})")
            if mean_c > THRESH_CLEAR:
                self.safe_move("forward")
            elif mean_l > mean_r:
                self.safe_move("left")
            elif mean_r > mean_l:
                self.safe_move("right")
            else:
                self.safe_move("backward")
            self.last_action_time = now

    def safe_move(self, direction):
        if self.moving:
            print(f"[DEBUG] safe_move: đã có motor chạy, bỏ qua lệnh mới.")
            return
        def worker():
            print(f"[DEBUG] safe_move(worker): GỌI move_vehicle({direction}), running_scan={shared_state.running_scan}")
            self.moving = True
            move_vehicle(direction, target_rps=0.08, duration=1.0)
            self.moving = False
        threading.Thread(target=worker, daemon=True).start()

def start_lidar_ros():
    return subprocess.Popen(['ros2', 'launch', 'sllidar_ros2', 'sllidar_c1_launch.py'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main():
    lidar_proc = start_lidar_ros()
    threading.Thread(target=receive_from_pi5, daemon=True).start()

    # ==== Khởi động Bluetooth Server ====
    bt_server = BluetoothServer(on_receive=on_bt_receive)
    threading.Thread(target=bt_server.start, daemon=True).start()

    rclpy.init()
    node = LiDARNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[Pi4] ⛔ Dừng hệ thống")
    finally:
        rclpy.shutdown()
        lidar_proc.terminate()
        stop_all()
        bt_server.close()

if __name__ == '__main__':
    main()

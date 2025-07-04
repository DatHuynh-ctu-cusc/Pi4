# Pi4.py (Autonomous Robot - Fixed LiDAR Angles, PID Conflict, and Command Start)
import subprocess, time, rclpy, threading, socket, json, os, math
import RPi.GPIO as GPIO  # type: ignore
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan

# --- Motor GPIO Config ---
MOTORS = {
    'M1': {'PWM': 17, 'IN1': 24, 'IN2': 25},
    'M2': {'PWM': 12, 'IN1': 22, 'IN2': 23},
    'M3': {'PWM': 4,  'IN1': 21, 'IN2': 27},
    'M4': {'PWM': 16, 'IN1': 19, 'IN2': 20},
}

GPIO.setmode(GPIO.BCM)
for motor in MOTORS.values():
    GPIO.setup(motor["IN1"], GPIO.OUT)
    GPIO.setup(motor["IN2"], GPIO.OUT)
    GPIO.setup(motor["PWM"], GPIO.OUT)

PWM_objs = {}
for key, pins in MOTORS.items():
    pwm = GPIO.PWM(pins['PWM'], 100)
    pwm.start(0)
    PWM_objs[key] = pwm

# --- PID Control Class ---
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.last_error = 0

    def compute(self, setpoint, actual, dt):
        error = setpoint - actual
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# --- Motor control helpers ---
def set_motor_direction(key, forward=True):
    pins = MOTORS[key]
    GPIO.output(pins['IN1'], GPIO.HIGH if forward else GPIO.LOW)
    GPIO.output(pins['IN2'], GPIO.LOW if forward else GPIO.HIGH)

def stop_all_motors():
    for pwm in PWM_objs.values():
        pwm.ChangeDutyCycle(0)
    for pins in MOTORS.values():
        GPIO.output(pins['IN1'], GPIO.LOW)
        GPIO.output(pins['IN2'], GPIO.LOW)

def get_motor_direction(key):
    pins = MOTORS[key]
    in1 = GPIO.input(pins['IN1'])
    in2 = GPIO.input(pins['IN2'])
    if in1 == GPIO.HIGH and in2 == GPIO.LOW:
        return 'TIẾN'
    elif in1 == GPIO.LOW and in2 == GPIO.HIGH:
        return 'LÙI'
    elif in1 == GPIO.LOW and in2 == GPIO.LOW:
        return 'DẮNG'
    return '???'

# --- Communication Config ---
HOST_PI5 = '192.168.100.2'
PORT_SEND = 8899
PORT_RECV = 9999
CPR = 200

# --- Global State ---
latest_data = {
    'E1': 0, 'E2': 0, 'E3': 0, 'E4': 0,
    'LIMITS': {'L1': 'OFF', 'L2': 'OFF', 'L3': 'OFF', 'L4': 'OFF'},
    'MOTORS': {'M1': 'OFF', 'M2': 'OFF', 'M3': 'OFF', 'M4': 'OFF'}
}

target_rps = {'M1': 2, 'M2': 2, 'M3': 2, 'M4': 2}
pid_controllers = {m: PID(0.8, 0.01, 0.05) for m in MOTORS}
REVERSE_DURATION = 1.0
reverse_timer = {m: 0 for m in MOTORS}
pid_enabled = True
autonomy_enabled = False

# --- PID Loop ---
def pid_loop():
    global pid_enabled
    last_counts = {'E1': 0, 'E2': 0, 'E3': 0, 'E4': 0}
    last_time = time.time()
    for m in MOTORS:
        set_motor_direction(m, forward=True)

    while True:
        time.sleep(0.05)
        if not pid_enabled:
            continue
        now = time.time()
        dt = now - last_time
        last_time = now

        if any(v == 'ON' for v in latest_data['LIMITS'].values()):
            for m in MOTORS:
                reverse_timer[m] = REVERSE_DURATION

        for i, enc_key in enumerate(['E1', 'E2', 'E3', 'E4']):
            motor_key = f"M{i+1}"
            count_diff = latest_data[enc_key] - last_counts[enc_key]
            last_counts[enc_key] = latest_data[enc_key]
            rps = count_diff / CPR / dt
            output = pid_controllers[motor_key].compute(target_rps[motor_key], rps, dt)
            pwm_value = max(0, min(100, output))

            if reverse_timer[motor_key] > 0:
                set_motor_direction(motor_key, forward=False)
                reverse_timer[motor_key] -= dt
            else:
                set_motor_direction(motor_key, forward=True)

            PWM_objs[motor_key].ChangeDutyCycle(pwm_value)
            latest_data['MOTORS'][motor_key] = 'ON' if pwm_value > 0 else 'OFF'

# --- Pi5 Listener ---
def receive_from_pi5():
    global autonomy_enabled, pid_enabled
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('', PORT_RECV))
    server.listen(1)
    print("[Pi4] 🟢 Sẵn sàng nhận encoder/limit từ Pi5...")

    def display_status():
        os.system('cls' if os.name == 'nt' else 'clear')
        print("========= 📿 ROBOT STATUS =========")
        print(f"{'ENCODER':<10}{'COUNT':<10} |  MOTOR STATUS | DIR")
        print(f"{'-'*50}")
        for i, key in enumerate(['E1', 'E2', 'E3', 'E4']):
            motor_key = f"M{i+1}"
            enc_val = latest_data[key]
            motor_state = latest_data['MOTORS'][motor_key]
            direction = get_motor_direction(motor_key)
            print(f"{key:<10}{enc_val:<10} |  {motor_key}:{motor_state:<3} | {direction:<5}")
        print("\nLIMIT SWITCH:")
        for lx in ['L1', 'L2', 'L3', 'L4']:
            print(f"{lx}: {latest_data['LIMITS'][lx]}", end='  ')
        print("\n===================================")

    while True:
        conn, addr = server.accept()
        print(f"[Pi4] ✅ Kết nối từ Pi5: {addr}")
        with conn:
            while True:
                try:
                    data = conn.recv(1024).decode().strip()
                    if not data:
                        break
                    if data.startswith("CMD:"):
                        cmd = data.split("CMD:")[1].strip().upper()
                        if cmd == "START":
                            autonomy_enabled = True
                            pid_enabled = False
                            print("[Pi4] 💡 Tự hành Bật")
                        elif cmd == "STOP":
                            autonomy_enabled = False
                            pid_enabled = True
                            print("[Pi4] ❌ Tự hành Tắt")
                    elif data.startswith("ENC{"):
                        enc_block = data.split("};")[0][4:]
                        for part in enc_block.split(';'):
                            if ':' in part:
                                k, v = part.split(':')
                                latest_data[k.strip()] = int(v.strip())
                        if "LIMITS{" in data:
                            lim_block = data.split("LIMITS{")[-1].split('}')[0]
                            for part in lim_block.split(';'):
                                if ':' in part:
                                    k, v = part.split(':')
                                    latest_data['LIMITS'][k.strip()] = 'ON' if v.strip() == '1' else 'OFF'
                        display_status()
                except Exception as e:
                    print(f"[Pi4] ❌ Lỗi khi nhận dữ liệu: {e}")
                    break

# --- ROS2 LiDAR Node ---
class LiDARNode(Node):
    def __init__(self):
        super().__init__('lidar_sender')
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.sock = None
        self.connect_to_pi5()
        self.last_action_time = self.get_clock().now()
        self.moving = False
        self.last_direction = None

    def connect_to_pi5(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((HOST_PI5, PORT_SEND))
                print("[Pi4] ✅ Đã kết nối gửi LiDAR tới Pi5")
                break
            except:
                print("[Pi4] 🔁 Đang chờ Pi5...")
                time.sleep(2)

    def safe_move(self, direction):
        print(f"[Pi4] 🚗 Di chuyển: {direction.upper()}")
        self.moving = True
        if direction == "forward":
            for m in MOTORS:
                set_motor_direction(m, forward=True)
        elif direction == "backward":
            for m in MOTORS:
                set_motor_direction(m, forward=False)
        elif direction == "left":
            set_motor_direction("M1", forward=False)
            set_motor_direction("M2", forward=False)
            set_motor_direction("M3", forward=True)
            set_motor_direction("M4", forward=True)
        elif direction == "right":
            set_motor_direction("M1", forward=True)
            set_motor_direction("M2", forward=True)
            set_motor_direction("M3", forward=False)
            set_motor_direction("M4", forward=False)
        time.sleep(0.2)
        self.moving = False

    def scan_callback(self, msg):
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

        if not autonomy_enabled:
            return

        THRESH_CLEAR = 1.5
        free_l, free_c, free_r = [], [], []

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            deg = math.degrees(angle)
            if -130 <= deg <= -50:
                free_l.append(distance)
            elif -25 <= deg <= 25:
                free_c.append(distance)
            elif 50 <= deg <= 130:
                free_r.append(distance)

        def filtered_mean(lst):
            arr = [d for d in lst if 0.1 < d < 3.5]
            return np.mean(arr) if arr else 0

        mean_l = filtered_mean(free_l)
        mean_c = filtered_mean(free_c)
        mean_r = filtered_mean(free_r)

        print(f"[DEBUG] Trái={mean_l:.2f} | Giữa={mean_c:.2f} | Phải={mean_r:.2f}")

        now = self.get_clock().now()
        if (now - self.last_action_time).nanoseconds / 1e9 > 0.5 and not self.moving:
            if mean_c > THRESH_CLEAR:
                self.safe_move("forward")
            elif mean_l > THRESH_CLEAR or mean_r > THRESH_CLEAR:
                if mean_l > mean_r:
                    if self.last_direction != "left":
                        self.safe_move("left")
                        self.last_direction = "left"
                else:
                    if self.last_direction != "right":
                        self.safe_move("right")
                        self.last_direction = "right"
            else:
                self.safe_move("backward")
                time.sleep(0.4)
                self.safe_move("left")
                time.sleep(0.4)
                self.safe_move("left")
                self.last_direction = None
            self.last_action_time = now

# --- Start LiDAR driver ---
def start_lidar_ros():
    return subprocess.Popen(
        ['ros2', 'launch', 'sllidar_ros2', 'sllidar_c1_launch.py'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# --- Main ---
def main():
    lidar_proc = start_lidar_ros()
    threading.Thread(target=receive_from_pi5, daemon=True).start()
    threading.Thread(target=pid_loop, daemon=True).start()
    rclpy.init()
    node = LiDARNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[Pi4] ⛔ Dừng hệ thống.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        lidar_proc.terminate()
        stop_all_motors()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
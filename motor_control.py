from gpiozero import Motor, PWMOutputDevice
from time import sleep, time

# ==== GPIO Cấu hình động cơ ====
motor1 = Motor(forward=24, backward=25, pwm=True)   # Trái trước (E1)
motor1_pwm = PWMOutputDevice(17)

motor2 = Motor(forward=22, backward=23, pwm=True)   # Trái sau (E2)
motor2_pwm = PWMOutputDevice(12)

motor3 = Motor(forward=21, backward=27, pwm=True)   # Phải trước (E3)
motor3_pwm = PWMOutputDevice(4)

motor4 = Motor(forward=19, backward=20, pwm=True)   # Phải sau (E4)
motor4_pwm = PWMOutputDevice(16)

pwm = {
    "E1": motor1_pwm,
    "E2": motor2_pwm,
    "E3": motor3_pwm,
    "E4": motor4_pwm,
}

# ==== Bộ điều khiển PID ====
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

# ==== Tham số PID riêng cho từng hành động ====
PID_PARAMS = {
    "forward":  {"Kp": 0.0015, "Ki": 0.000005, "Kd": 0.00005},
    "backward": {"Kp": 0.0018, "Ki": 0.00001,  "Kd": 0.00008},
    "left":  {"Kp": 0.0013, "Ki": 0.000004, "Kd": 0.00006},
    "right": {"Kp": 0.0013, "Ki": 0.000004, "Kd": 0.00006},
}

# ==== PWM khởi tạo riêng theo hướng ====
PWM_INIT = {
    "forward":  {"E1": 0.12, "E2": 0.12, "E3": 0.12, "E4": 0.12},
    "backward": {"E1": 0.135, "E2": 0.135, "E3": 0.13, "E4": 0.13},
    "left":  {"E1": 0.11, "E2": 0.11, "E3": 0.11, "E4": 0.11},
    "right": {"E1": 0.11, "E2": 0.11, "E3": 0.11, "E4": 0.11},
}

PWM_MIN = 0.08
PWM_MAX = 0.4

# ==== Dừng tất cả động cơ ====
def stop_all():
    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()
    for p in pwm.values():
        p.value = 0

# ==== Điều khiển chiều quay ====
def set_motor(direction):
    if direction == "forward":
        motor1.forward(); motor2.forward(); motor3.forward(); motor4.forward()
    elif direction == "backward":
        motor1.backward(); motor2.backward(); motor3.backward(); motor4.backward()
    elif direction == "left":
        motor1.forward(); motor2.forward(); motor3.backward(); motor4.backward()
    elif direction == "right":
        motor1.backward(); motor2.backward(); motor3.forward(); motor4.forward()

# ==== Hàm di chuyển PID riêng từng hướng ====
def move_vehicle(direction="forward", target_rps=0.1, duration=1.0, counts=None, CPR=200):
    if not counts:
        print("[WARNING] Thiếu encoder. Không thể PID.")
        return

    for key in counts:
        counts[key] = 0

    # Chọn hệ số PID cho hướng
    pid_cfg = PID_PARAMS.get(direction, PID_PARAMS["forward"])
    pids = {
        "E1": PID(pid_cfg["Kp"], pid_cfg["Ki"], pid_cfg["Kd"], target_rps),
        "E2": PID(pid_cfg["Kp"], pid_cfg["Ki"], pid_cfg["Kd"], target_rps),
        "E3": PID(pid_cfg["Kp"], pid_cfg["Ki"], pid_cfg["Kd"], target_rps),
        "E4": PID(pid_cfg["Kp"], pid_cfg["Ki"], pid_cfg["Kd"], target_rps),
    }

    # Khởi tạo PWM phù hợp với hướng
    pwm_values = PWM_INIT.get(direction, PWM_INIT["forward"]).copy()

    set_motor(direction)
    start = last = time()

    try:
        while time() - start < duration:
            sleep(0.05)
            now = time()
            dt = now - last
            last = now
            if dt == 0:
                continue

            # Tính RPS từng bánh
            rps = {}
            for key in counts:
                rps[key] = counts[key] / dt / CPR
                counts[key] = 0

            # PID cập nhật PWM
            for key in ["E1", "E2", "E3", "E4"]:
                pwm_values[key] += pids[key].compute(rps.get(key, 0))
                pwm_values[key] = max(PWM_MIN, min(pwm_values[key], PWM_MAX))
                pwm[key].value = pwm_values[key]

    except Exception as e:
        print(f"[ERROR] PID move failed: {e}")
    finally:
        stop_all()

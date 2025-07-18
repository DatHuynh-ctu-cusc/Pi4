from gpiozero import Motor, PWMOutputDevice
from time import sleep, time
from shared_state import shared_counts

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

# ==== PID_PARAMS cho từng động cơ ====
PID_PARAMS = {
    "forward": {
        "E1": {"Kp": 0.0015, "Ki": 0.000005, "Kd": 0.00005},
        "E2": {"Kp": 0.0015, "Ki": 0.000005, "Kd": 0.00005},
        "E3": {"Kp": 0.0015, "Ki": 0.000005, "Kd": 0.00005},
        "E4": {"Kp": 0.0015, "Ki": 0.000005, "Kd": 0.00005},
    },
    "backward": {
        "E1": {"Kp": 0.0018, "Ki": 0.00001,  "Kd": 0.00008},
        "E2": {"Kp": 0.0019, "Ki": 0.00001,  "Kd": 0.00008},
        "E3": {"Kp": 0.0017, "Ki": 0.00001,  "Kd": 0.00007},
        "E4": {"Kp": 0.0018, "Ki": 0.00001,  "Kd": 0.00007},
    },
    "left": {
        "E1": {"Kp": 0.0019, "Ki": 0.000006, "Kd": 0.00008},
        "E2": {"Kp": 0.0019, "Ki": 0.000006, "Kd": 0.00008},
        "E3": {"Kp": 0.0010, "Ki": 0.000003, "Kd": 0.00005},  
        "E4": {"Kp": 0.0010, "Ki": 0.000003, "Kd": 0.00005},
    },
    "right": {
        "E1": {"Kp": 0.0012, "Ki": 0.000004, "Kd": 0.00005},
        "E2": {"Kp": 0.0012, "Ki": 0.000004, "Kd": 0.00005},
        "E3": {"Kp": 0.0012, "Ki": 0.000004, "Kd": 0.00006},
        "E4": {"Kp": 0.0013, "Ki": 0.000004, "Kd": 0.00006},
    },
}

# ==== PWM khởi tạo riêng theo hướng (nhẹ) ====
PWM_INIT = {
    "forward":  {"E1": 0.09, "E2": 0.09, "E3": 0.09, "E4": 0.09},
    "backward": {"E1": 0.10, "E2": 0.10, "E3": 0.10, "E4": 0.10},
    "left":     {"E1": 0.105, "E2": 0.105, "E3": 0.085, "E4": 0.085}, 
    "right":    {"E1": 0.095, "E2": 0.095, "E3": 0.095, "E4": 0.095},
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
        motor3.forward(); motor4.forward(); motor1.backward(); motor2.backward()
    elif direction == "right":
        motor3.backward(); motor4.backward(); motor1.forward(); motor2.forward()

# ==== Di chuyển với tốc độ mặc định 10 cm/s (0.53 RPS) ====
def move_vehicle(direction="forward", target_rps=0.53, duration=1.0, counts=shared_counts, CPR=171):
    if not counts:
        print("[WARNING] Thiếu encoder. Không thể PID.")
        return

    for key in counts:
        counts[key] = 0

    pid_cfg = PID_PARAMS.get(direction, PID_PARAMS["forward"])
    pids = {
        key: PID(cfg["Kp"], cfg["Ki"], cfg["Kd"], 0.0)  # ramp từ 0
        for key, cfg in pid_cfg.items()
    }
    for pid in pids.values():
        pid.last_error = 0
        pid.integral = 0
    pwm_values = {key: PWM_MIN for key in ["E1", "E2", "E3", "E4"]}
    pwm_target = PWM_INIT.get(direction, PWM_INIT["forward"]).copy()

    set_motor(direction)
    start = last = time()

    try:
        while True:
            now = time()
            dt = now - last
            last = now
            elapsed = now - start

            if elapsed > duration:
                break

            sleep(0.05)
            if dt == 0:
                continue

            # ==== Ramp RPS trong 2 giây ====
            ramp_up_ratio = min(elapsed / 2.0, 1.0)
            current_rps = target_rps * ramp_up_ratio
            for pid in pids.values():
                pid.setpoint = current_rps

            # ==== Tính RPS ====
            rps = {}
            for key in counts:
                rps[key] = counts[key] / dt / CPR
                counts[key] = 0

            # ==== PWM ramp và PID ====
            for key in ["E1", "E2", "E3", "E4"]:
                if pwm_values[key] < pwm_target[key]:
                    pwm_values[key] += 0.002  # ramp mượt
                else:
                    pwm_values[key] += pids[key].compute(rps.get(key, 0))

                pwm_values[key] = max(PWM_MIN, min(pwm_values[key], PWM_MAX))
                pwm[key].value = pwm_values[key]

    except Exception as e:
        print(f"[ERROR] PID move failed: {e}")
    finally:
        stop_all()

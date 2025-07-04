# motor_control.py
from gpiozero import Motor, PWMOutputDevice
from time import sleep, time

# ==== GPIO Cấu hình động cơ ====
motor1 = Motor(forward=24, backward=25, pwm=True)
motor1_pwm = PWMOutputDevice(17)

motor2 = Motor(forward=22, backward=23, pwm=True)
motor2_pwm = PWMOutputDevice(12)

motor3 = Motor(forward=21, backward=27, pwm=True)
motor3_pwm = PWMOutputDevice(4)

motor4 = Motor(forward=19, backward=20, pwm=True)
motor4_pwm = PWMOutputDevice(16)

# PWM mapping
pwm_r1 = motor3_pwm
pwm_r2 = motor4_pwm
pwm_l1 = motor1_pwm
pwm_l2 = motor2_pwm

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
        self.integral = max(min(self.integral, 100), -100)  # Giới hạn tích phân
        derivative = error - self.last_error
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# ==== Dừng tất cả động cơ ====
def stop_all():
    motor1.stop()
    motor2.stop()
    motor3.stop()
    motor4.stop()
    pwm_l1.value = pwm_l2.value = pwm_r1.value = pwm_r2.value = 0

# ==== Điều khiển chiều quay động cơ ====
def set_motor(forward_l, forward_r):
    if forward_l:
        motor1.forward()
        motor2.forward()
    else:
        motor1.backward()
        motor2.backward()

    if forward_r:
        motor3.forward()
        motor4.forward()
    else:
        motor3.backward()
        motor4.backward()

# ==== Hàm di chuyển với PID ====
def move_vehicle(direction="forward", target_rps=0.1, duration=1.0, counts=None, CPR=200):
    if counts:
        counts["E1"] = counts["E2"] = counts["E3"] = counts["E4"] = 0

    pid_r = PID(0.0015, 0.000005, 0.00005, target_rps)
    pid_l = PID(0.0015, 0.000005, 0.00005, target_rps)

    pwm_r_val = pwm_l_val = 0.12
    PWM_MIN = 0.08
    PWM_MAX = 0.4

    # Chọn chiều di chuyển
    if direction == "forward":
        set_motor(True, True)
    elif direction == "backward":
        set_motor(False, False)
    elif direction == "left":
        set_motor(True, False)
    elif direction == "right":
        set_motor(False, True)
    else:
        return

    start = last = time()
    try:
        while time() - start < duration:
            sleep(0.05)
            now = time()
            dt = now - last
            last = now

            if dt == 0:
                continue

            rps_r = ((counts["E3"] + counts["E4"]) / 2) / dt / CPR
            rps_l = ((counts["E1"] + counts["E2"]) / 2) / dt / CPR
            counts["E1"] = counts["E2"] = counts["E3"] = counts["E4"] = 0

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
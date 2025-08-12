# path_node.py
import time
from motor_control import move_vehicle, stop_all
import shared_state

def parse_command(command):
    """Chuyển chuỗi lệnh thành (hướng, giá trị)."""
    parts = command.strip().lower().split()
    if len(parts) == 2:
        direction, val = parts
        try:
            value = float(val)
            return direction, value
        except ValueError:
            pass
    return None, None

def execute_path(path_string, counts):
    if shared_state.running_scan:
        print("[PATH] 🚫 Đang quét bản đồ, bỏ qua lệnh path.")
        return

    shared_state.running_path = True
    print("[PATH] 🧭 Bắt đầu chạy theo đường vẽ...")

    commands = [cmd.strip() for cmd in path_string.split(";") if cmd.strip()]

    for raw_cmd in commands:
        direction, value = parse_command(raw_cmd)
        if direction is None:
            print(f"[PATH] ⚠️ Bỏ qua lệnh không hợp lệ: {raw_cmd}")
            continue

        print(f"[PATH] ▶️ {direction} {value}")

        if direction in ["forward", "backward"]:
            # Hệ số: 3.33 giây / mét
            duration = value * 3.33
            move_vehicle(direction, 0.25, duration, counts)
        elif direction == "left":
            # Hệ số: 0.0274 giây / độ
            duration = value * 0.0274
            move_vehicle(direction, 0.25, duration, counts)
        elif direction == "right":
            # Hệ số: 0.0317 giây / độ
            duration = value * 0.0317
            move_vehicle(direction, 0.25, duration, counts)
        else:
            print(f"[PATH] ❌ Lệnh không hỗ trợ: {direction}")

        time.sleep(0.2)

    shared_state.running_path = False

    stop_all()
    shared_state.running_path = False
    print("[PATH] ✅ Hoàn tất lộ trình.")

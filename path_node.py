import time
from motor_control import move_vehicle, stop_all
import shared_state

def parse_command(command):
    """Chuyển chuỗi thành (direction, value)."""
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
    """
    Thực hiện đường đi được gửi từ Pi5, dạng: 'forward 0.5; right 90; forward 1.2'
    """
    commands = path_string.split(";")
    for raw_cmd in commands:
        direction, value = parse_command(raw_cmd)
        if direction is None:
            print(f"[PATH] ⚠️ Bỏ qua lệnh không hợp lệ: {raw_cmd}")
            continue

        print(f"[PATH] 🚗 Đang thực hiện: {direction} {value}")
        if direction in ["forward", "backward"]:
            move_vehicle(direction, 0.25, value, counts)
        elif direction in ["left", "right"]:
            # giả sử value là góc độ, ta chuyển sang thời gian (cần sửa nếu dùng encoder)
            duration = value / 90.0  # ví dụ: 90 độ = 1s
            move_vehicle(direction, 0.25, duration, counts)
        else:
            print(f"[PATH] ❌ Lệnh không hỗ trợ: {direction}")

        time.sleep(0.2)

    stop_all()
    print("[PATH] ✅ Đã hoàn thành đường đi.")


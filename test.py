from motor_control import move_vehicle, stop_all
import time

# Bộ đếm giả để test (nếu chưa tích hợp encoder thực tế)
counts = {
    "E1": 0, "E2": 0, "E3": 0, "E4": 0
}

def print_menu():
    print("\n=== TEST MOTOR ===")
    print("Các lệnh:")
    print("  forward   → đi tới")
    print("  backward  → lùi")
    print("  left      → xoay trái")
    print("  right     → xoay phải")
    print("  stop      → dừng lại")
    print("  exit      → thoát")
    print("=======================")

if __name__ == "__main__":
    print_menu()
    while True:
        cmd = input("Nhập lệnh điều khiển: ").strip().lower()

        if cmd == "exit":
            stop_all()
            print("[EXIT] Kết thúc test.")
            break

        elif cmd == "stop":
            stop_all()
            print("[STOP] Đã dừng toàn bộ động cơ.")

        elif cmd in ("forward", "backward", "left", "right"):
            print(f"[RUN] Đang chạy lệnh {cmd} trong 2 giây...")
            move_vehicle(direction=cmd, target_rps=0.2, duration=2.0, counts=counts)

        else:
            print("[⚠️] Lệnh không hợp lệ!")
            print_menu()

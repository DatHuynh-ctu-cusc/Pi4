# main.py – Pi4
import subprocess
import threading
import rclpy
from autonomous_node import LiDARNode
from receiver import start_receiver
from motor_control import move_vehicle, stop_all
from bluetooth_rfcomm_server import BluetoothServer
from path_node import execute_path
import shared_state

def start_lidar_ros():
    print("[Pi4] 🚀 Khởi động ROS2 LiDAR...")
    return subprocess.Popen(
        ['ros2', 'launch', 'sllidar_ros2', 'sllidar_c1_launch.py'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

def on_bt_receive(msg):
    print("[BT] Lệnh từ Pi5:", msg)

    # Tách từng dòng lệnh
    lines = msg.strip().splitlines()

    # Nếu chỉ có 1 dòng lệnh
    if len(lines) == 1:
        cmd = lines[0].strip().lower()

        if cmd == "start_scan":
            shared_state.running_path = False
            shared_state.running_scan = True
            print("[BT] ▶️ Bắt đầu chế độ quét bản đồ")

        elif cmd == "stop":
            shared_state.running_scan = False
            shared_state.running_path = False
            stop_all()
            print("[BT] 🛑 Dừng robot")

        elif cmd.startswith("path:"):
            shared_state.running_scan = False
            shared_state.running_path = True
            path_data = cmd[5:].strip()
            execute_path(path_data, shared_counts)
            shared_state.running_path = False

        else:
            print("[BT] ❌ Lệnh không hợp lệ:", cmd)

    # Nếu là chuỗi nhiều dòng → ghép lại thành path
    elif len(lines) > 1:
        path_data = "; ".join(line.strip() for line in lines if line.strip())
        shared_state.running_scan = False
        shared_state.running_path = True
        print("[BT] 🧭 Chạy theo đường đi từ nhiều dòng...")
        execute_path(path_data, shared_counts)
        shared_state.running_path = False


def main():
    global shared_counts
    shared_counts = {"E1": 0, "E2": 0, "E3": 0, "E4": 0}

    print("[Pi4] Nhận encoder từ Pi5...")
    threading.Thread(target=start_receiver, args=(shared_counts,), daemon=True).start()

    bt_server = BluetoothServer(on_receive=on_bt_receive)
    threading.Thread(target=bt_server.start, daemon=True).start()

    lidar_proc = start_lidar_ros()

    rclpy.init()
    node = LiDARNode(shared_counts)
    print("[Pi4] ROS2 Node LiDAR đang hoạt động...")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[Pi4] ⛔ Dừng hệ thống")
    finally:
        rclpy.shutdown()
        stop_all()
        lidar_proc.terminate()
        bt_server.close()
        print("[Pi4] 🛑 Dừng toàn bộ hệ thống")

if __name__ == '__main__':
    main()

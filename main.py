# main.py – Pi4
import subprocess
import threading
import rclpy
from autonomous_node import LiDARNode
from receiver import start_receiver
from motor_control import move_vehicle, stop_all
from bluetooth_rfcomm_server import BluetoothServer      # <<< THÊM DÒNG NÀY
import shared_state


def start_lidar_ros():
    print("[Pi4] 🚀 Đang khởi động ROS2 LiDAR...")
    return subprocess.Popen(
        ['ros2', 'launch', 'sllidar_ros2', 'sllidar_c1_launch.py'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

def on_bt_receive(msg):
    print("[BT] Lệnh nhận từ Pi5:", msg)
    cmd = msg.strip().lower()
    if cmd == "forward":
        if shared_state.running_scan:
            move_vehicle("forward", 5)
        else:
            print("[BT] Chưa ở chế độ quét, lệnh bị bỏ qua.")
    elif cmd == "left":
        if shared_state.running_scan:
            move_vehicle("left", 5)
        else:
            print("[BT] Chưa ở chế độ quét, lệnh bị bỏ qua.")
    elif cmd == "right":
        if shared_state.running_scan:
            move_vehicle("right", 5)
        else:
            print("[BT] Chưa ở chế độ quét, lệnh bị bỏ qua.")
    elif cmd == "backward":
        if shared_state.running_scan:
            move_vehicle("backward", 5)
        else:
            print("[BT] Chưa ở chế độ quét, lệnh bị bỏ qua.")
    elif cmd == "stop":
        stop_all()
        shared_state.running_scan = False
        print("[BT] Đã dừng routine quét map, xe dừng.")
    elif cmd == "start_scan":
        shared_state.running_scan = True
        print("[BT] Đã nhận lệnh bắt đầu quét bản đồ! Xe sẽ bắt đầu chạy.")
    else:
        print("[BT] Lệnh không hợp lệ:", cmd)




def main():
    counts = {"E1": 0, "E2": 0, "E3": 0, "E4": 0}

    print("[Pi4] 🏁 Bắt đầu nhận Encoder & Limit Switch từ Pi5...")
    start_receiver(counts)

    # <<< KHỞI ĐỘNG BLUETOOTH SERVER (chạy ở luồng riêng)
    bt_server = BluetoothServer(on_receive=on_bt_receive)
    threading.Thread(target=bt_server.start, daemon=True).start()

    lidar_proc = start_lidar_ros()

    rclpy.init()
    node = LiDARNode(counts)
    print("[Pi4] ✅ ROS2 Node LiDAR đã chạy – Đang thu thập dữ liệu...")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[Pi4] ⛔ Đã dừng hệ thống (Ctrl+C)")
    finally:
        rclpy.shutdown()
        stop_all()
        lidar_proc.terminate()
        bt_server.close()   # <<< ĐÓNG BLUETOOTH KHI KẾT THÚC
        print("[Pi4] 🛑 Đã dừng toàn bộ LiDAR, Động cơ & Bluetooth")

if __name__ == '__main__':
    main()

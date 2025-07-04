# main.py – Pi4
import subprocess
import threading
import rclpy
from autonomous_node import LiDARNode
from receiver import start_receiver
from motor_control import move_vehicle, stop_all

def start_lidar_ros():
    print("[Pi4] 🚀 Đang khởi động ROS2 LiDAR...")
    return subprocess.Popen(
        ['ros2', 'launch', 'sllidar_ros2', 'sllidar_c1_launch.py'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

def main():
    counts = {"E1": 0, "E2": 0, "E3": 0, "E4": 0}

    print("[Pi4] 🏁 Bắt đầu nhận Encoder & Limit Switch từ Pi5...")
    start_receiver(counts)

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
        print("[Pi4] 🛑 Đã dừng toàn bộ LiDAR & Động cơ")

if __name__ == '__main__':
    main()
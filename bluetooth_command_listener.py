# bluetooth_command_listener.py
import serial
import threading
from motor_control import move_vehicle, stop_all

def start_bluetooth_listener(shared_counts):
    def listen():
        try:
            ser = serial.Serial('/dev/rfcomm0', 9600, timeout=1)
            print("[Pi4] ✅ Bluetooth sẵn sàng – chờ lệnh Pi5...")
            while True:
                data = ser.readline().decode().strip()
                if data:
                    print(f"[Pi4] 📥 Lệnh nhận: {data}")
                    if data == "forward":
                        move_vehicle("forward", 0.1, 1.0, shared_counts)
                    elif data == "left":
                        move_vehicle("left", 0.1, 0.8, shared_counts)
                    elif data == "right":
                        move_vehicle("right", 0.1, 0.8, shared_counts)
                    elif data == "backward":
                        move_vehicle("backward", 0.1, 1.0, shared_counts)
                    elif data == "stop":
                        stop_all()
        except Exception as e:
            print(f"[Pi4] ❌ Lỗi Bluetooth: {e}")

    threading.Thread(target=listen, daemon=True).start()

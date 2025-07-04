import socket
import threading
from motor_control import move_vehicle
import time

# Biến toàn cục: Pi4 dùng để dừng tự hành khi công tắc được nhấn
limit_active = False

def start_receiver(shared_counts):
    def run():
        global limit_active
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('', 9999))
        server.listen(1)
        print("[Pi4] 🟢 Chờ Pi5 gửi encoder + limit switch...")

        while True:
            conn, addr = server.accept()
            print(f"[Pi4] ✅ Kết nối Pi5: {addr}")
            buffer = ""

            try:
                while True:
                    chunk = conn.recv(1024).decode()
                    if not chunk:
                        break
                    buffer += chunk

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue

                        # === Nhận ENCODER ===
                        if "ENC{" in line:
                            enc_block = line.split("ENC{")[1].split("}")[0]
                            enc_data = {
                                kv.split(':')[0]: int(kv.split(':')[1])
                                for kv in enc_block.split(';') if ':' in kv
                            }
                            shared_counts.update(enc_data)
                            print(f"[ENCODER] {shared_counts}")

                        # === Nhận LIMIT SWITCH ===
                        if "LIMITS{" in line:
                            sw_block = line.split("LIMITS{")[1].split("}")[0]
                            sw_data = {
                                kv.split(':')[0]: int(kv.split(':')[1])
                                for kv in sw_block.split(';') if ':' in kv
                            }
                            print(f"[LIMIT] {sw_data}")

                            # Nếu có bất kỳ công tắc nào được nhấn
                            if sw_data.get('L1') or sw_data.get('L2'):
                                if not limit_active:
                                    limit_active = True
                                    print("[⚠️] L1/L2 được nhấn – Tiến 1s → Xoay phải 1s")
                                    move_vehicle("forward", 0.1, 1.0, shared_counts)
                                    time.sleep(0.2)
                                    move_vehicle("right", 0.1, 1.0, shared_counts)

                            elif sw_data.get('L3') or sw_data.get('L4'):
                                if not limit_active:
                                    limit_active = True
                                    print("[⚠️] L3/L4 được nhấn – Lùi 1s → Xoay trái 1s")
                                    move_vehicle("backward", 0.1, 1.0, shared_counts)
                                    time.sleep(0.2)
                                    move_vehicle("left", 0.1, 1.0, shared_counts)

                            else:
                                if limit_active:
                                    print("[✔️] Công tắc đã nhả – Cho phép tự hành lại")
                                limit_active = False

            except Exception as e:
                print(f"[ERROR] Lỗi nhận Pi5: {e}")
            finally:
                conn.close()
                print("[Pi4] 🔌 Mất kết nối Pi5 – Chờ kết nối lại...")

    threading.Thread(target=run, daemon=True).start()

import socket
import threading
from motor_control import move_vehicle, stop_all
import time
import shared_state  # Biến chia sẻ trạng thái tự hành và tránh va

def start_receiver(shared_counts):
    def run():
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('', 9999))
        server.listen(1)
        print("[Pi4] 🟢 Chờ Pi5 gửi encoder + limit switch...")

        already_stopped_due_to_no_scan = False
        prev_enc_data = {}
        prev_limit_data = {}

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
                            if enc_data != prev_enc_data:
                                shared_counts.update(enc_data)
                                print(f"[ENCODER] {shared_counts}")
                                prev_enc_data = enc_data.copy()

                        # === Nhận LIMIT SWITCH ===
                        elif "LIMITS{" in line:
                            sw_block = line.split("LIMITS{")[1].split("}")[0]
                            sw_data = {
                                kv.split(':')[0]: int(kv.split(':')[1])
                                for kv in sw_block.split(';') if ':' in kv
                            }

                            if sw_data != prev_limit_data:
                                print(f"[LIMIT] {sw_data}")
                                prev_limit_data = sw_data.copy()

                            l1 = sw_data.get('L1', 0)
                            l2 = sw_data.get('L2', 0)
                            l3 = sw_data.get('L3', 0)
                            l4 = sw_data.get('L4', 0)

                            if not shared_state.running_scan:
                                if any([l1, l2, l3, l4]):
                                    if not already_stopped_due_to_no_scan:
                                        print("[LIMIT] 🚫 Công tắc bị nhấn nhưng chưa start_scan => STOP motor!")
                                        stop_all()
                                        already_stopped_due_to_no_scan = True
                                else:
                                    already_stopped_due_to_no_scan = False
                                continue

                            # Dừng khẩn cấp khi va chạm đối nghịch
                            dangerous = (
                                (l1 and l3) or (l2 and l3) or
                                (l1 and l4) or (l2 and l4) or
                                (l1 and l2 and l3 and l4)
                            )
                            if dangerous:
                                print("[❌] ⚠️ Va chạm đối nghịch hoặc toàn bộ – DỪNG KHẨN CẤP")
                                stop_all()
                                shared_state.limit_active = True
                                continue

                            if any([l1, l2, l3, l4]):
                                if not shared_state.limit_active:
                                    shared_state.limit_active = True
                                    print("[⚠️] Công tắc va chạm được nhấn – DỪNG và CHỜ 1 GIÂY...")
                                    stop_all()
                                    time.sleep(1.0)

                                    # Kiểm tra lại sau 1 giây
                                    if not any([l1, l2, l3, l4]):
                                        print("[✔️] Công tắc đã nhả sau 1s – Không cần xử lý")
                                        shared_state.limit_active = False
                                        continue

                                    # === Hành vi tránh ===
                                    if l1 or l2:
                                        print("[⚠️] L1/L2 giữ – Tiến 0.1s → Xoay phải 0.1s")
                                        move_vehicle("forward", 0.1, 1.0, shared_counts)
                                        time.sleep(0.2)
                                        move_vehicle("right", 0.1, 1.0, shared_counts)
                                        shared_state.limit_active = False

                                    elif l3 or l4:
                                        print("[⚠️] L3/L4 giữ – Lùi 0.1s và chờ xử lý LiDAR để xoay")
                                        move_vehicle("backward", 0.1, 1.0, shared_counts)
                                        shared_state.escape_required = True

                            else:
                                if shared_state.limit_active and not shared_state.escape_required:
                                    print("[✔️] Công tắc đã nhả – Cho phép tự hành lại")
                                    shared_state.limit_active = False

            except Exception as e:
                print(f"[ERROR] Lỗi nhận Pi5: {e}")
            finally:
                conn.close()
                print("[Pi4] 🔌 Mất kết nối Pi5 – Chờ kết nối lại...")

    threading.Thread(target=run, daemon=True).start()

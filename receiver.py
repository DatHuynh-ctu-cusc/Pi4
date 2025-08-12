import socket
import threading
from motor_control import move_vehicle, stop_all
import time
import shared_state  # Biến chia sẻ trạng thái tự hành và tránh va

def is_dangerous_pair(l1, l2, l3, l4):
    return (l1 and l3) or (l1 and l4) or (l2 and l3) or (l2 and l4)

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
        in_danger = False

        # Trạng thái công tắc lần trước
        last_pressed_state = {'L1': 0, 'L2': 0, 'L3': 0, 'L4': 0}

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
                        if "LIMITS{" in line:
                            sw_block = line.split("LIMITS{")[1].split("}")[0]
                            sw_data = {
                                kv.split(':')[0]: int(kv.split(':')[1])
                                for kv in sw_block.split(';') if ':' in kv
                            }

                            l1 = sw_data.get('L1', 0)
                            l2 = sw_data.get('L2', 0)
                            l3 = sw_data.get('L3', 0)
                            l4 = sw_data.get('L4', 0)

                            if sw_data != prev_limit_data:
                                print(f"[LIMIT] {sw_data}")
                                prev_limit_data = sw_data.copy()

                            # Nếu chưa bật chế độ tự hành
                            if not shared_state.running_scan:
                                if any([l1, l2, l3, l4]):
                                    if not already_stopped_due_to_no_scan:
                                        print("[LIMIT] 🚫 Công tắc bị nhấn nhưng chưa start_scan => STOP motor!")
                                        stop_all()
                                        already_stopped_due_to_no_scan = True
                                else:
                                    already_stopped_due_to_no_scan = False
                                continue

                            # === Kiểm tra công tắc mới nhấn ===
                            new_press = {
                                'L1': l1 and not last_pressed_state['L1'],
                                'L2': l2 and not last_pressed_state['L2'],
                                'L3': l3 and not last_pressed_state['L3'],
                                'L4': l4 and not last_pressed_state['L4'],
                            }

                            # === Va chạm đối nghịch ===
                            if is_dangerous_pair(l1, l2, l3, l4):
                                if not in_danger:
                                    print("[❌] ⚠️ Va chạm đối nghịch – DỪNG KHẨN CẤP")
                                    stop_all()
                                    shared_state.limit_active = True
                                    in_danger = True
                                continue
                            elif in_danger and not is_dangerous_pair(l1, l2, l3, l4):
                                print("[✅] Công tắc đối nghịch đã nhả – Cho phép tiếp tục")
                                shared_state.limit_active = False
                                in_danger = False
                                continue

                            # === Xử lý L1/L2 ===
                            if new_press['L1'] or new_press['L2']:
                                print("[LIMIT] ⏱ L1/L2 – Dừng và đợi công tắc nhả trong 1s...")
                                shared_state.limit_active = True
                                stop_all()

                                # Đợi từng bước nhỏ trong 1s, nếu nhả thì thoát sớm
                                waited = 0
                                while waited < 1.0:
                                    if not (l1 or l2):
                                        print("[LIMIT] ✅ Công tắc đã nhả trong thời gian chờ – quay lại tự hành")
                                        shared_state.limit_active = False
                                        break
                                    time.sleep(0.1)
                                    waited += 0.1
                                else:
                                    print("[LIMIT] ❌ Công tắc vẫn giữ sau 1s – Tiến 1s để thoát")
                                    move_vehicle("forward", 0.1, 1.0, shared_counts)
                                    time.sleep(0.2)
                                    shared_state.limit_active = False


                            # === Xử lý L3/L4 ===
                            elif new_press['L3'] or new_press['L4']:
                                print("[LIMIT] ⏱ L3/L4 – Dừng 1s chờ nhả...")
                                shared_state.limit_active = True
                                stop_all()
                                time.sleep(1.0)

                                if not (l3 or l4):
                                    print("[LIMIT] ✅ Công tắc đã nhả – Quay lại tự hành")
                                    shared_state.limit_active = False
                                else:
                                    print("[LIMIT] ❌ Vẫn giữ công tắc – Lùi 0.3s và xét LiDAR để xoay hoặc dừng")
                                    move_vehicle("backward", 0.1, 0.3, shared_counts)
                                    time.sleep(0.4)

                                    try:
                                        from shared_state import mean_l, mean_r
                                        if mean_l == 0.0 and mean_r == 0.0:
                                            raise ValueError("No LiDAR scan")

                                        if mean_l > mean_r:
                                            print("[LIMIT] ↪️ Xoay trái (ít vật bên trái)")
                                            move_vehicle("left", 0.1, 0.5, shared_counts)
                                        else:
                                            print("[LIMIT] ↩️ Xoay phải (ít vật bên phải)")
                                            move_vehicle("right", 0.1, 0.5, shared_counts)
                                        time.sleep(0.4)
                                        shared_state.limit_active = False

                                    except Exception:
                                        print("[LIMIT] ⚠️ Không có dữ liệu LiDAR – DỪNG CHỜ")
                                        stop_all()
                                        # giữ limit_active = True để chờ công tắc nhả

                            # === Cập nhật trạng thái nhấn công tắc ===
                            last_pressed_state['L1'] = l1
                            last_pressed_state['L2'] = l2
                            last_pressed_state['L3'] = l3
                            last_pressed_state['L4'] = l4

                            # Nếu tất cả công tắc đã nhả
                            if not any([l1, l2, l3, l4]):
                                if shared_state.limit_active or shared_state.escape_required or in_danger:
                                    print("[✔️] Tất cả công tắc đã nhả – Reset trạng thái")
                                    shared_state.limit_active = False
                                    shared_state.escape_required = False
                                    in_danger = False

            except Exception as e:
                print(f"[ERROR] Lỗi nhận Pi5: {e}")
            finally:
                conn.close()
                print("[Pi4] 🔌 Mất kết nối Pi5 – Chờ kết nối lại...")

    threading.Thread(target=run, daemon=True).start()

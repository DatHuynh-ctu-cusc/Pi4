# bluetooth_rfcomm_server.py
import bluetooth
import threading

class BluetoothServer:
    def __init__(self, port=1, on_receive=None):
        self.port = port
        self.server_sock = None
        self.client_sock = None
        self.running = threading.Event()
        self.running.set()
        self.on_receive = on_receive  # callback khi nhận dữ liệu

    def start(self):
        self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.server_sock.bind(("", self.port))
        self.server_sock.listen(1)
        print(f"[BT] 🔵 Đang chờ Pi5 kết nối trên RFCOMM port {self.port} ...")
        self.client_sock, address = self.server_sock.accept()
        print(f"[BT] ✅ Đã kết nối từ Pi5: {address}")
        threading.Thread(target=self.handle_receive, daemon=True).start()

    def handle_receive(self):
        while self.running.is_set():
            try:
                data = self.client_sock.recv(1024)
                if data:
                    msg = data.decode()
                    print("📥 Nhận từ Pi5:", msg)
                    if self.on_receive:
                        self.on_receive(msg)
            except:
                break

    def send(self, msg):
        if self.client_sock:
            self.client_sock.send(msg.encode())
            print("📤 Đã gửi:", msg)

    def close(self):
        self.running.clear()
        if self.client_sock:
            self.client_sock.close()
        if self.server_sock:
            self.server_sock.close()
        print("🔌 Đã đóng kết nối Bluetooth.")

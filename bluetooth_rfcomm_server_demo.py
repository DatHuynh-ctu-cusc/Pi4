# bluetooth_rfcomm_server_demo.py
import bluetooth
import threading

def handle_receive(client_sock):
    while True:
        try:
            data = client_sock.recv(1024)
            if data:
                print("📥 Nhận từ Pi5:", data.decode())
        except:
            break

def handle_send(client_sock):
    while True:
        msg = input("Nhập lệnh gửi đến Pi5 (q để thoát): ")
        if msg.lower() == "q":
            break
        client_sock.send(msg.encode())
        print("📤 Đã gửi:", msg)

server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_sock.bind(("", 1))
server_sock.listen(1)

print("🔵 Đang chờ kết nối từ Pi5...")
client_sock, address = server_sock.accept()
print("✅ Đã kết nối từ:", address)

# Khởi tạo 2 luồng
threading.Thread(target=handle_receive, args=(client_sock,), daemon=True).start()
handle_send(client_sock)

client_sock.close()
server_sock.close()
print("🔌 Đã đóng kết nối.")

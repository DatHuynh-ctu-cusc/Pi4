# shared_state.py

# Ngăn tự hành hoạt động nếu đang xử lý công tắc
limit_active = False

# Yêu cầu autonomous_node xử lý hướng thoát sau khi lùi (L3/L4)
escape_required = False

# Cờ cho phép robot di chuyển tự hành và tránh vật (chỉ True khi đã nhận lệnh start_scan từ Pi5)
running_scan = False

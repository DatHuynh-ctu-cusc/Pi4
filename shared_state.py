# shared_state.py
shared_counts = {"E1": 0, "E2": 0, "E3": 0, "E4": 0}
# Ngăn tự hành hoạt động nếu đang xử lý công tắc
limit_active = False

# Yêu cầu autonomous_node xử lý hướng thoát sau khi lùi (L3/L4)
escape_required = False

# Cờ cho phép robot di chuyển tự hành và tránh vật (chỉ True khi đã nhận lệnh start_scan từ Pi5)
running_scan = False

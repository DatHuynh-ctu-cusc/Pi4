# shared_state.py
shared_counts = {"E1": 0, "E2": 0, "E3": 0, "E4": 0}
# Ngăn tự hành hoạt động nếu đang xử lý công tắc
limit_active = False
mean_l = 0.0
mean_r = 0.0
# Yêu cầu autonomous_node xử lý hướng thoát sau khi lùi (L3/L4)
escape_required = False


running_scan = False     # Chế độ tự hành quét bản đồ
running_path = False     # Chế độ đi theo đường vẽ
limit_active = False     # Trạng thái công tắc giới hạn
escape_required = False  # Cờ thoát va chạm

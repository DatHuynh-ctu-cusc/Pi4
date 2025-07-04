# odometry.py
import math

class Odometry:
    def __init__(self, wheel_diameter=0.065, wheel_base=0.23, counts_per_rev=514.8):
        self.x = 0.0  # mét
        self.y = 0.0
        self.theta = 0.0  # radian (góc robot)

        self.wheel_diameter = wheel_diameter
        self.wheel_base = wheel_base
        self.counts_per_rev = counts_per_rev

    def update(self, counts):
        # Tính khoảng cách từng bên
        left_counts = (counts["E1"] + counts["E2"]) / 2
        right_counts = (counts["E3"] + counts["E4"]) / 2

        # Tính quãng đường từng bên
        wheel_circumference = math.pi * self.wheel_diameter
        distance_left = (left_counts / self.counts_per_rev) * wheel_circumference
        distance_right = (right_counts / self.counts_per_rev) * wheel_circumference

        # Tính trung bình quãng đường
        distance_center = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / self.wheel_base

        # Cập nhật vị trí
        self.theta += delta_theta
        self.x += distance_center * math.cos(self.theta)
        self.y += distance_center * math.sin(self.theta)

    def get_pose(self):
        return {"x": self.x, "y": self.y, "theta": self.theta}
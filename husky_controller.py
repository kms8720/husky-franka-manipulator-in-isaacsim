#!/usr/bin/env python3
"""
Husky Virtual Joystick - 외부 ROS2 노드
Isaac Sim의 ROS2 Subscribe Twist 노드로 /cmd_vel을 publish
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import math
import threading


class HuskyJoystick(Node):
    def __init__(self):
        super().__init__('husky_virtual_joystick')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Husky 속도 한계 (실기 스펙)
        self.max_linear = 1.0
        self.max_angular = 1.0

        # 조이스틱 UI
        self.win_size = 400
        self.center = (self.win_size // 2, self.win_size // 2 + 40)
        self.outer_radius = 130
        self.stick_radius = 40
        self.deadzone = 8

        self.stick_pos = self.center
        self.dragging = False
        self.running = True
        self.last_cmd = (0.0, 0.0)

        # 50Hz로 publish
        self.timer = self.create_timer(0.001, self.publish_cmd)

    def on_mouse(self, event, x, y, flags, param):
        cx, cy = self.center
        if event == cv2.EVENT_LBUTTONDOWN:
            if math.hypot(x - cx, y - cy) <= self.outer_radius:
                self.dragging = True
                self.update_stick(x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            self.update_stick(x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
            self.stick_pos = self.center

    def update_stick(self, x, y):
        cx, cy = self.center
        dx, dy = x - cx, y - cy
        dist = math.hypot(dx, dy)
        r = self.outer_radius
        if dist > r:
            dx = dx * r / dist
            dy = dy * r / dist
        self.stick_pos = (int(cx + dx), int(cy + dy))

    def compute_twist(self):
        cx, cy = self.center
        sx, sy = self.stick_pos
        dx, dy = sx - cx, sy - cy
        if math.hypot(dx, dy) < self.deadzone:
            return 0.0, 0.0
        r = self.outer_radius
        linear = (-dy / r) * self.max_linear
        angular = (-dx / r) * self.max_angular
        return linear, angular

    def publish_cmd(self):
        linear, angular = self.compute_twist()
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)
        self.last_cmd = (linear, angular)

    def draw(self):
        img = np.zeros((self.win_size, self.win_size, 3), dtype=np.uint8)
        linear, angular = self.last_cmd

        cv2.putText(img, f'Max Lin Speed: {self.max_linear:.1f} m/s',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f'Max Ang Speed: {math.degrees(self.max_angular):.1f} deg/s',
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, f'Linear  Vel X: {linear:+.2f} m/s',
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
        cv2.putText(img, f'Angular Vel Z: {math.degrees(angular):+.1f} deg/s',
                    (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 200), 1)

        # 외곽 빨간 원
        cv2.circle(img, self.center, self.outer_radius, (0, 0, 200), 2)
        # 중심 십자선
        cx, cy = self.center
        r = self.outer_radius
        cv2.line(img, (cx - r, cy), (cx + r, cy), (50, 50, 50), 1)
        cv2.line(img, (cx, cy - r), (cx, cy + r), (50, 50, 50), 1)
        # 흰 스틱
        cv2.circle(img, self.stick_pos, self.stick_radius, (255, 255, 255), -1)

        # 안내
        cv2.putText(img, 'Drag to control. Q/ESC to quit.',
                    (10, self.win_size - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1)
        return img

    def run_ui(self):
        win = 'Husky Virtual Joystick -> /cmd_vel'
        cv2.namedWindow(win)
        cv2.setMouseCallback(win, self.on_mouse)

        while self.running and rclpy.ok():
            cv2.imshow(win, self.draw())
            key = cv2.waitKey(20) & 0xFF
            if key == 27 or key == ord('q'):
                self.running = False
                break

        cv2.destroyAllWindows()


def main():
    rclpy.init()
    node = HuskyJoystick()

    # ROS2 spin은 별도 스레드, 메인 스레드는 OpenCV UI
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_ui()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

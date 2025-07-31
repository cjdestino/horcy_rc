from pynput import keyboard, mouse
from pynput.mouse import Controller
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time
import screeninfo

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear = [0.0, 0.0]
        self.angular = 0.0
        self.sensitivity = 0.05
        self.last_x = None
        self.last_time = None
        self.last_move_time = 0.0

        self.mouse_controller = Controller()
        screen = screeninfo.get_monitors()[0]
        self.screen_width = screen.width
        self.screen_height = screen.height
        self.screen_center = (self.screen_width // 2, self.screen_height // 2)
        self.mouse_controller.position = self.screen_center

        keyboard.Listener(on_press=self.on_key_down, on_release=self.on_key_up).start()
        mouse.Listener(on_move=self.on_mouse_move).start()
        threading.Thread(target=self.publish_loop, daemon=True).start()

    def on_key_down(self, key):
        try:
            if key.char == 'w': self.linear[1] = 1.0
            elif key.char == 's': self.linear[1] = -1.0
            elif key.char == 'a': self.linear[0] = -1.0
            elif key.char == 'd': self.linear[0] = 1.0
        except AttributeError:
            pass

    def on_key_up(self, key):
        try:
            if key.char in ['w', 's']: self.linear[1] = 0.0
            elif key.char in ['a', 'd']: self.linear[0] = 0.0
        except AttributeError:
            pass

    def on_mouse_move(self, x, y):
        now = time.time()

        if self.last_x is None:
            self.last_x = x
            self.last_time = now
            return

        dx = x - self.last_x
        dt = now - self.last_time
        self.last_x = x
        self.last_time = now

        if dt > 0 and abs(dx) > 1.0:
            raw_omega = -dx / dt * self.sensitivity
            self.angular = max(min(raw_omega, 1.0), -1.0)
            self.last_move_time = now
        else:
            self.angular = 0.0

        edge_threshold = 5
        if x <= edge_threshold or x >= self.screen_width - edge_threshold:
            self.mouse_controller.position = self.screen_center
            self.last_x = None
            self.last_time = None

    def publish_loop(self):
        rate = self.create_rate(20)
        while rclpy.ok():
            msg = Twist()
            msg.linear.x = self.linear[0]
            msg.linear.y = self.linear[1]

            if time.time() - self.last_move_time < 0.05:
                msg.angular.z = self.angular
            else:
                msg.angular.z = 0.0

            self.pub.publish(msg)
            rate.sleep()

def main():
    rclpy.init()
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

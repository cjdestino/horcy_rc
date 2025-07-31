#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MecanumController(Node):
    def __init__(self):
        super().__init__('mecanum_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.run_control_loop()

    def calculate_velocity(self, angle, speed):
        """ Convert angle and speed into x, y velocities for Mecanum wheels. """
        rad = math.radians(angle)
        vel_x = speed * math.cos(rad)
        vel_y = speed * math.sin(rad)
        return vel_x, vel_y

    def run_control_loop(self):
        angle = float(input("Enter direction angle (degrees): "))
        speed = 1.0  # Full speed

        vel_x, vel_y = self.calculate_velocity(angle, speed)

        msg = Twist()
        msg.linear.x = vel_x
        msg.linear.y = vel_y

        self.publisher.publish(msg)
        self.get_logger().info(f"Moving at angle {angle}° with velocity X={vel_x}, Y={vel_y}")

def main(args=None):
    rclpy.init(args=args)
    node = MecanumController()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from wheel_control.HiwonderSDK import Board  

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)
        self.subscription  

    def cmd_callback(self, msg):
        vx = msg.linear.x     
        vy = msg.linear.y     
        omega = msg.angular.z 

        fl = vy + vx - omega  
        fr = vy - vx + omega  
        rl = vy - vx - omega  
        rr = vy + vx + omega  

        max_val = max(abs(fl), abs(fr), abs(rl), abs(rr), 1.0)
        scale = 100.0 / max_val

        fl = int(fl * scale)
        fr = int(fr * scale)
        rl = int(rl * scale)
        rr = int(rr * scale)

        try:
            Board.setMotor(1, fl)
            Board.setMotor(2, fr)
            Board.setMotor(3, rl)
            Board.setMotor(4, rr)
        except Exception as e:
            self.get_logger().error(f"Motor command failed: {e}")

def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

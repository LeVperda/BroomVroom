import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2
import math
 
i = 0

class AnglePublisher(Node):

    def __init__(self):
        super().__init__('double_publisher')
        self.speed_publisher = self.create_publisher(Int32, 'wheel_angle_control', 10)
        self.angle_publisher = self.create_publisher(Int32, 'speed_control', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg_speed = Int32()
        msg_speed.data = 1550
        msg_angle = Int32()
        msg_angle.data = (90)
        self.speed_publisher.publish(msg_speed)
        self.angle_publisher.publish(msg_angle)
        self.get_logger().info(f'Publishing wheel angle: {msg_angle.data}')
        self.get_logger().info(f'Publishing speed: {msg_speed.data}')





def main(args=None):
    rclpy.init(args=args)

    angle_publisher = AnglePublisher()
    
    rclpy.spin(angle_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    angle_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

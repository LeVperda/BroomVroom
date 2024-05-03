import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2
import math
import threading

class WebcamStream:
    def __init__(self):
        self.cam = cv2.VideoCapture(4)
        cv2.namedWindow("preview")

    def start_stream(self):
        while True:
            ret, img = self.cam.read()
            cv2.imshow("preview", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def stop_stream(self):
        self.cam.release()
        cv2.destroyAllWindows()

class AnglePublisher(Node):
    def __init__(self):
        super().__init__('control_publisher')
        self.wheel_publisher = self.create_publisher(Int32, 'wheel_angle_control', 1)
        self.speed_publisher = self.create_publisher(Int32, 'speed_control', 1)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mutex = threading.Lock()
        self.webcam_stream = WebcamStream()
        self.webcam_thread = threading.Thread(target=self.webcam_stream.start_stream)
        self.webcam_thread.start()

    def timer_callback(self):
        msg_wheel = Int32()
        msg_speed = Int32()
        msg_speed.data = 1550
        # VALUE HERE

        ret, img = self.webcam_stream.cam.read()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        height, width, colorlayers = img.shape
        horizontal_line = int(height * (3/5))
        blue = 0
        green = 1
        red = 2
        middle = int(width / 2) + 100
        car_line = height
        gap = car_line - horizontal_line

        self.mutex.acquire()

        # find left and right sides of the track
        def get_track_sides():
            color_limiter = 200
            color_low_lim = 199
            l_line, r_line = 0, width

            for x in range(int(width/2), 0, -1):
                fr = img[horizontal_line, x]
                if fr[blue] >= color_limiter and fr[green] <= color_low_lim and fr[red] <= color_low_lim or fr[blue] <= color_low_lim and fr[green] <= color_low_lim and fr[red] >= color_limiter or fr[blue] >= color_limiter and fr[green] <= color_low_lim and fr[red] >= color_limiter:
                    l_line = x
                    break
                continue

            for x in range(int(width/2), width):
                fr = img[horizontal_line, x]
                if fr[blue] >= color_limiter and fr[green] <= color_low_lim and fr[red] <= color_low_lim or fr[blue] <= color_low_lim and fr[green] <= color_low_lim and fr[red] >= color_limiter or fr[blue] >= color_limiter and fr[green] <= color_low_lim and fr[red] >= color_limiter:
                    r_line = x
                    break
                continue

            return l_line, r_line

        # get the turning angle
        def get_turn():
            l_line, r_line = get_track_sides()
            mid_point = l_line + (r_line - l_line) / 2
            mid_point -= 100

            if mid_point > middle:
                distance = mid_point - middle
                angle_rad = math.atan(distance/gap)
                angle = math.degrees(angle_rad)
                return 90 - angle

            elif mid_point < middle:
                distance = middle - mid_point
                angle_rad = math.atan(distance/gap)
                angle = math.degrees(angle_rad)
                return 90 + angle

            else:
                return 90

        self.mutex.release()

        msg_wheel.data = int(get_turn())

        self.wheel_publisher.publish(msg_wheel)
        self.get_logger().info(f'Publishing wheel angle: {msg_wheel.data}')
        # self.speed_publisher.publish(msg_speed)
        # self.get_logger().info(f'Publishing speed angle: {msg_speed.data}')

def main(args=None):
    rclpy.init(args=args)

    angle_publisher = AnglePublisher()

    rclpy.spin(angle_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    angle_publisher.destroy_node()

    angle_publisher.webcam_stream.stop_stream()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

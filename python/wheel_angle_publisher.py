import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import cv2
import math
 
cam = cv2.VideoCapture(4)
spid = True

class AnglePublisher(Node):

    def __init__(self):
        super().__init__('control_publisher')
        self.wheel_publisher = self.create_publisher(Int32, 'wheel_angle_control', 1)
        self.speed_publisher = self.create_publisher(Int32, 'speed_control', 1)
        timer_period = 0.20 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    def timer_callback(self):
        msg_wheel = Int32()
        msg_speed = Int32()
        msg_speed.data = 1550
        # VALUE HERE
        ret, img = cam.read()
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        '''
        while ret:
            cv2.imshow("preview", img)
            rval, img = cam.read()
            key = cv2.waitKey(20)
            if key == 27: # exit on ESC
                break
        #cv2.imshow('trash', img)
                # waits for user to press any key 
        #cv2.waitKey(0) 
        '''
    
        height, width, colorlayers = img.shape
        horizontal_line = int(height * (3/5))
        blue = 0
        green = 1
        red = 2
        middle = int(width / 2)
        car_line = height
        gap = car_line - horizontal_line

        # find left and rightsides of the track
        def get_track_sides():
            color_limiter = 220
            color_low_lim = 219
            l_line, r_line = 0, width
            
            for x in range(int(width/2), 0, -1):
                fr = img[horizontal_line, x]
                if fr[blue] >= color_limiter and fr[green] <= color_low_lim and fr[red] <= color_low_lim or fr[blue] <= color_low_lim and fr[green] <= color_low_lim and fr[red] >= color_limiter or fr[blue] >= color_limiter and fr[green] <= color_low_lim and fr[red] >= color_limiter:
                    l_line = x
                    #if l_line > width*0.4:
                    #    l_line = width*0.5
                    break
                continue

            for x in range(int(width/2), width):
                fr = img[horizontal_line, x]
                if fr[blue] >= color_limiter and fr[green] <= color_low_lim and fr[red] <= color_low_lim or fr[blue] <= color_low_lim and fr[green] <= color_low_lim and fr[red] >= color_limiter or fr[blue] >= color_limiter and fr[green] <= color_low_lim and fr[red] >= color_limiter:
                    r_line = x
                    #if r_line < width*0.6:
                     #   r_line = width*0.5
                    break
                continue
            if (r_line - l_line < width*0.3):
                if (r_line - middle) > (middle - l_line):
                    l_line = 0
                else:
                    r_line = width

            return l_line, r_line
        
        # get the turning angle
        def get_turn():
            l_line, r_line = get_track_sides()                                                        
            mid_point = l_line + (r_line - l_line) / 2
            mid_point -= 0

            if mid_point > middle:
                distance = mid_point - middle
                angle_rad = math.atan(distance/gap)
                angle = math.degrees(angle_rad)
                #if angle > 35:
                #    angle*0.7
                if 90 - angle < 45:
                    return 45
                else: return 90 - angle
                        
            elif mid_point < middle:
                distance = middle - mid_point
                angle_rad = math.atan(distance/gap)
                angle = math.degrees(angle_rad)
                #if angle > 35:
                #    angle*0.7
                if 90 + angle > 135:
                    return 135
                else:
                    return 90 + angle
            
            else:
                return 90
        
            
        def get_speed():
            global spid
            if spid == True:
                spid = False
                return 1545
            else:
                spid = True
                return 1530
                
        msg_wheel.data = int(get_turn())
        #msg_speed.data = get_speed()

        self.wheel_publisher.publish(msg_wheel)
        self.get_logger().info(f'Publishing wheel angle: {msg_wheel.data}')
        #self.speed_publisher.publish(msg_speed)
        #self.get_logger().info(f'Publishing speed angle: {msg_speed.data}')



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

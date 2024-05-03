import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from joblib import load
import urllib.request, json 
import pandas as pd
import sklearn
model = load("/home/team1/microros_ws/src/publisher/publisher/weathercontrolmodel.joblib")

class SpeedPublisher(Node):

    def __init__(self):
        super().__init__('speed_publisher')
        self.publisher_ = self.create_publisher(Int32, 'speed_control', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        data = None
        msg = Int32()
        with urllib.request.urlopen("https://edu.frostbit.fi/api/road_weather/") as url:
          data = json.load(url)
        labels = ['Ice', 'Normal', 'Rain', 'Snow']

        if data != None:
            # create a pandas DataFrame
            tester_row = pd.DataFrame([data])
            result = model.predict(tester_row)[0]
            if result == "Normal":
                msg.data = 1530
                weather_log = "Normal weather, drive with normal speed!"

            elif result == "Rain":
                msg.data = 1530
                weather_log = "Raining. Reduce speed by 20%!"
            elif result == "Snow":
                msg.data = 1530
                weather_log = "Snowing. Reduce speed by 33%!"
            elif result == "Ice":
                msg.data = 1530
                weather_log = "Icy! Reduce speed by 50%!"
            
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing speed: {weather_log} {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    speed_publisher = SpeedPublisher()
    rclpy.spin(speed_publisher)
    speed_publisher.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
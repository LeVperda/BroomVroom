#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


#include <std_msgs/msg/String.h>
#include <std_msgs/msg/Int32.h>

rcl_subscription_t subscriber_wheel_angle;
rcl_subscription_t subscriber_speed;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,temp_rc); error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  delay(5000);
}

void speed_subscription_callback(const void *msgin) {
  // STRINGS
  //const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  //const char *data = msg->data.data;  // Access the string data
  //const char *data = msg->data;  // Access the string data
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  Serial.print("Wheel Angle: ");
  Serial.println(msg->data);
  
}

void wheel_angle_subscription_callback(const void *msgin) {
  // STRINGS
  //const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  //const char *data = msg->data.data;  // Access the string data
  //const char *data = msg->data;  // Access the string data
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  Serial.print("Speed: ");
  Serial.println(msg->data);

}


void setup() {
  Serial.begin(115200);
  WiFi.begin("Test", "11111111");
  Serial.println("Connecting to Wi-Fi");
  set_microros_wifi_transports("Test", "11111111", "172.20.10.5", 8888);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  Serial.println("Initilizing...");

  // create node
  RCCHECK(rclc_node_init_default(&node, "arduino_node", "", &support));
  Serial.println("Initializing node...");

  // create subscriber for speed
RCCHECK(rclc_subscription_init_default(
  &subscriber_speed,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), 
  "wheel_angle_control"));
  Serial.println("Wheel angle subscriber ready...");

  // Create subscriber for wheels' angle
  RCCHECK(rclc_subscription_init_default(
  &subscriber_wheel_angle,
  &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), 
  "speed_control"));
  Serial.println("Speed subscriber ready...");


  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_wheel_angle, &msg, &wheel_angle_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_speed, &msg, &speed_subscription_callback, ON_NEW_DATA));
  Serial.println("Executor ready");
}

void loop() {
  delay(100);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

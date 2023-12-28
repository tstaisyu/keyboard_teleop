#include <M5Stack.h>
#include "micro_ros_arduino.h"
#include <std_msgs/msg/String.h>
#include "CytronMotorDriver.h"

rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t subscriber;
std_msgs__msg__String msg;

const int motorRPin1 = 25;
const int motorRPin2 = 2;
const int motorLPin1 = 26;
const int motorLPin2 = 5;

CytronMD motor_R(PWM_DIR, motorRPin1, motorRPin2);
CytronMD motor_L(PWM_DIR, motorLPin1, motorLPin2);

void subscription_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  if (strcmp(msg->data.data, "forward") == 0) {
    motor_R.setSpeed(255);
    motor_L.setSpeed(255);
  } else if (strcmp(msg->data.data, "backward") == 0) {
    motor_R.setSpeed(-255);
    motor_L.setSpeed(-255);
  } else if (strcmp(msg->data.data, "left") == 0) {
    motor_R.setSpeed(-255);
    motor_L.setSpeed(255);
  } else if (strcmp(msg->data.data, "right") == 0) {
    motor_R.setSpeed(255);
    motor_L.setSpeed(-255);
  } else if (strcmp(msg->data.data, "stop") == 0) {
    motor_R.setSpeed(0);
    motor_L.setSpeed(0);
  }
}

void setup() {
  M5.begin();
  set_microros_transports();

  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "m5stack_motor_control", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "motor_control");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}

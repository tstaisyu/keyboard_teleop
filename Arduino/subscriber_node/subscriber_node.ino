#include <micro_ros_arduino.h>

#include <M5Stack.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include "CytronMotorDriver.h"

rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn;}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

const int motorRPin1 = 25;
const int motorRPin2 = 2;
const int motorLPin1 = 26;
const int motorLPin2 = 5;

CytronMD motor_R(PWM_DIR, motorRPin1, motorRPin2);
CytronMD motor_L(PWM_DIR, motorLPin1, motorLPin2);

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  switch (msg->data) {
    case 1: // forward
      motor_R.setSpeed(255);
      motor_L.setSpeed(255);
      M5.Lcd.setCursor(0, 20);
      M5.Lcd.print("Moving Forward ");
      break;
    case 2: // backward
      motor_R.setSpeed(-255);
      motor_L.setSpeed(-255);
      M5.Lcd.setCursor(0, 20);
      M5.Lcd.print("Moving Backward");
      break;
    case 3: // left
      motor_R.setSpeed(255);
      motor_L.setSpeed(-255);
      M5.Lcd.setCursor(0, 20);
      M5.Lcd.print("Turning Left   ");
      break;
    case 4: // right
      motor_R.setSpeed(-255);
      motor_L.setSpeed(255);
      M5.Lcd.setCursor(0, 20);
      M5.Lcd.print("Turning Right  ");
      break;
    default:
      // stop or undefined command
      motor_R.setSpeed(0);
      motor_L.setSpeed(0);
      M5.Lcd.setCursor(0, 20);
      M5.Lcd.print("Stopped        ");
      break;
  }
}

void setup() {
  M5.begin();
  M5.Lcd.setTextSize(2);
  M5.Lcd.print("micro ROS2 M5Stack START\n");  
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "m5stack_motor_control", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/motor_control"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}

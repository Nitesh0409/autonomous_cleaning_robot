#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Pins for L298N Motor Driver
#define ENA 14
#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33
#define ENB 32

// Pins for HC-SR04 Ultrasonic Sensors
#define TRIG1 12
#define ECHO1 13
#define TRIG2 15
#define ECHO2 2

#define EMERGENCY_STOP_DISTANCE 15 // cm

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

float current_linear = 0.0;
float current_angular = 0.0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

void move_motors(float linear, float angular) {
  // Simple Skid-Steer Logic
  int left_speed = (linear - angular) * 255;
  int right_speed = (linear + angular) * 255;
  
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);

  digitalWrite(IN1, left_speed > 0);
  digitalWrite(IN2, left_speed < 0);
  analogWrite(ENA, abs(left_speed));

  digitalWrite(IN3, right_speed > 0);
  digitalWrite(IN4, right_speed < 0);
  analogWrite(ENB, abs(right_speed));
}

float get_distance(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH);
  return duration * 0.034 / 2;
}

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  current_linear = msg->linear.x;
  current_angular = msg->angular.z;
}

void setup() {
  set_microros_transports();
  
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(TRIG1, OUTPUT); pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT); pinMode(ECHO2, INPUT);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_survival_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  float dist1 = get_distance(TRIG1, ECHO1);
  float dist2 = get_distance(TRIG2, ECHO2);
  
  if (dist1 < EMERGENCY_STOP_DISTANCE || dist2 < EMERGENCY_STOP_DISTANCE) {
    // Survival Priority: STOP and BACK UP
    move_motors(-0.5, 0.0);
    delay(500);
    move_motors(0, 0.5); // Turn to evade
    delay(500);
  } else {
    // Mastermind Priority: Follow cmd_vel
    move_motors(current_linear, current_angular);
  }
  
  delay(10);
}

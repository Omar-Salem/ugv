// // https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca
// // https://github.com/micro-ROS/micro_ros_platformio

// #include <Arduino.h>
// #include <micro_ros_platformio.h>

// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <ugv_interfaces/msg/motor.h>
// #include <ugv_interfaces/msg/motors_odom.h>
// #include "TwoPinStepperMotor.h"

// #if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
// #error This example is only avaliable for Arduino framework with serial transport.
// #endif

// rcl_subscription_t velocityCommandSubscriber;
// ugv_interfaces__msg__MotorsOdom velocityCommandCallbackMessage;
// rclc_executor_t subscriberExecutor;

// rcl_publisher_t odomStatePublisher;
// rclc_executor_t publisherExecutor;

// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t publisherTimer;
// const unsigned int PUBLISHER_TIMER_TIMEOUT_MILL = 100;

// const int right_step = 4;
// const int right_dir = 16;
// TwoPinStepperMotor right(right_step, right_dir, true);

// const int left_step = 12;
// const int left_dir = 14;
// TwoPinStepperMotor left(left_step, left_dir, false);


// // #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("NOOOOOOOOOOOOOO");}}

// // Error handle loop
// // void error_loop() {
// //     while (1) {
// //         delay(100);
// //     }
// // }

// void velocityCommandCallback(const void *msgin) {
//     const ugv_interfaces__msg__MotorsOdom *command = (const ugv_interfaces__msg__MotorsOdom *) msgin;
//     left.setVelocity(command->left.velocity);
//     right.setVelocity(command->right.velocity);

//     printf("left velocity: %d\n", command->left.velocity);
//     printf("right velocity: %d\n", command->right.velocity);
// }


// void setup() {
//     Serial.begin(115200);
//     set_microros_serial_transports(Serial);

//     allocator = rcl_get_default_allocator();

//     RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

//     RCSOFTCHECK(rclc_node_init_default(&node, "micro_ros_ugv_motors", "", &support));

//        left.setVelocity(4000);
//     right.setVelocity(4000);

// }

// void loop() {
//         left.move();
//         right.move();
// }
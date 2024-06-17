# 1 "/tmp/tmpzb5k_6dc"
#include <Arduino.h>
# 1 "/home/omar/ugv_ws/src/ugv_chassis_firmware/esp32dev-microros/src/esp32.ino"



#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <ugv_interfaces/msg/motor.h>
#include <ugv_interfaces/msg/motors_odom.h>
#include "TwoPinStepperMotor.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t velocityCommandSubscriber;
ugv_interfaces__msg__MotorsOdom velocityCommandCallbackMessage;
rclc_executor_t subscriberExecutor;

rcl_publisher_t odomStatePublisher;
rclc_executor_t publisherExecutor;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t publisherTimer;
const unsigned int PUBLISHER_TIMER_TIMEOUT_MILL = 100;

const int right_step = 4;
const int right_dir = 16;
TwoPinStepperMotor right(right_step, right_dir, true);

const int left_step = 12;
const int left_dir = 14;
TwoPinStepperMotor left(left_step, left_dir, false);


TaskHandle_t moveMotorsTask;



#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("NOOOOOOOOOOOOOO");}}
# 54 "/home/omar/ugv_ws/src/ugv_chassis_firmware/esp32dev-microros/src/esp32.ino"
void velocityCommandCallback(const void *msgin);
void odomStateTimerCallback(rcl_timer_t *timer, int64_t last_call_time);
void createStatePublisher();
void createCommandSubscriber();
void setup();
void loop();
void moveMotors(void *pvParameters);
#line 54 "/home/omar/ugv_ws/src/ugv_chassis_firmware/esp32dev-microros/src/esp32.ino"
void velocityCommandCallback(const void *msgin) {
    const ugv_interfaces__msg__MotorsOdom *command = (const ugv_interfaces__msg__MotorsOdom *) msgin;
    left.setVelocity(command->left.velocity);
    right.setVelocity(command->right.velocity);

    printf("left velocity: %d\n", command->left.velocity);
    printf("right velocity: %d\n", command->right.velocity);
}

void odomStateTimerCallback(rcl_timer_t *timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        ugv_interfaces__msg__MotorsOdom msg;

        msg.left.position = left.getPosition();
        msg.right.position = right.getPosition();

        printf("left position: %d\n", left.getPosition());
        printf("right position: %d\n", right.getPosition());

        RCSOFTCHECK(rcl_publish(&odomStatePublisher, &msg, NULL));
    }
}

void createStatePublisher() {

    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(ugv_interfaces, msg,
                                                                                    MotorsOdom);


    RCSOFTCHECK(rclc_publisher_init_default(
            &odomStatePublisher,
            &node,
            type_support,
            "ugv/motors_state"));


    RCSOFTCHECK(rclc_timer_init_default(
            &publisherTimer,
            &support,
            RCL_MS_TO_NS(PUBLISHER_TIMER_TIMEOUT_MILL),
            odomStateTimerCallback));



    RCSOFTCHECK(rclc_executor_init(&publisherExecutor, &support.context, 1, &allocator));
    RCSOFTCHECK(rclc_executor_add_timer(&publisherExecutor, &publisherTimer));
}

void createCommandSubscriber() {
    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(ugv_interfaces, msg,
                                                                                    MotorsOdom);

    RCSOFTCHECK(rclc_subscription_init_default(
            &velocityCommandSubscriber,
            &node,
            type_support,
            "ugv/motors_cmd"));


    RCSOFTCHECK(rclc_executor_init(&subscriberExecutor, &support.context, 1, &allocator));

    RCSOFTCHECK(rclc_executor_add_subscription(&subscriberExecutor,
                                               &velocityCommandSubscriber,
                                               &velocityCommandCallbackMessage,
                                               &velocityCommandCallback,
                                               ON_NEW_DATA));
}

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCSOFTCHECK(rclc_node_init_default(&node, "micro_ros_ugv_motors", "", &support));

    createStatePublisher();

    createCommandSubscriber();

    xTaskCreatePinnedToCore(
            moveMotors,
            "moveMotorsTask",
            10000,
            NULL,
            0,
            &moveMotorsTask,
            0);

}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&publisherExecutor, RCL_MS_TO_NS(50)));
    RCSOFTCHECK(rclc_executor_spin_some(&subscriberExecutor, RCL_MS_TO_NS(50)));
}

void moveMotors(void *pvParameters) {
    for (;;) {
        left.move();
        right.move();
    }
}
# 1 "/home/omar/ugv_ws/src/ugv_chassis_firmware/esp32dev-microros/src/basic.ino"
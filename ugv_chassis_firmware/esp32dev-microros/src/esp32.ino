// https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca
// https://github.com/micro-ROS/micro_ros_platformio

#include <Arduino.h>
#include <Motor.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <ugv_interfaces/msg/motors_odom.h>

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

// X
const int REAR_LEFT_STEP = 26;
const int REAR_LEFT_DIR = 15;
Motor rearLeftWheel(REAR_LEFT_STEP, REAR_LEFT_DIR);

// Z
const int REAR_RIGHT_STEP = 14;
const int REAR_RIGHT_DIR = 0;
Motor rearRightWheel(REAR_RIGHT_STEP, REAR_RIGHT_DIR);

// Y
const int FRONT_LEFT_STEP = 27;
const int FRONT_LEFT_DIR = 2;
Motor frontLeftWheel(FRONT_LEFT_STEP, FRONT_LEFT_DIR);

// A
const int FRONT_RIGHT_STEP = 12;
const int FRONT_RIGHT_DIR = 4;
Motor frontRightWheel(FRONT_RIGHT_STEP, FRONT_RIGHT_DIR);

Motor *motors[4] = {&rearLeftWheel,
                    &rearRightWheel,
                    &frontLeftWheel,
                    &frontRightWheel};

// https://randomnerdtutorials.com/esp32-dual-core-arduino-ide/
TaskHandle_t moveMotorsTask;

// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK))   \
        {                              \
            printf("NOOOOOOOOOOOOOO"); \
        }                              \
    }

void velocityCommandCallback(const void *msgin)
{
    const ugv_interfaces__msg__MotorsOdom *command = (const ugv_interfaces__msg__MotorsOdom *)msgin;
    rearLeftWheel.setSpeed(command->rear_left);
    frontLeftWheel.setSpeed(command->front_left);

    rearRightWheel.setSpeed(command->rear_right);
    frontRightWheel.setSpeed(command->front_right);
}

void odomStateTimerCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        ugv_interfaces__msg__MotorsOdom msg;

        msg.rear_left = rearLeftWheel.getCurrentPosition();
        msg.rear_right = rearRightWheel.getCurrentPosition();

        msg.front_left = frontLeftWheel.getCurrentPosition();
        msg.front_right = frontRightWheel.getCurrentPosition();

        RCSOFTCHECK(rcl_publish(&odomStatePublisher, &msg, NULL));
    }
}

void createStatePublisher()
{

    const rosidl_message_type_support_t *type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(ugv_interfaces, msg,
                                                                                    MotorsOdom);

    RCSOFTCHECK(rclc_publisher_init_default(
        &odomStatePublisher,
        &node,
        type_support,
        "ugv/motors_state"));

    // create timer
    RCSOFTCHECK(rclc_timer_init_default(
        &publisherTimer,
        &support,
        RCL_MS_TO_NS(PUBLISHER_TIMER_TIMEOUT_MILL),
        odomStateTimerCallback));

    // create executor
    RCSOFTCHECK(rclc_executor_init(&publisherExecutor, &support.context, 1, &allocator));
    RCSOFTCHECK(rclc_executor_add_timer(&publisherExecutor, &publisherTimer));
}

void createCommandSubscriber()
{
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

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();

    RCSOFTCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCSOFTCHECK(rclc_node_init_default(&node, "micro_ros_ugv_motors", "", &support));

    createStatePublisher();

    createCommandSubscriber();

    xTaskCreatePinnedToCore(
        moveMotors,       /* Task function. */
        "moveMotorsTask", /* name of task. */
        10000,            /* Stack size of task */
        NULL,             /* parameter of the task */
        0,                /* priority of the task */
        &moveMotorsTask,  /* Task handle to keep track of created task */
        0);               /* pin task to core 1 */
}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&publisherExecutor, RCL_MS_TO_NS(50)));
    RCSOFTCHECK(rclc_executor_spin_some(&subscriberExecutor, RCL_MS_TO_NS(50)));
}

void moveMotors(void *pvParameters)
{
    for (;;)
    {
        for (auto s : motors)
        {
            s->run();
        }
    }
}

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Husarnet.h>
#include <WiFi.h>
#include <Wire.h>
#include <geometry_msgs/msg/twist.h>
#include <math.h>
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>
#include <stdio.h>

#include <queue>

#include "credentials.h"
#include "utils.h"

#define SKIP_UROS false
#define SKIP_HUSARNET false
#define SKIP_WIFI false

#define LED_BUILDIN 2
#define DEADMAN_SWITCH 13
#define RCCHECK(fn)                        \
    {                                      \
        if (not SKIP_UROS) {               \
            rcl_ret_t temp_rc = fn;        \
            if ((temp_rc != RCL_RET_OK)) { \
                error_loop();              \
            }                              \
        }                                  \
    }

#define RCCRETRY(fn)                         \
    {                                        \
        if (not SKIP_UROS) {                 \
            rcl_ret_t temp_rc;               \
            do {                             \
                Serial1.printf(".");         \
                temp_rc = fn;                \
            } while (temp_rc != RCL_RET_OK); \
        }                                    \
    }

#define RCSOFTCHECK(fn)                    \
    {                                      \
        if (not SKIP_UROS) {               \
            rcl_ret_t temp_rc = fn;        \
            if ((temp_rc != RCL_RET_OK)) { \
            }                              \
        }                                  \
    }

#define AGENT_PORT 8888
#define NODE_NAME "esp32_imu_controller"

constexpr float MIN_THRESHOLD = 0.1;
char *agent_hostname = (char *)"rosbot";

rcl_publisher_t cmd_vel_pub;
rcl_publisher_t imu_pub;

geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__Imu imu_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
static bool is_holonomic = true;
size_t last_time = 0;

static Adafruit_MPU6050 mpu;
static sensors_event_t accelerometer, gyroscope, temp;
static MPU6050RPY initial_rpy_angles;
static Quaternion q;

static QueueHandle_t queue_imu_data;
static QueueHandle_t queue_rpy;

void error_loop() {
    while (1) {
        Serial1.println("error loop");
        delay(500);
    }
}
void toggleLED(void *parameter);
void read_imu(void *parameter);
void publish_data(void *parameter);
void initialize_uros();
void initialize_husarnet();
void initialize_mpu6050();
void initialize_wifi();
void get_initial_rpy();
void publish_imu(void *parameter);
void calculate_imu();
void publish_cmd_vel(void *parameter);
void get_rpy_from_sensors(MPU6050RPY *rpy);

void setup(void) {
    Serial1.begin(115200, SERIAL_8N1, 3, 1);
    Serial1.println("\r\n**************************************");
    Serial1.println("micro-ROS + Husarnet + MPU6050 example");
    Serial1.println("**************************************\r\n");
    pinMode(DEADMAN_SWITCH, INPUT_PULLUP);

    queue_imu_data = xQueueCreate(3, sizeof(ImuMeansurements));
    queue_rpy = xQueueCreate(3, sizeof(MPU6050RPY));

    if (not SKIP_WIFI) {
        initialize_wifi();
        if (not SKIP_HUSARNET) {
            initialize_husarnet();
            if (not SKIP_UROS) {
                initialize_uros();
            }
        }
    }
    // initialize_uros();
    initialize_mpu6050();
    pinMode(LED_BUILDIN, OUTPUT);
    xTaskCreate(
        toggleLED,
        "Toggle LED",
        1000,
        NULL,
        1,
        NULL);
    // xTaskCreate(
    //     publish_imu,
    //     "publish imu data",
    //     10000,
    //     NULL,
    //     1,
    //     NULL);
    xTaskCreate(
        publish_cmd_vel,
        "publish cmd data",
        10000,
        NULL,
        1,
        NULL);
}

void loop(void) {
    static TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, 1000);
}

void toggleLED(void *parameter) {
    for (;;) {
        digitalWrite(LED_BUILDIN, HIGH);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        digitalWrite(LED_BUILDIN, LOW);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void initialize_wifi() {
    Serial1.printf("📻 1. Connecting to: %s Wi-Fi network ", ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        static int cnt = 0;
        delay(500);
        Serial1.print(".");
        cnt++;
        if (cnt > 10) {
            ESP.restart();
        }
    }

    Serial1.println("done\r\n");
}

void initialize_uros() {
    Serial1.printf("⌛ 3. Launching Micro-ROS ");
    set_microros_husarnet_transports(agent_hostname, AGENT_PORT);

    allocator = rcl_get_default_allocator();
    RCCRETRY(rclc_support_init(&support, 0, NULL, &allocator));
    RCCRETRY(rclc_node_init_default(&node, NODE_NAME, "", &support));
    RCCRETRY(rclc_publisher_init_best_effort(
        &cmd_vel_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    // RCCRETRY(rclc_publisher_init_best_effort(
    //     &imu_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    //     "imu_raw"));

    RCCHECK(rmw_uros_sync_session(1000));
    Serial1.println("Connected to agent!");
}

void initialize_mpu6050() {
    Serial1.println("⌛ 4. Initializing I2C and MPU6050 ");
    Serial1.printf("Testing device connections...");
    Wire.begin();

    while (!mpu.begin()) {
        static int cnt = 0;
        delay(500);
        Serial1.print(".");
        cnt++;
        if (cnt > 10) {
            ESP.restart();
        }
    }
    Serial1.println("\nMPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
    delay(1000);
    get_initial_rpy();

    xTaskCreate(
        read_imu,
        "Read IMU data",
        20000,
        NULL,
        2,
        NULL);
}

void initialize_husarnet() {
    Serial1.printf("⌛ 2. Waiting for Husarnet to be ready ");

    Husarnet.selfHostedSetup("default");
    Husarnet.join(husarnetJoinCode, hostName);
    Husarnet.start();

    // waiting for Micro-ROS agent to be available on known peers list
    bool husarnetReady = 0;
    while (husarnetReady == 0) {
        Serial1.print(".");
        for (auto const &host : Husarnet.listPeers()) {
            if (host.second == String(agent_hostname)) {
                husarnetReady = 1;
            }
        }
        delay(1000);
    }
    Serial1.println();
}

void read_imu(void *parameter) {
    for (;;) {
        mpu.getEvent(&accelerometer, &gyroscope, &temp);
        calculate_imu();
        // vTaskDelay(10);
        if (digitalRead(DEADMAN_SWITCH)) {
            get_rpy_from_sensors(&initial_rpy_angles);
        }
    }
}
void get_rpy_from_sensors(MPU6050RPY *rpy) {
    rpy->pitch = atan(accelerometer.acceleration.x / sqrt(accelerometer.acceleration.y * accelerometer.acceleration.y + accelerometer.acceleration.z * accelerometer.acceleration.z));
    rpy->roll = atan(accelerometer.acceleration.y / sqrt(accelerometer.acceleration.x * accelerometer.acceleration.x + accelerometer.acceleration.z * accelerometer.acceleration.z));
    // rpy->yaw = atan(accelerometer.acceleration.z / sqrt(accelerometer.acceleration.x * accelerometer.acceleration.x + accelerometer.acceleration.z * accelerometer.acceleration.z));
}

void get_initial_rpy() {
    mpu.getEvent(&accelerometer, &gyroscope, &temp);
    get_rpy_from_sensors(&initial_rpy_angles);
}

void calculate_imu() {
    MPU6050RPY rpy_angles;
    get_rpy_from_sensors(&rpy_angles);
    rpy_angles.pitch -= initial_rpy_angles.pitch;
    rpy_angles.roll -= initial_rpy_angles.roll;
    rpy_angles.yaw -= initial_rpy_angles.yaw;
    xQueueSend(queue_rpy, &rpy_angles, 1000);
    Serial1.print("Roll: ");
    Serial1.print(rpy_angles.roll);
    Serial1.print(", Pitch: ");
    Serial1.print(rpy_angles.pitch);
    Serial1.print(", Yaw: ");
    Serial1.print(rpy_angles.yaw);
    Serial1.println("");

    // q = ToQuaternion(rpy_angles.roll, rpy_angles.pitch, rpy_angles.yaw);
    // normalize(&q);
    // ImuMeansurements imu_data;
    // imu_data.orientation = q;
    // imu_data.gyration.x = gyroscope.gyro.x;
    // imu_data.gyration.y = gyroscope.gyro.y;
    // imu_data.gyration.z = gyroscope.gyro.z;
    // imu_data.acceleration.x = accelerometer.acceleration.x;
    // imu_data.acceleration.y = accelerometer.acceleration.y;
    // imu_data.acceleration.z = accelerometer.acceleration.z;

    // xQueueSend(queue_imu_data, &imu_data, 1000);
}

void publish_imu(void *parameter) {
    while (true) {
        ImuMeansurements imu_data;
        // xQueueReceive(queue_imu_data, &imu_data, portMAX_DELAY);
        if (not SKIP_UROS or not SKIP_HUSARNET or not SKIP_WIFI) {
            imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu");
            if (rmw_uros_epoch_synchronized()) {
                imu_msg.header.stamp.sec = (int32_t)(rmw_uros_epoch_nanos() / 1000000000);
                imu_msg.header.stamp.nanosec = (uint32_t)(rmw_uros_epoch_nanos() % 1000000000);
            }
        }

        imu_msg.orientation.x = imu_data.orientation.x;
        imu_msg.orientation.y = imu_data.orientation.y;
        imu_msg.orientation.z = imu_data.orientation.z;
        imu_msg.orientation.w = imu_data.orientation.w;

        imu_msg.angular_velocity.x = imu_data.gyration.x;
        imu_msg.angular_velocity.y = imu_data.gyration.y;
        imu_msg.angular_velocity.z = imu_data.gyration.z;

        imu_msg.linear_acceleration.x = imu_data.acceleration.x;
        imu_msg.linear_acceleration.y = imu_data.acceleration.y;
        imu_msg.linear_acceleration.z = imu_data.acceleration.z;
        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void publish_cmd_vel(void *parameter) {

    // Initialise the xLastWakeTime variable with the current time.
    while (true) {
        MPU6050RPY rpy;
        xQueueReceive(queue_rpy, &rpy, portMAX_DELAY);
        if (not isnan(rpy.roll) and not isnan(rpy.pitch)) {
            cmd_vel_msg.linear.y = fabs(rpy.roll) < MIN_THRESHOLD ? 0.0 : rpy.roll / PI;
            // cmd_vel_msg.angular.z = fabs(rpy.roll)*1.2 < MIN_THRESHOLD ? 0.0 : rpy.roll / PI;
            cmd_vel_msg.linear.x = fabs(rpy.pitch) < MIN_THRESHOLD ? 0.0 : rpy.pitch / PI;

            if (not digitalRead(DEADMAN_SWITCH)) {
                RCSOFTCHECK(rcl_publish(&cmd_vel_pub, &cmd_vel_msg, NULL));
            } else {
                last_time = millis();
            }
            // vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

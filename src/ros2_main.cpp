#include <Arduino.h>

#include <MPU9250.h>
#include <stepper_motors.h>
#include <battery.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/empty.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <nav_msgs/msg/odometry.h>

// Micro-ROS constants
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ROS2 entities
rcl_context_t context;
rcl_node_t node;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;

// Publishers
rcl_publisher_t imu_publisher;
rcl_publisher_t battery_state_publisher;
rcl_publisher_t odom_publisher;

// Subscribers
rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t accel_gyro_subscriber;
rcl_subscription_t battery_enable_subscriber;

// Message buffers
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__BatteryState battery_state_msg;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Empty empty_msg;
std_msgs__msg__UInt32 battery_timeout_msg;

// MPU9250
MPU9250 mpu;

// Constants
static constexpr auto WHEEL_DIST = 0.33F;
static constexpr auto MAX_WHEEL_SPEED = 1.5F;

void error_loop()
{
  Serial.println("Error in micro-ROS! Halting execution.");
  while(1){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float left = msg->linear.x - msg->angular.z * WHEEL_DIST * 0.5;
  float right = msg->linear.x + msg->angular.z * WHEEL_DIST * 0.5;

  const auto m = abs(max(left, right));
  if (m > MAX_WHEEL_SPEED)
  {
    left *= MAX_WHEEL_SPEED / m;
    right *= MAX_WHEEL_SPEED / m;
  }

  steppersSetSpeed(left, right);
}

void battery_timeout_callback(const void * msgin)
{
  const std_msgs__msg__UInt32 * msg = (const std_msgs__msg__UInt32 *)msgin;
  batterySetWithTimeoutMs(msg->data * 1000);
}

void calibration_accel_gyro_callback(const void * msgin)
{
  (void)msgin;  // unused
  
  constexpr auto numTries = 2;

  float accBias[3] = {0, 0, 0};
  float gyroBias[3] = {0, 0, 0};

  for (int t = 0; t < numTries; t++)
  {
    mpu.setAccBias(0, 0, 0);
    mpu.setGyroBias(0, 0, 0);

    delay(10);
    mpu.calibrateAccelGyro();

    for (int i = 0; i < 3; i++)
    {
      accBias[i] += mpu.getAccBias(i);
    }

    for (int i = 0; i < 3; i++)
    {
      gyroBias[i] += mpu.getGyroBias(i);
    }
  }

  for (int i = 0; i < 3; i++)
  {
    accBias[i] /= numTries;
  }

  for (int i = 0; i < 3; i++)
  {
    gyroBias[i] /= numTries;
  }

  // Note: Logging in micro-ROS is more limited, use Serial for now
  char stringbuf[120];
  snprintf(stringbuf, sizeof(stringbuf), "acc bias: %f, %f, %f\n gyro bias: %f, %f, %f",
           accBias[0], accBias[1], accBias[2],
           gyroBias[0], gyroBias[1], gyroBias[2]);
  Serial.println(stringbuf);
}

void setup()
{
  // Configure serial for communication with agent
  Serial.begin(115200);
  Serial.println("Initializing...");

  SerialUSB.begin(115200);

  set_microros_serial_transports(SerialUSB);

  analogReadResolution(12);

  steppersInit();
  batteryInit();

  Wire.begin();

  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize MPU9250
  if (!mpu.setup(0x68))
  {
    Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
  }

  mpu.setAccBias(-40, 266, 95);
  mpu.setGyroBias(154, 138, 13.5);
  mpu.setMagBias(161.59, 82.50, 47.44);
  mpu.setMagScale(0.80, 0.79, 2.08);

  // Initialize micro-ROS
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "arips_base", "", &support));

  // Create publishers
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu"));

  RCCHECK(rclc_publisher_init_default(
    &battery_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "base_battery_state"));

  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // Create subscribers
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  RCCHECK(rclc_subscription_init_default(
    &accel_gyro_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "imu/calibrate_accel_gyro"));

  RCCHECK(rclc_subscription_init_default(
    &battery_enable_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
    "base_battery_enable_for_sec"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

  // Add subscriptions to executor
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &cmd_vel_subscriber,
    &cmd_vel_msg,
    &cmd_vel_callback,
    ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &accel_gyro_subscriber,
    &empty_msg,
    &calibration_accel_gyro_callback,
    ON_NEW_DATA));

  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &battery_enable_subscriber,
    &battery_timeout_msg,
    &battery_timeout_callback,
    ON_NEW_DATA));

  Serial.println("Setup complete!");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);

  watchdogEnable(1000);
}

void update_imu()
{
  if (mpu.update())
  {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25)
    {
      // Populate IMU message
      imu_msg.header.frame_id.data = (char*)"imu";
      imu_msg.header.frame_id.size = strlen("imu");
      imu_msg.header.stamp.sec = (uint32_t)(millis() / 1000);
      imu_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

      imu_msg.orientation.x = mpu.getQuaternionX();
      imu_msg.orientation.y = -mpu.getQuaternionY();
      imu_msg.orientation.z = -mpu.getQuaternionZ();
      imu_msg.orientation.w = mpu.getQuaternionW();

      for (int i = 0; i < 9; i++) {
        imu_msg.orientation_covariance[i] = 0.0;
      }
      imu_msg.orientation_covariance[0] = 0.0025;
      imu_msg.orientation_covariance[4] = 0.0025;
      imu_msg.orientation_covariance[8] = 0.0025;

      imu_msg.angular_velocity.x = mpu.getGyroX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;
      imu_msg.angular_velocity.y = mpu.getGyroY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;
      imu_msg.angular_velocity.z = mpu.getGyroZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;

      for (int i = 0; i < 9; i++) {
        imu_msg.angular_velocity_covariance[i] = 0.0;
      }
      imu_msg.angular_velocity_covariance[0] = 0.0025;
      imu_msg.angular_velocity_covariance[4] = 0.0025;
      imu_msg.angular_velocity_covariance[8] = 0.0025;

      imu_msg.linear_acceleration.x = mpu.getLinearAccX();
      imu_msg.linear_acceleration.y = mpu.getLinearAccY();
      imu_msg.linear_acceleration.z = mpu.getLinearAccZ() + 9.81F;

      for (int i = 0; i < 9; i++) {
        imu_msg.linear_acceleration_covariance[i] = 0.0;
      }
      imu_msg.linear_acceleration_covariance[0] = 0.0025;
      imu_msg.linear_acceleration_covariance[4] = 0.0025;
      imu_msg.linear_acceleration_covariance[8] = 0.0025;

      rcl_publish(&imu_publisher, &imu_msg, NULL);

      prev_ms = millis();
    }
  }
}

void update_battery_voltage()
{
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + 1000)
  {
    battery_state_msg.header.stamp.sec = (uint32_t)(millis() / 1000);
    battery_state_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);
    battery_state_msg.voltage = batteryReadVoltage();
    battery_state_msg.current = NAN;
    battery_state_msg.charge = NAN;
    battery_state_msg.capacity = NAN;
    battery_state_msg.design_capacity = NAN;
    battery_state_msg.percentage = NAN;
    battery_state_msg.power_supply_status = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_UNKNOWN;
    battery_state_msg.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_state_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
    battery_state_msg.present = batteryGetState();

    rcl_publish(&battery_state_publisher, &battery_state_msg, NULL);

    prev_ms = millis();
  }
}

void update_odometry()
{
  static double posX = 0;
  static double posY = 0;
  static double yaw = 0;
  static double oldStepsLeft = 0;
  static double oldStepsRight = 0;

  static uint32_t prev_ms = millis();
  const auto dMillis = millis() - prev_ms;
  if (dMillis > 40)
  {
    double left, right;
    steppersGetDist(left, right);
    const auto dL = left - oldStepsLeft;
    const auto dR = right - oldStepsRight;
    oldStepsLeft = left;
    oldStepsRight = right;

    const auto dDist = (dL + dR) / 2.0;
    const auto dYaw = (dR - dL) / WHEEL_DIST;

    posX += dDist * cos(yaw + dYaw / 2.0);
    posY += dDist * sin(yaw + dYaw / 2.0);
    yaw += dYaw;
    if (yaw > PI)
    {
      yaw -= 2 * PI;
    }
    else if (yaw < -PI)
    {
      yaw += 2 * PI;
    }

    const auto dt = dMillis * 0.001;
    const auto speed = dDist / dt;
    const auto rotSpeed = dYaw / dt;

    // Set odometry message
    odom_msg.pose.pose.position.x = posX;
    odom_msg.pose.pose.position.y = posY;
    // Create quaternion from yaw
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = sy;
    odom_msg.pose.pose.orientation.w = cy;

    odom_msg.twist.twist.linear.x = speed;
    odom_msg.twist.twist.angular.z = rotSpeed;

    for (int i = 0; i < 36; i++) {
      odom_msg.pose.covariance[i] = 0.0;
      odom_msg.twist.covariance[i] = 0.0;
    }

    odom_msg.pose.covariance[0] = 0.03;
    odom_msg.pose.covariance[7] = 0.03;
    odom_msg.pose.covariance[14] = 99999;
    odom_msg.pose.covariance[21] = 99999;
    odom_msg.pose.covariance[28] = 99999;
    odom_msg.pose.covariance[35] = 0.03;

    odom_msg.header.stamp.sec = (uint32_t)(millis() / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.size = strlen("odom");
    odom_msg.child_frame_id.data = (char*)"arips_base";
    odom_msg.child_frame_id.size = strlen("arips_base");

    memcpy(&odom_msg.twist.covariance, odom_msg.pose.covariance, sizeof(odom_msg.pose.covariance));

    rcl_publish(&odom_publisher, &odom_msg, NULL);

    prev_ms = millis();
  }
}

void loop()
{
  watchdogReset();

  // Spin executor to handle callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  batteryCheckTimeout();
  stepperCheckTimeout();
  update_imu();
  update_battery_voltage();
  update_odometry();

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

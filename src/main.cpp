#include <Arduino.h>

#include <MPU9250.h>
#include <stepper_motors.h>
#include <battery.h>

#include <ros_due.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>

// Need to override default weak watchdogSetup(), but never call it!
// https://forum.arduino.cc/t/arduino-due-watchdog-einstellen/432906/8
void watchdogSetup (void){}

DueNodeHandle nh;
MPU9250 mpu;

void cmdVelCb(const geometry_msgs::Twist &msg)
{
  static constexpr auto WHEEL_DIST = 0.33F;
  static constexpr auto MAX_WHEEL_SPEED = 1.5F;

  float left = msg.linear.x - msg.angular.z * WHEEL_DIST;
  float right = msg.linear.x + msg.angular.z * WHEEL_DIST;

  const auto m = abs(max(left, right));
  if (m > MAX_WHEEL_SPEED)
  {
    left *= MAX_WHEEL_SPEED / m;
    right *= MAX_WHEEL_SPEED / m;
  }

  steppersSetSpeed(left, right);
}

void baseBatteryCb(const std_msgs::UInt32 &msg)
{
  batterySetWithTimeoutMs(msg.data * 1000);
}

void calibrationAccelGyroCb(const std_msgs::Empty &)
{
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

  char stringbuf[120];

  snprintf(stringbuf, sizeof(stringbuf), "acc bias: %f, %f, %f\n gyro bias: %f, %f, %f",
           accBias[0], accBias[1], accBias[2],
           gyroBias[0], gyroBias[1], gyroBias[2]);

  nh.loginfo(stringbuf);
}

ros::Subscriber<std_msgs::Empty> accelGyroSub("imu/calibrate_accel_gyro", &calibrationAccelGyroCb);
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", &cmdVelCb);
ros::Subscriber<std_msgs::UInt32> baseBatterySub("base_battery_enable_for_sec", &baseBatteryCb);

sensor_msgs::Imu imu_msg;
sensor_msgs::BatteryState batteryStateMsg;
ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher batteryStatePub("base_battery_state", &batteryStateMsg);

void setup()
{
  analogReadResolution(12);

  steppersInit();
  batteryInit();

  Wire.begin();

  pinMode(LED_BUILTIN, OUTPUT); // sets the digital pin as output
  Serial.begin(115200);
  Serial.println("Start");

  if (!mpu.setup(0x68))
  { // change to your own address
    Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
  }

  mpu.setAccBias(-40, 266, 95);
  mpu.setGyroBias(154, 138, 13.5);
  mpu.setMagBias(161.59, 82.50, 47.44);
  mpu.setMagScale(0.80, 0.79, 2.08);

  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(batteryStatePub);
  nh.subscribe(accelGyroSub);
  nh.subscribe(cmdVelSub);
  nh.subscribe(baseBatterySub);

  watchdogEnable(1000);
}

void print_roll_pitch_yaw()
{
  Serial.print("Yaw, Pitch, Roll: ");
  Serial.print(mpu.getYaw(), 2);
  Serial.print(", ");
  Serial.print(mpu.getPitch(), 2);
  Serial.print(", ");
  Serial.println(mpu.getRoll(), 2);
}

void updateIMU()
{
  if (mpu.update())
  {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25)
    {
      // print_roll_pitch_yaw();
      // str_msg.data = hello;
      imu_msg.header.frame_id = "imu";
      imu_msg.header.stamp = nh.now();
      imu_msg.orientation.x = mpu.getQuaternionX();
      imu_msg.orientation.y = -mpu.getQuaternionY();
      imu_msg.orientation.z = -mpu.getQuaternionZ();
      imu_msg.orientation.w = mpu.getQuaternionW();
      imu_msg.orientation_covariance[0] = 0.0025;
      imu_msg.orientation_covariance[1] = 0;
      imu_msg.orientation_covariance[2] = 0;

      imu_msg.orientation_covariance[3] = 0;
      imu_msg.orientation_covariance[4] = 0.0025;
      imu_msg.orientation_covariance[5] = 0;

      imu_msg.orientation_covariance[6] = 0;
      imu_msg.orientation_covariance[7] = 0;
      imu_msg.orientation_covariance[8] = 0.0025;

      imu_msg.angular_velocity.x = mpu.getGyroX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;
      imu_msg.angular_velocity.y = mpu.getGyroY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;
      imu_msg.angular_velocity.z = mpu.getGyroZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY;
      imu_msg.angular_velocity_covariance[0] = 0.0025;
      imu_msg.angular_velocity_covariance[1] = 0;
      imu_msg.angular_velocity_covariance[2] = 0;

      imu_msg.angular_velocity_covariance[3] = 0;
      imu_msg.angular_velocity_covariance[4] = 0.0025;
      imu_msg.angular_velocity_covariance[5] = 0;

      imu_msg.angular_velocity_covariance[6] = 0;
      imu_msg.angular_velocity_covariance[7] = 0;
      imu_msg.angular_velocity_covariance[8] = 0.0025;

      imu_msg.linear_acceleration.x = mpu.getLinearAccX();
      imu_msg.linear_acceleration.y = mpu.getLinearAccY();
      imu_msg.linear_acceleration.z = mpu.getLinearAccZ() + 9.81F;
      imu_msg.linear_acceleration_covariance[0] = 0.0025;
      imu_msg.linear_acceleration_covariance[1] = 0;
      imu_msg.linear_acceleration_covariance[2] = 0;

      imu_msg.linear_acceleration_covariance[3] = 0;
      imu_msg.linear_acceleration_covariance[4] = 0.0025;
      imu_msg.linear_acceleration_covariance[5] = 0;

      imu_msg.linear_acceleration_covariance[6] = 0;
      imu_msg.linear_acceleration_covariance[7] = 0;
      imu_msg.linear_acceleration_covariance[8] = 0.0025;

      imu_pub.publish(&imu_msg);

      prev_ms = millis();
    }
  }
}

void updateBatteryVoltage()
{
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + 1000)
  {
    batteryStateMsg.header.stamp = nh.now();
    batteryStateMsg.voltage = batteryReadVoltage();
    batteryStateMsg.current = NAN;
    batteryStateMsg.charge = NAN;
    batteryStateMsg.capacity = NAN;
    batteryStateMsg.design_capacity = NAN;
    batteryStateMsg.percentage = NAN;
    batteryStateMsg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    batteryStateMsg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    batteryStateMsg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batteryStateMsg.present = batteryGetState();

    batteryStatePub.publish(&batteryStateMsg);

    prev_ms = millis();
  }
}

void loop()
{
  watchdogReset();

  nh.spinOnce();
  batteryCheckTimeout();
  stepperCheckTimeout();
  updateIMU();
  updateBatteryVoltage();

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

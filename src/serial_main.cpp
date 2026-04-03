#include <Arduino.h>
#include <ArduinoJson.h>

#include <MPU9250.h>
#include <stepper_motors.h>
#include <battery.h>

// MPU9250
MPU9250 mpu;

// Constants
static constexpr auto WHEEL_DIST = 0.317F;
static constexpr auto MAX_WHEEL_SPEED = 1.5F;
static constexpr size_t SERIAL_BUF_SIZE = 512;
static const char* const ODOM_CHILD_FRAME_ID = "arips_wheel_center";

// Time synchronization: offset added to millis() to match host clock
static uint64_t millisOffset = 0;

// Serial input buffer
static char inputBuf[SERIAL_BUF_SIZE];
static size_t inputLen = 0;

// Return synchronized time in milliseconds
static uint64_t syncedMillis() { return (uint64_t)millis() + millisOffset; }

// ---------------------------------------------------------------------------
// Helpers for publishing (sending JSON lines over SerialUSB)
// ---------------------------------------------------------------------------

static void publishImu(const char* frameId, uint32_t sec, uint32_t nsec,
                       double qx, double qy, double qz, double qw,
                       double gx, double gy, double gz,
                       double ax, double ay, double az)
{
  StaticJsonDocument<768> doc;

  JsonObject header = doc.createNestedObject("header");
  JsonObject stamp = header.createNestedObject("stamp");
  stamp["sec"] = sec;
  stamp["nanosec"] = nsec;
  header["frame_id"] = frameId;

  JsonObject orientation = doc.createNestedObject("orientation");
  orientation["x"] = qx;
  orientation["y"] = qy;
  orientation["z"] = qz;
  orientation["w"] = qw;

  JsonArray oc = doc.createNestedArray("orientation_covariance");
  for (int i = 0; i < 9; i++) oc.add(0.0);
  oc[0] = 0.0025; oc[4] = 0.0025; oc[8] = 0.0025;

  JsonObject angular_velocity = doc.createNestedObject("angular_velocity");
  angular_velocity["x"] = gx;
  angular_velocity["y"] = gy;
  angular_velocity["z"] = gz;

  JsonArray avc = doc.createNestedArray("angular_velocity_covariance");
  for (int i = 0; i < 9; i++) avc.add(0.0);
  avc[0] = 0.0025; avc[4] = 0.0025; avc[8] = 0.0025;

  JsonObject linear_acceleration = doc.createNestedObject("linear_acceleration");
  linear_acceleration["x"] = ax;
  linear_acceleration["y"] = ay;
  linear_acceleration["z"] = az;

  JsonArray lac = doc.createNestedArray("linear_acceleration_covariance");
  for (int i = 0; i < 9; i++) lac.add(0.0);
  lac[0] = 0.0025; lac[4] = 0.0025; lac[8] = 0.0025;

  SerialUSB.print("pub imu sensor_msgs/msg/Imu ");
  serializeJson(doc, SerialUSB);
  SerialUSB.println();
}

static void publishBatteryState(uint32_t sec, uint32_t nsec,
                                float voltage, bool present)
{
  StaticJsonDocument<384> doc;

  JsonObject header = doc.createNestedObject("header");
  JsonObject stamp = header.createNestedObject("stamp");
  stamp["sec"] = sec;
  stamp["nanosec"] = nsec;

  doc["voltage"] = voltage;
  doc["current"] = (const char*)nullptr; // NaN -> null
  doc["charge"] = (const char*)nullptr;
  doc["capacity"] = (const char*)nullptr;
  doc["design_capacity"] = (const char*)nullptr;
  doc["percentage"] = (const char*)nullptr;
  doc["power_supply_status"] = 0; // UNKNOWN
  doc["power_supply_health"] = 0; // UNKNOWN
  doc["power_supply_technology"] = 2; // LION
  doc["present"] = present;

  SerialUSB.print("pub base_battery_state sensor_msgs/msg/BatteryState ");
  serializeJson(doc, SerialUSB);
  SerialUSB.println();
}

static void publishOdometry(uint32_t sec, uint32_t nsec,
                            double posX, double posY, double yaw,
                            double speed, double rotSpeed,
                            const double poseCovariance[36])
{
  StaticJsonDocument<1536> doc;

  JsonObject header = doc.createNestedObject("header");
  JsonObject stamp = header.createNestedObject("stamp");
  stamp["sec"] = sec;
  stamp["nanosec"] = nsec;
  header["frame_id"] = "odom";
  doc["child_frame_id"] = ODOM_CHILD_FRAME_ID;

  JsonObject pose = doc.createNestedObject("pose");
  JsonObject pose_pose = pose.createNestedObject("pose");
  JsonObject pos = pose_pose.createNestedObject("position");
  pos["x"] = posX;
  pos["y"] = posY;
  pos["z"] = 0.0;

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  JsonObject ori = pose_pose.createNestedObject("orientation");
  ori["x"] = 0.0;
  ori["y"] = 0.0;
  ori["z"] = sy;
  ori["w"] = cy;

  JsonArray pc = pose.createNestedArray("covariance");
  for (int i = 0; i < 36; i++) pc.add(poseCovariance[i]);

  JsonObject twist = doc.createNestedObject("twist");
  JsonObject twist_twist = twist.createNestedObject("twist");
  JsonObject lin = twist_twist.createNestedObject("linear");
  lin["x"] = speed;
  lin["y"] = 0.0;
  lin["z"] = 0.0;
  JsonObject ang = twist_twist.createNestedObject("angular");
  ang["x"] = 0.0;
  ang["y"] = 0.0;
  ang["z"] = rotSpeed;

  JsonArray tc = twist.createNestedArray("covariance");
  for (int i = 0; i < 36; i++) tc.add(poseCovariance[i]);

  SerialUSB.print("pub odom nav_msgs/msg/Odometry ");
  serializeJson(doc, SerialUSB);
  SerialUSB.println();
}

static void publishTfOdom(uint32_t sec, uint32_t nsec,
                          double posX, double posY, double yaw)
{
  StaticJsonDocument<512> doc;

  JsonArray transforms = doc.createNestedArray("transforms");
  JsonObject tf = transforms.createNestedObject();

  JsonObject header = tf.createNestedObject("header");
  JsonObject stamp = header.createNestedObject("stamp");
  stamp["sec"] = sec;
  stamp["nanosec"] = nsec;
  header["frame_id"] = "odom";
  tf["child_frame_id"] = ODOM_CHILD_FRAME_ID;

  JsonObject transform = tf.createNestedObject("transform");
  JsonObject translation = transform.createNestedObject("translation");
  translation["x"] = posX;
  translation["y"] = posY;
  translation["z"] = 0.0;

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  JsonObject rotation = transform.createNestedObject("rotation");
  rotation["x"] = 0.0;
  rotation["y"] = 0.0;
  rotation["z"] = sy;
  rotation["w"] = cy;

  SerialUSB.print("pub /tf tf2_msgs/msg/TFMessage ");
  serializeJson(doc, SerialUSB);
  SerialUSB.println();
}

// ---------------------------------------------------------------------------
// Subscription handlers
// ---------------------------------------------------------------------------

static void handleCmdVel(JsonObjectConst json)
{
  float linX = json["linear"]["x"] | 0.0f;
  float angZ = json["angular"]["z"] | 0.0f;

  float left = linX - angZ * WHEEL_DIST * 0.5f;
  float right = linX + angZ * WHEEL_DIST * 0.5f;

  const auto m = abs(max(left, right));
  if (m > MAX_WHEEL_SPEED)
  {
    left *= MAX_WHEEL_SPEED / m;
    right *= MAX_WHEEL_SPEED / m;
  }

  steppersSetSpeed(left, right);
}

static void handleBatteryEnable(JsonObjectConst json)
{
  uint32_t secs = json["data"] | 0u;
  batterySetWithTimeoutMs(secs * 1000);
}

static void handleCalibrateAccelGyro()
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
      accBias[i] += mpu.getAccBias(i);

    for (int i = 0; i < 3; i++)
      gyroBias[i] += mpu.getGyroBias(i);
  }

  for (int i = 0; i < 3; i++)
    accBias[i] /= numTries;

  for (int i = 0; i < 3; i++)
    gyroBias[i] /= numTries;

  char stringbuf[120];
  snprintf(stringbuf, sizeof(stringbuf),
           "acc bias: %f, %f, %f\n gyro bias: %f, %f, %f",
           accBias[0], accBias[1], accBias[2],
           gyroBias[0], gyroBias[1], gyroBias[2]);
  Serial.println(stringbuf);
}

// Odometry state (file-level so it can be reset externally)
static double odomPosX = 0;
static double odomPosY = 0;
static double odomYaw = 0;
static double odomOldStepsLeft = 0;
static double odomOldStepsRight = 0;

static void handleResetOdometry()
{
  odomPosX = 0;
  odomPosY = 0;
  odomYaw = 0;
  Serial.println("Odometry reset");
}


// ---------------------------------------------------------------------------
// Parse an incoming serial line
// ---------------------------------------------------------------------------

static void processLine(const char* line)
{
  // Handle "list_subs" command
  if (strcmp(line, "list_subs") == 0)
  {
    SerialUSB.println("subscriptions ["
      "{\"cmd_vel\": \"geometry_msgs/msg/Twist\"}, "
      "{\"base_battery_enable_for_sec\": \"std_msgs/msg/UInt32\"}, "
      "{\"imu/calibrate_accel_gyro\": \"std_msgs/msg/Empty\"}, "
      "{\"base_reset_odometry\": \"std_msgs/msg/Empty\"}"
      "]");
    return;
  }

  // Handle "timestamp <sec> <nsec>" command
  if (strncmp(line, "timestamp ", 10) == 0)
  {
    uint32_t sec = 0, nsec = 0;
    if (sscanf(line + 10, "%lu %lu", &sec, &nsec) >= 1)
    {
      uint64_t hostMs = (uint64_t)sec * 1000ULL + (uint64_t)nsec / 1000000ULL;
      millisOffset = hostMs - (uint64_t)millis();
      Serial.print("Time synced, offset: ");
      Serial.println((long)millisOffset);
    }
    return;
  }

  // Format: "pub <topic> <type> <json>"
  // Expect "pub" as first word
  if (strncmp(line, "pub ", 4) != 0) return;
  const char* rest = line + 4;

  // Find first space -> topic
  const char* p1 = strchr(rest, ' ');
  if (!p1) return;

  size_t topicLen = p1 - rest;
  char topic[64];
  if (topicLen >= sizeof(topic)) return;
  memcpy(topic, rest, topicLen);
  topic[topicLen] = '\0';

  // Find second space -> type (we skip type for dispatch, but need to find json start)
  const char* p2 = strchr(p1 + 1, ' ');
  if (!p2) return;

  const char* jsonStr = p2 + 1;

  if (strcmp(topic, "cmd_vel") == 0)
  {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, jsonStr);
    if (err)
    {
      Serial.print("cmd_vel JSON parse error: ");
      Serial.println(err.c_str());
      return;
    }
    handleCmdVel(doc.as<JsonObjectConst>());
  }
  else if (strcmp(topic, "base_battery_enable_for_sec") == 0)
  {
    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, jsonStr);
    if (err)
    {
      Serial.print("battery_enable JSON parse error: ");
      Serial.println(err.c_str());
      return;
    }
    handleBatteryEnable(doc.as<JsonObjectConst>());
  }
  else if (strcmp(topic, "imu/calibrate_accel_gyro") == 0)
  {
    // Empty message, no JSON payload needed
    handleCalibrateAccelGyro();
  }
  else if (strcmp(topic, "base_reset_odometry") == 0)
  {
    // Empty message, no JSON payload needed
    handleResetOdometry();
  }
  else
  {
    Serial.print("Unknown topic: ");
    Serial.println(topic);
  }
}

// ---------------------------------------------------------------------------
// Read incoming serial data, process complete lines
// ---------------------------------------------------------------------------

static void readSerial()
{
  while (SerialUSB.available())
  {
    char c = SerialUSB.read();
    if (c == '\n' || c == '\r')
    {
      if (inputLen > 0)
      {
        inputBuf[inputLen] = '\0';
        processLine(inputBuf);
        inputLen = 0;
      }
    }
    else
    {
      if (inputLen < SERIAL_BUF_SIZE - 1)
      {
        inputBuf[inputLen++] = c;
      }
      else
      {
        // Line too long, discard
        Serial.println("Input line too long, discarding");
        inputLen = 0;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Periodic update functions
// ---------------------------------------------------------------------------

static void update_imu()
{
  if (mpu.update())
  {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25)
    {
      uint64_t now = syncedMillis();
      publishImu("imu",
                  (uint32_t)(now / 1000),
                  (uint32_t)((now % 1000) * 1000000),
                  mpu.getQuaternionX(),
                  -mpu.getQuaternionY(),
                  -mpu.getQuaternionZ(),
                  mpu.getQuaternionW(),
                  mpu.getGyroX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY,
                  mpu.getGyroY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY,
                  mpu.getGyroZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY,
                  mpu.getLinearAccX(),
                  mpu.getLinearAccY(),
                  mpu.getLinearAccZ() + 9.81F);

      prev_ms = millis();
    }
  }
}

static void update_battery_voltage()
{
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + 1000)
  {
    uint64_t now = syncedMillis();
    publishBatteryState(
      (uint32_t)(now / 1000),
      (uint32_t)((now % 1000) * 1000000),
      batteryReadVoltage(),
      batteryGetState());

    prev_ms = millis();
  }
}

static void update_odometry()
{

  static uint32_t prev_ms = millis();
  const auto dMillis = millis() - prev_ms;
  if (dMillis > 40)
  {
    double left, right;
    steppersGetDist(left, right);
    const auto dL = left - odomOldStepsLeft;
    const auto dR = right - odomOldStepsRight;
    odomOldStepsLeft = left;
    odomOldStepsRight = right;

    const auto dDist = (dL + dR) / 2.0;
    const auto dYaw = (dR - dL) / WHEEL_DIST;

    odomPosX += dDist * cos(odomYaw + dYaw / 2.0);
    odomPosY += dDist * sin(odomYaw + dYaw / 2.0);
    odomYaw += dYaw;
    if (odomYaw > PI)
      odomYaw -= 2 * PI;
    else if (odomYaw < -PI)
      odomYaw += 2 * PI;

    const auto dt = dMillis * 0.001;
    const auto speed = dDist / dt;
    const auto rotSpeed = dYaw / dt;

    double covariance[36] = {0};
    covariance[0] = 0.03;
    covariance[7] = 0.03;
    covariance[14] = 99999;
    covariance[21] = 99999;
    covariance[28] = 99999;
    covariance[35] = 0.03;

    uint64_t now = syncedMillis();
    publishOdometry(
      (uint32_t)(now / 1000),
      (uint32_t)((now % 1000) * 1000000),
      odomPosX, odomPosY, odomYaw,
      speed, rotSpeed,
      covariance);

    publishTfOdom(
      (uint32_t)(now / 1000),
      (uint32_t)((now % 1000) * 1000000),
      odomPosX, odomPosY, odomYaw);

    prev_ms = millis();
  }
}

// ---------------------------------------------------------------------------
// Arduino setup & loop
// ---------------------------------------------------------------------------

void setup()
{
  batteryInit();
  steppersInit();

  Serial.begin(115200);
  Serial.println("Initializing...");

  SerialUSB.begin(1000000);

  analogReadResolution(12);


  Wire.begin();

  pinMode(LED_BUILTIN, OUTPUT);

//   if (!mpu.setup(0x68))
//   {
//     Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
//   }

//   mpu.setAccBias(-40, 266, 95);
//   mpu.setGyroBias(154, 138, 13.5);
//   mpu.setMagBias(161.59, 82.50, 47.44);
//   mpu.setMagScale(0.80, 0.79, 2.08);

  Serial.println("Setup complete!");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);

  watchdogEnable(1000);
}

void loop()
{
  watchdogReset();

  readSerial();

  batteryCheckTimeout();
  stepperCheckTimeout();
  // update_imu();
  update_battery_voltage();
  update_odometry();

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

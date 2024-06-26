// Copyright (c) 2023 Charles Nguyen

// Permission is hereby granted, free of charge, to any person obtaining a copy

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
#include "main.hpp"
#include "RwStatus.hpp"
#include "Controller.hpp"
#include "PointingModes.hpp"

/* Setup functions */
// Initializes serial monitor
static void SetupSerial();
// Initializes motor pwm and direction pins
static void SetupMotors();
// Initializes the BNO085
static void SetupImu();
// Initializes SD card reader
// Only necessary for testing. Should not exist in finished system
static void SetupSd();
// Initializes interrupt pins and timer
static void SetupRpm();

rw_status::RwStatus wheel_status;
controller::QuaternionPD qpd(1, 0);
// controller::WheelSpeedPD init_wpd(1e-2, 0);
controller::WheelSpeedPD wpd(1e-3, 0);
pointing_modes::FourWheelMode four;

/* Loop functions */
// Itializes system time
static void UpdateSysTime();
// Reads the latest information from the imu.
// Returns the attitude quaternion reading in q and angular velocity in v.
static void ReadImu(imu::Quaternion& q, imu::Vector<3>& v);

void setup() {
  SetupSerial();
  SetupMotors();
  SetupImu();
  SetupSd();
  SetupRpm();

  timer::init_time = millis();
  timer::loop_start_time = timer::init_time;
}

int counter = 0;
void loop() {
  UpdateSysTime();

  imu::Quaternion q;
  imu::Vector<3> v;
  ReadImu(q, v);

  imu::Quaternion q_desired(0.0, 0.0, 0.0, 1.0);
  imu::Vector<3> torque_req = qpd.Compute(q_desired, q, v);
  float wheel_torques[4];
  uint8_t pwm[4];
  Serial.print("q: ");
  imu::Quaternion qe = q_desired.conjugate() * q;
  Serial.print(q.w());
  Serial.print(", ");
  Serial.print(q.x());
  Serial.print(", ");
  Serial.print(q.y());
  Serial.print(", ");
  Serial.println(q.z());
  // Serial.print("wheel_torques: ");
  // Serial.println(wheel_torques[0]);
  // Serial.print("dt: ");
  // Serial.println(timer::loop_dt);
  // Serial.print("calc: ");
  // Serial.println(wheel_torques[0] / four.kWheelMoment[0] * timer::loop_dt);
  // Serial.println(interrupt::wheel_rpm[0]);

  if (counter < 200) {
    float test[] = {15000, 15000, 15000, 15000};
    four.Test_Speed_Command(test, interrupt::wheel_rpm, timer::loop_dt, wpd, pwm);
    counter++;
    Serial.println("init speeds");
  } else {
    four.Calculate(torque_req, wheel_torques);
    four.Pid_Speed(wheel_torques, timer::loop_dt, wpd, interrupt::wheel_rpm, pwm);
  }


  for (int i = 0; i < 4; i++) {
    digitalWrite(physical::kDirectionPins[i], 1);
    analogWrite(physical::kPwmPins[i], abs(pwm[i]));
  }

  // TODO: use wheel_status

  // Serial.println(interrupt::wheel_rpm[0]);
  // Serial.println(interrupt::wheel_rpm[1]);
  // Serial.println(interrupt::wheel_rpm[2]);
  // Serial.println(interrupt::wheel_rpm[3]);
  // Serial.println(v[0]);
  // Serial.println(v[1]);
  // Serial.println(v[2]);
  Serial.println(pwm[0]);
  Serial.println(pwm[1]);
  Serial.println(pwm[2]);
  Serial.println(pwm[3]);
  // Serial.println(torque_req[0]);
  // Serial.println(torque_req[1]);
  // Serial.println(torque_req[2]);
  // Serial.println(wheel_torques[0]);
  // Serial.println(wheel_torques[1]);
  // Serial.println(wheel_torques[2]);
  // Serial.println(wheel_torques[3]);
  // Serial.println(q.w());
  // Serial.println(q.x());
  // Serial.println(q.y());
  // Serial.println(q.z());
  Serial.println("");
}

/* Setup */
static void SetupSerial() {
  Serial.begin(physical::kSerialRate);
  while (!Serial) {}  // wait for Serial
}
static void SetupMotors() {
  // init motor pins
  for (int i = 0; i < physical::kNumWheels; i++) {
    pinMode(physical::kPwmPins[i], OUTPUT);
    pinMode(physical::kDirectionPins[i], OUTPUT);
  }
}
static void SetupImu() {
  // TODO anything here failing is pretty bad. It would be impossible for both
  // reaction wheels and magnetorquers to have functionality.
  if (!physical::bno.begin_I2C()) {
    Serial.print("No BNO085 detected");
    exit(EXIT_FAILURE);
  }
  // GAME_ROTATION_VECTOR has no magnetometer input, so it's more applicable
  // to HS3. Consider making it absolute orientation (respective to magentic
  // north) and doing math to get a relative orientation for satellites in a
  // magnetic field.
  // It gives values in quaternion form.
  if (!physical::bno.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
  // SH2_GYROSCOPE_CALIBRATED gives velocity for the x,y,z axies.
  // It includes a bias for compensation that can be separated with
  // SH2_GYROSCOPE_UNCALIBRATED
  if (!physical::bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable game vector");
  }
}
static void SetupSd() {
  // init SD reader
  pinMode(physical::kSdPin, OUTPUT);
  digitalWrite(physical::kSdPin, HIGH);
  pinMode(SS, OUTPUT);
  if (!SD.begin(physical::kSdPin)) {
    Serial.println("card failed or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  physical::file = SD.open("data.csv", FILE_WRITE);
  if (physical::file) {
    physical::file.println("setpoint,rpm0,error,send pwm");
  } else {
    Serial.println("card failed write");
    physical::file.close();
    exit(EXIT_FAILURE);
  }
}
static void SetupRpm() {
  for (int i = 0; i < physical::kNumWheels; i++) {
  pinMode(physical::kFgPins[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(physical::kFgPins[0]), interrupt::ReadRpm0, FALLING);
  attachInterrupt(digitalPinToInterrupt(physical::kFgPins[1]), interrupt::ReadRpm1, FALLING);
  attachInterrupt(digitalPinToInterrupt(physical::kFgPins[2]), interrupt::ReadRpm2, FALLING);
  attachInterrupt(digitalPinToInterrupt(physical::kFgPins[3]), interrupt::ReadRpm3, FALLING);

  // RPM timers
  // wheels start at 0, which relative to internal is this
  for (int i = 0; i < physical::kNumWheels; i++) { interrupt::wheel_rpm[i] = -interrupt::kZeroRpm; }

  // setup global timer
  interrupt::global_time = 0;
  ITimer3.init();
  if (ITimer3.attachInterrupt(interrupt::kGlobalRate, interrupt::TimerHandler)) {
    Serial.println("Starting ITimer3 OK, millis() = " + String(millis()));
  } else {
    Serial.println("Can't set ITimer3. Select another freq. or timer");
  }
}

/* Loop */
static void UpdateSysTime() {
  timer::loop_prev_start_time = timer::loop_start_time;
  timer::loop_start_time = millis();
  timer::loop_dt = timer::loop_start_time - timer::loop_prev_start_time;
}
static void ReadImu(imu::Quaternion& q, imu::Vector<3>& v) {
  sh2_SensorValue_t sensor_value;
  if(!physical::bno.getSensorEvent(&sensor_value)) {
    Serial.println("bno085 not responsive");
  }

  switch (sensor_value.sensorId) {
    case SH2_GAME_ROTATION_VECTOR:
      q = {sensor_value.un.gameRotationVector.real,
        sensor_value.un.gameRotationVector.i,
        sensor_value.un.gameRotationVector.j,
        sensor_value.un.gameRotationVector.k};
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      v = {sensor_value.un.gyroscope.x, sensor_value.un.gyroscope.y,
        sensor_value.un.gyroscope.z};
  }
}

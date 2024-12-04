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
// Initializes interrupt pins and timer
static void SetupRpm();

static void write_PWM(uint8_t PWMs[4]);
// writes the pwm values to appropriate pins, defined out for readability

rw_status::RwStatus wheel_status;
controller::QuaternionPD QuaternionTorque_PD(test_parameters::torque_PD_params[0], test_parameters::torque_PD_params[1]);
controller::WheelSpeedPD WheelSpeed_PD(test_parameters::wheel_speed_PD_params[0], test_parameters::wheel_speed_PD_params[1]);
pointing_modes::FourWheelMode WheelController;

/* Loop functions */
// Initializes system time
static void UpdateSysTime();
// Reads the latest information from the imu.
// Returns the attitude quaternion reading in q and angular velocity in v.
static void ReadImu(imu::Quaternion &q, imu::Vector<3> &v);

void setup()
{

  SetupSerial();
  SetupMotors();
  SetupImu();
  // SetupSd();
  SetupRpm();

  Serial.write(serial_stuff::MCU_init_phrase);

  delay(test_parameters::test_delay * 1000);

  timer::init_time = millis();
  timer::loop_start_time = timer::init_time;

  test_parameters::timeout_sec = (test_parameters::timeout_sec * 1000) + timer::init_time;
}

imu::Quaternion target_quaternion(0, 0, 0, 0);
float target_RPM[4] = {0, 0, 0, 0};
bool is_using_quaternion = true;

static bool activated = false;

static int counter = 0;
void loop()
{

  if (millis() > test_parameters::timeout_sec)
  {
    // this completely halts the program and stops it
    //  the intent is that if a test goes wrong the test will stop on its own after an amount of time
    while (true)
    {
      delay(10);
    }
  }

  /*
  This short block decides whether or not the current loop is one where values should be printed
  This is its own block because different print functions need to occur at different places, and it saves having to do the calculation multiple different times.
  ie if you want to print both target quaternion and target rpm, those don't exist at the same time so you need separate print blocks for each, and having the should_serial flag improves readability
  */
  bool should_serial = false;
  if (counter % serial_stuff::cycles_per_print == 0)
  {
    should_serial = true;
  }

  UpdateSysTime();
  counter++;

  serial_stuff::pull_from_serial();
  if (serial_stuff::unparsed_from_serial == true)
  {
    switch (serial_stuff::current_command)
    {
    case 0: // "stop"
      activated = static_cast<bool>(serial_stuff::current_argument[0]);
      break;
    case 10: //  "set_quaternion"
      serial_stuff::FLOATUNION_t current_quaternion_float[4];
      memcpy(current_quaternion_float, serial_stuff::current_argument, 16);
      target_quaternion = imu::Quaternion(current_quaternion_float[0].number, current_quaternion_float[1].number, current_quaternion_float[2].number, current_quaternion_float[3].number);

      is_using_quaternion = true;
      break;
    case 20: //  "set_rpm"
      memcpy(target_RPM, serial_stuff::current_argument, 16);

      is_using_quaternion = false;
      break;
    }

    serial_stuff::unparsed_from_serial = false;
  }

  // The loop reads the IMU every cycle regardless of whether
  imu::Quaternion current_quaternion;
  imu::Vector<3> current_gyro_reading;
  ReadImu(current_quaternion, current_gyro_reading);

  if (should_serial)
  {
    //  11,  "get_quaternion"
    serial_stuff::write_quaternion_to_serial(11, current_quaternion);

    //  21, "get_rpm"

    float temp_rpm[4];

    for (int i = 0; i < 4; i++)
    {
      temp_rpm[i] = interrupt::wheel_rpm[i];
    }

    serial_stuff::write_4_floats_to_serial(21, temp_rpm);
  }

  if (activated)
  {
    uint8_t pwm_values[4] = {0, 0, 0, 0}; // creating pwm values

    if (is_using_quaternion)
    {
      imu::Vector<3> required_torque = QuaternionTorque_PD.Compute(target_quaternion, current_quaternion, current_gyro_reading);
      float required_wheel_torques[4];

      WheelController.Calculate(required_torque, required_wheel_torques);
      WheelController.Pid_Speed(required_wheel_torques, timer::loop_dt, WheelSpeed_PD, interrupt::wheel_rpm, pwm_values);
    }
    else
    {
      WheelController.Test_Speed_Command(target_RPM, interrupt::wheel_rpm, timer::loop_dt, WheelSpeed_PD, pwm_values);
    }
    write_PWM(pwm_values);

    if (should_serial)
    {
      Serial.write(serial_stuff::start_byte);

      //  30, "get_pwm"
      Serial.write(static_cast<byte>(30));
      for (int i = 0; i < 4; i++)
      {
        Serial.write(pwm_values[i]);
      }
      Serial.write(serial_stuff::stop_byte);
    }
  }
}

/* Setup */
static void SetupSerial()
{
  Serial.begin(serial_stuff::kSerialRate);
  while (!Serial)
  {
  } // wait for Serial
}
static void SetupMotors()
{
  // init motor pins
  for (int i = 0; i < physical::kNumWheels; i++)
  {
    pinMode(physical::kPwmPins[i], OUTPUT);
    pinMode(physical::kDirectionPins[i], OUTPUT);
  }
}
static void SetupImu()
{
  // TODO anything here failing is pretty bad. It would be impossible for both
  // reaction wheels and magnetorquers to have functionality.
  if (!physical::bno.begin_I2C())
  {
    Serial.print("No BNO085 detected");
    exit(EXIT_FAILURE);
  }
  // GAME_ROTATION_VECTOR has no magnetometer input, so it's more applicable
  // to HS3. Consider making it absolute orientation (respective to magnetic
  // north) and doing math to get a relative orientation for satellites in a
  // magnetic field.
  // It gives values in quaternion form.
  if (!physical::bno.enableReport(SH2_GAME_ROTATION_VECTOR))
  {
    Serial.println("Could not enable game vector");
  }
  // SH2_GYROSCOPE_CALIBRATED gives velocity for the x,y,z axes.
  // It includes a bias for compensation that can be separated with
  // SH2_GYROSCOPE_UNCALIBRATED
  if (!physical::bno.enableReport(SH2_GYROSCOPE_CALIBRATED))
  {
    Serial.println("Could not enable game vector");
  }
}

static void SetupRpm()
{
  for (int i = 0; i < physical::kNumWheels; i++)
  {
    pinMode(physical::kFgPins[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(physical::kFgPins[0]), interrupt::ReadRpm0, FALLING);
  attachInterrupt(digitalPinToInterrupt(physical::kFgPins[1]), interrupt::ReadRpm1, FALLING);
  attachInterrupt(digitalPinToInterrupt(physical::kFgPins[2]), interrupt::ReadRpm2, FALLING);
  attachInterrupt(digitalPinToInterrupt(physical::kFgPins[3]), interrupt::ReadRpm3, FALLING);

  // RPM timers
  // wheels start at 0, which relative to internal is this
  for (int i = 0; i < physical::kNumWheels; i++)
  {
    interrupt::wheel_rpm[i] = -interrupt::kZeroRpm;
  }

  // setup global timer
  interrupt::global_time = 0;
  ITimer3.init();
  if (!(ITimer3.attachInterrupt(interrupt::kGlobalRate, interrupt::TimerHandler)))
  {
    Serial.println("Can't set ITimer3. Select another freq. or timer");
  }
}

/* Loop */
static void UpdateSysTime()
{
  timer::loop_prev_start_time = timer::loop_start_time;
  timer::loop_start_time = millis();
  timer::loop_dt = timer::loop_start_time - timer::loop_prev_start_time;
}
static void ReadImu(imu::Quaternion &q, imu::Vector<3> &v)
{
  /*
  One pretty fundamental question I have here is that I'm pretty sure this function leaves either 'q' or 'v' in its default unassigned state.
  In loop(), q and v are unassigned
  */
  // so these few lines are pretty self explanatory: create new sensor_value struct, and then read the imu, and if the read fails print an error
  sh2_SensorValue_t sensor_value;
  if (!physical::bno.getSensorEvent(&sensor_value))
  {
    Serial.println("bno085 not responsive");
  }

  // What I don't understand here the 'sensor_value.sensorId' can only be one number (right?), so EITHER case 1 will be true and q will be assigned OR case 2 will be true and v will be assigned
  switch (sensor_value.sensorId)
  {
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
static void write_PWM(uint8_t PWMs[4])
{
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(physical::kDirectionPins[i], 1);
    analogWrite(physical::kPwmPins[i], abs(PWMs[i]));
  }
}

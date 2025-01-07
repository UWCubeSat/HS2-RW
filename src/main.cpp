// Copyright (c) 2023 Charles Nguyen

// Permission is hereby granted, free of charge, to any person obtaining a copy

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
#include "main.hpp"
#include "RwStatus.hpp"
#include "Controller.hpp"
#include "PointingModes.hpp"
#include "testing.hpp"

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

static void write_PWM(uint8_t PWMs[4]);
// writes the pwm values to appropriate pins, defined out for readability

rw_status::RwStatus wheel_status;
controller::QuaternionPD QuaternionTorque_PD(test_parameters::torque_PD_params[0], test_parameters::torque_PD_params[1]);
// controller::WheelSpeedPD init_WheelSpeed_PD(1e-2, 0);
controller::WheelSpeedPD WheelSpeed_PD(test_parameters::wheel_speed_PD_params[0], test_parameters::wheel_speed_PD_params[1]);
pointing_modes::FourWheelMode WheelController;

/* Loop functions */
// Initializes system time
static void UpdateSysTime();
// Reads the latest information from the imu.
// Returns the attitude quaternion reading in q and angular velocity in v.
// static void ReadImu(imu::Quaternion &q, imu::Vector<3> &v);

static void print_float_array(float *array, int array_len, const char *array_identifier);

long timeout_ms;
void setup()
{
  SetupSerial();
  Serial.println("begin setup");

  SetupMotors();
  /*
  these two are commented out because currently testing only with arduino
  */

  SetupImu();
  /* SetupSd();
   */
  SetupRpm();
  Serial.println("setup successful!");

  Serial.println("starting testing delay");
  delay((long)(test_parameters::test_delay_secs) * 1000);
  Serial.println("testing delay over");

  timer::init_time = millis();
  timer::current_loop_start_time = timer::init_time;
  test_parameters::test_init_time_ms = timer::init_time;

  timeout_ms = (test_parameters::timeout_secs * 1000) + timer::init_time;
}

bool should_serial = false;
void loop()
{
  UpdateSysTime();

  if (timer::current_loop_start_time > ((test_parameters::timeout_secs * 1000) + timer::init_time))
  {
    // this completely halts the program and stops it
    // the intent is that if a test goes wrong the test will stop on its own after an amount of time
    Serial.println("test timeout exceeded");
    Serial.flush();

    exit(EXIT_SUCCESS);
  }

  /*
  This short block decides whether or not the current loop is one where values should be printed
  This is its own block because different print functions need to occur at different places, and it saves having to do the calculation multiple different times.
  ie if you want to print both target quaternion and target rpm, those don't exist at the same time so you need separate print blocks for each, and having the should_serial flag improves readability
  */
  if (timer::counter % test_parameters::cycles_per_print == 0)
  {
    should_serial = true;
  }
  else
  {
    should_serial = false;
  }

  // The loop reads the IMU every cycle regardless of whether
  imu::Quaternion current_quaternion = physical::bno.getQuat();
  imu::Vector<3> current_gyro_reading = physical::bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // ReadImu(current_quaternion, current_gyro_reading);//- left commented out because currently testing only with arduino

  if (should_serial)
  {
    if (test_parameters::print_current_quaternion)
    {
      float quaternion_array[4] = {current_quaternion.w(), current_quaternion.x(), current_quaternion.y(), current_quaternion.z()};
      print_float_array(quaternion_array, 4, "Current Quaternion");
    }
    if (test_parameters::print_current_RPM)
    {
      float rpm_array[4] = {interrupt::wheel_rpm[0], interrupt::wheel_rpm[1], interrupt::wheel_rpm[2], interrupt::wheel_rpm[3]};

      print_float_array(rpm_array, sizeof(interrupt::wheel_rpm) / sizeof(interrupt::wheel_rpm[0]), "Current RPMs");
    }
  }
  // imu::Quaternion qe = test_parameters::target_quaternion.conjugate() * current_quaternion;

  // this block of code is responsible for the testing logic
  if ((test_parameters::list_of_tests[test_parameters::test_index].is_indefinite == true) || (test_parameters::list_of_tests[test_parameters::test_index].delay_time_secs * 1000) > (timer::current_loop_start_time - test_parameters::test_init_time_ms))

  // this if statement checks if either the current test's time hasn't elapsed or whether the current test has the indefinite value set to true, if yes to either the test changes
  {
    /*if a test is going on, this codeblock runs*/

    uint8_t pwm_values[4] = {0, 0, 0, 0}; // creating pwm values

    if (test_parameters::list_of_tests[test_parameters::test_index].is_using_quaternion == true)
    // if test is quaternion control
    {
      // get target quaternion from list_of_tests
      imu::Quaternion target_quaternion(test_parameters::list_of_tests[test_parameters::test_index].test_value[0], test_parameters::list_of_tests[test_parameters::test_index].test_value[1], test_parameters::list_of_tests[test_parameters::test_index].test_value[2], test_parameters::list_of_tests[test_parameters::test_index].test_value[3]);
      // calculate required torque from previously calculated wheel torques
      imu::Vector<3> required_torque = QuaternionTorque_PD.Compute(target_quaternion, current_quaternion, current_gyro_reading);

      float required_wheel_torques[4];

      WheelController.Calculate(required_torque, required_wheel_torques);
      WheelController.Pid_Speed(required_wheel_torques, timer::loop_dt, WheelSpeed_PD, interrupt::wheel_rpm, pwm_values);

      if (should_serial && test_parameters::print_target_quaternion)
      {
        print_float_array(test_parameters::list_of_tests[test_parameters::test_index].test_value, 4, "Target Quaternion");
      }
    }
    else
    {
      // If not quaternion control, then it's rpm control

      // Calculate wheel PWM's
      WheelController.Test_Speed_Command(test_parameters::list_of_tests[test_parameters::test_index].test_value, interrupt::wheel_rpm, timer::loop_dt, WheelSpeed_PD, pwm_values);

      if (should_serial && test_parameters::print_target_RPM)
      {
        print_float_array(test_parameters::list_of_tests[test_parameters::test_index].test_value, 4, "Target RPM");
      }
    }

    write_PWM(pwm_values);

    if (should_serial && test_parameters::print_current_PWM)
    {
      Serial.print("PWMs:");
      for (int i = 0; i < 4; i++)
      {
        Serial.print(pwm_values[i]);
        Serial.print(", ");
      }
      Serial.print("\n");
    }
  }
  else
  {
    // if the current test is over, then either we advance to the next test, or there are no more tests, and the program is over

    if (test_parameters::test_index < test_parameters::number_of_tests)
    {
      // if the current test is over, update the index to the next test, and set the next test start time as the current time

      Serial.print("test ");
      Serial.print(test_parameters::test_index);
      Serial.print(" over, beginning with test ");

      test_parameters::test_index++;
      test_parameters::test_init_time_ms = timer::current_loop_start_time;

      Serial.print(test_parameters::test_index);
      Serial.println(" (0 indexed)");
    }
    else
    {
      // the program has no more tests, so after the last test times out, then the code will end up here, which is an empty block and nothing will happen
    }
  }
}

/* Setup */
static void SetupSerial()
{
  Serial.begin(physical::kSerialRate);
  while (!Serial)
  {
    delay(10);
  } // wait for Serial
}
static void SetupMotors()
{
  // init motor pins
  for (int i = 0; i < physical::kNumWheels; i++)
  {
    pinMode(physical::kPwmPins[i], OUTPUT);
    digitalWrite(physical::kPwmPins[i], LOW);

    pinMode(physical::kDirectionPins[i], OUTPUT);
    digitalWrite(physical::kDirectionPins[i], LOW);
  }
}
static void SetupImu()
{
  // TODO anything here failing is pretty bad. It would be impossible for both
  // reaction wheels and magnetorquers to have functionality.
  if (physical::bno.begin() == false)
  {
    Serial.print("No BNO085 detected");
    Serial.flush(); // flush here stops the message from not fully printing

    exit(EXIT_FAILURE);
  }
  /*
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
  */
}
static void SetupSd()
{
  // init SD reader
  pinMode(physical::kSdPin, OUTPUT);
  digitalWrite(physical::kSdPin, HIGH);
  pinMode(SS, OUTPUT);
  if (!SD.begin(physical::kSdPin))
  {
    Serial.println("card failed or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  physical::file = SD.open("data.csv", FILE_WRITE);
  if (physical::file)
  {
    physical::file.println("setpoint,rpm0,error,send pwm");
  }
  else
  {
    Serial.println("card failed write");
    Serial.flush();

    physical::file.close();
    exit(EXIT_FAILURE);
  }
}
static void SetupRpm()
{
  for (int i = 0; i < physical::kNumWheels; i++)
  {
    pinMode(physical::kFgPins[i], INPUT_PULLUP);
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
  if (ITimer3.attachInterrupt(interrupt::kGlobalRate, interrupt::TimerHandler))
  {
    Serial.println("Starting ITimer3 OK, millis() = " + String(millis()));
  }
  else
  {
    Serial.println("Can't set ITimer3. Select another freq. or timer");
  }
}

/* Loop */
static void UpdateSysTime()
{
  timer::prev_loop_start_time = timer::current_loop_start_time;
  timer::current_loop_start_time = millis();
  timer::loop_dt = timer::current_loop_start_time - timer::prev_loop_start_time;
  timer::counter++;
}
/*
static void ReadImu(imu::Quaternion &q, imu::Vector<3> &v)
{

  One pretty fundamental question I have here is that I'm pretty sure this function leaves either 'q' or 'v' in its default unassigned state.
  In loop(), q and v are unassigned

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
}*/
static void write_PWM(uint8_t PWMs[4])
{
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(physical::kDirectionPins[i], 1);
    analogWrite(physical::kPwmPins[i], abs(PWMs[i]));
  }
}
static void print_float_array(float *array, int array_len, const char *array_identifier)
{

  Serial.print(array_identifier);
  Serial.print(": ");

  for (int i = 0; i < array_len; i++)
  {
    Serial.print(*(array + i));

    (i == array_len - 1) ? Serial.println("") : Serial.print(", ");
  }
}

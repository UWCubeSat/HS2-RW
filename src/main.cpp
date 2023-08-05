#include "main.hpp"
#include "RwStatus.hpp"

/* Setup functions */
// initializes serial monitor
static void SetupSerial();
// initializes motor pwm and direction pins
static void SetupMotors();
// initializes SD card reader
static void SetupSd();
// initializes interrupt pins and timer
static void SetupRpm();

rw_status::RwStatus wheel_status;

/* Loop functions */
// initializes system time
static void UpdateSysTime();

void setup() {
  SetupSerial();
  SetupMotors();
  SetupSd();
  SetupRpm();

  timer::init_time = millis();
}

void loop() {
  UpdateSysTime();
}

void SetupSerial() {
  Serial.begin(physical::kSerialRate);
  while (!Serial);  // wait for Serial
}
void SetupMotors() {
  // init motor pins
  for (int i = 0; i < physical::kNumWheels; i++) {
    pinMode(physical::kPwmPins[i], OUTPUT);
    pinMode(physical::kDirectionPins[i], OUTPUT);
  }
}
void SetupSd() {
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
void SetupRpm() {
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

void UpdateSysTime() {
  timer::loop_prev_start_time = timer::loop_start_time;
  timer::loop_start_time = millis();
  timer::loop_dt = timer::loop_prev_start_time - timer::loop_start_time;
}
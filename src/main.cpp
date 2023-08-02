#include "Arduino.h"

// These define's must be placed at the beginning before #include "TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#define USE_TIMER_1     false
#define USE_TIMER_2     false
#define USE_TIMER_3     true
#define USE_TIMER_4     false
#define USE_TIMER_5     false

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "TimerInterrupt.h"           //https://github.com/khoih-prog/TimerInterrupt

// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "ISR_Timer.h"                //https://github.com/khoih-prog/TimerInterrupt

#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

const uint8_t kPwmPins[4] = {3, 8, 9, 11};
// const uint8_t kPwmPins[4] = {3, 9, 10, 11};
uint8_t direction_pins[4] = {5, 6, 12, 13};

uint8_t pwmVals[4] = {0, 0, 0, 0};  // 0-255
uint8_t directionVals[4] = {1, 1, 1, 1};  // 1 is forward, 0 is reverse

// SD
#include "SD.h"
const unsigned int chip_select = 53;
File file;

int init_time;
long start_time;
long prev_start_time;
long loop_dt;
long prev_dir_switch_time;
long prev_update_rpm_time;

static constexpr float kZeroRpm = 5500;
static constexpr float kMinRpm = 1000 - kZeroRpm;  // actual min - kZero
static constexpr float kMaxRpm = 10000 - kZeroRpm; // actual max - kZero
float desired_rpm;
static constexpr float kP = 0.005; 
static constexpr float kD = 0;
float prev_error;

int curr_pwm;
int8_t change_direction;

volatile float rpm0;
static constexpr float kGlobalRate = 10000;
static constexpr float kRotPerFG = 0.5;
volatile unsigned long global_time;  // 0.1 ms
volatile unsigned long prev_time;  // 0.1 ms
void TimerHandler() {
  global_time++;
}
void ReadRpm() {
  rpm0 = (60.0 * kRotPerFG * kGlobalRate / (float) (global_time - prev_time)) - kZeroRpm;
  prev_time = global_time;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  // pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(18, INPUT);
  attachInterrupt(digitalPinToInterrupt(18), ReadRpm, FALLING);

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  pinMode(SS, OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("1Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("2card initialized.");
  file = SD.open("data.csv", FILE_WRITE);
  if (file) {
    file.println("setpoint,rpm0,error,send pwm");
  } else {
    Serial.println("4card failed write");
    file.close();
    exit(1);
  }

  rpm0 = -kZeroRpm;
  global_time = 0;
  prev_time = 0;

  start_time = 0;
  prev_start_time = 0;
  loop_dt = 0;
  prev_dir_switch_time = 0;
  desired_rpm = 0;
  desired_rpm = 0;
  prev_error = 0;
  curr_pwm = 0;
  change_direction = 1;

  ITimer3.init();
  if (ITimer3.attachInterrupt(kGlobalRate, TimerHandler)) {
    Serial.println("Starting ITimer3 OK, millis() = " + String(millis()));
  } else {
    Serial.println("Can't set ITimer3. Select another freq. or timer");
  }

  init_time = millis();
}

void loop() {
  if (millis() - init_time > 60000) {
    analogWrite(kPwmPins[0], 0);
    file.close();
    Serial.println("10done!");
    exit(0);
  }

  start_time = millis();
  loop_dt = prev_start_time - start_time;  // in ms
  prev_start_time = start_time;

  float curr_error = desired_rpm - rpm0;
  float delta_pwm;
  if (loop_dt == 0) {  // check delta_time not 0
    delta_pwm = kP*curr_error;
  } else {
    delta_pwm = kP*curr_error + kD*(curr_error - prev_error)/loop_dt;
  }

  curr_pwm += delta_pwm;
  curr_pwm = constrain(curr_pwm, 0, 255);
  prev_error = curr_error;

  // switches setpoint direction for testing
  if (start_time - prev_dir_switch_time > 2500) {
    if ((desired_rpm <= kMinRpm && change_direction == -1) || (desired_rpm >= kMaxRpm && change_direction == 1)) {
      change_direction *= -1;
    }
    desired_rpm += 500 * change_direction;
    desired_rpm = constrain(desired_rpm, kMinRpm, kMaxRpm);
    prev_dir_switch_time = start_time;
  }

  // for (int i = 0; i < 4; i++) {
  int i = 0;
  int send_pwm = curr_pwm;
  digitalWrite(direction_pins[i], sgn(send_pwm));
  analogWrite(kPwmPins[i], abs(send_pwm));
  
  // Serial.print(curr_error);
  // Serial.print(",");
  // Serial.print(delta_pwm);
  // Serial.print(",");
  // Serial.print(dir_rot);
  // Serial.print(",");
  // Serial.println(send_pwm);
  // Serial.print(",");
  // Serial.println(rpm0);
  // Serial.print(",");
  // Serial.println(global_time - prev_time);
  if (file) {
    file.print(desired_rpm);
    file.print(",");
    file.print(rpm0);
    file.print(",");
    file.print(curr_error);
    file.print(",");
    file.print(send_pwm);
    file.println("");
  } else {
    Serial.println("4card failed write");
    file.close();
    exit(1);
  }

  delay(10);
}
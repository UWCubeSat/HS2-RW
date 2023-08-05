#ifndef RW_SRC_MAIN_HPP_
#define RW_SRC_MAIN_HPP_

#include "Arduino.h"
#include "SD.h"

/* Physical Info */
namespace physical {
// physical constants
static constexpr uint16_t kSerialRate = 9600;
static constexpr uint8_t kNumWheels = 4;

// pins
const uint8_t kPwmPins[kNumWheels] = {3, 8, 9, 11};
const uint8_t kDirectionPins[kNumWheels] = {5, 6, 12, 13};
const uint8_t kFgPins[kNumWheels] = {18, 19, 2, 3};  // rpm reading pins

// signals
uint8_t pwm_signal[kNumWheels] = {0, 0, 0, 0};          // 0-255
uint8_t direction_signal[kNumWheels] = {1, 1, 1, 1};    // 1 clockwise, 0 ccw

// SD reader
static constexpr uint8_t kChipSelect = 53;  // mega specific number
static constexpr uint8_t kSdPin = 10;       // pin on the SD reader
File file;                                  // file opened on the SD card
}  // namespace physical

/* Timing */
namespace timer {
uint8_t init_time;              // time in ms of finish setup
uint32_t loop_start_time;       // time in ms of this loop
uint32_t loop_prev_start_time;  // time in ms of prev loop
uint32_t loop_dt;               // delta between current and prev loop
}  // global time

/* Utility functions*/
namespace util {
// returns 1 if x > 0, 0 if x = 0, -1 if x < 0
static inline bool sign(double x) { return x < 0 ? -1 : (x > 0 ? 1 : 0); }
}

#endif // RW_SRC_MAIN_HPP_

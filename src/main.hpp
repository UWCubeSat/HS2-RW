#ifndef RW_SRC_MAIN_HPP_
#define RW_SRC_MAIN_HPP_

#include "Arduino.h"
#include <Adafruit_BNO08x.h>
#include <imumaths.hpp>
#include "SD.h"

/* Physical Info */
namespace physical
{
    // physical constants
    static constexpr uint16_t kSerialRate = 9600;
    static constexpr uint8_t kNumWheels = 4;

    // pins
    const uint8_t kPwmPins[kNumWheels] = {10, 11, 12, 13};
    const uint8_t kDirectionPins[kNumWheels] = {44, 40, 36, 32};
    const uint8_t kFgPins[kNumWheels] = {18, 19, 2, 3}; // rpm reading pins

    // signals
    uint8_t pwm_signal[kNumWheels] = {0, 0, 0, 0};       // 0-255
    uint8_t direction_signal[kNumWheels] = {1, 1, 1, 1}; // 1 clockwise, 0 ccw

    // imu
    Adafruit_BNO08x bno(-1); // we use I2C for the IMU, so this is unnecessary

    // SD reader
    static constexpr uint8_t kChipSelect = 53; // mega specific number
    static constexpr uint8_t kSdPin = 10;      // pin on the SD reader
    File file;                                 // file opened on the SD card
} // namespace physical

/* Timing */
namespace timer
{
    uint8_t init_time;             // time in ms of finish setup
    uint32_t loop_start_time;      // time in ms of this loop
    uint32_t loop_prev_start_time; // time in ms of prev loop
    uint32_t loop_dt;              // delta between current and prev loop
} // global time

/* Utility functions*/
namespace util
{
    // returns 1 if x > 0, 0 if x = 0, -1 if x < 0
    static inline bool sign(double x) { return (x < 0) ? -1 : ((x > 0) ? 1 : 0); }
}

/* configurable parameters for testing */

namespace test_parameters
{
    /*
    so my understanding is the motors will spin up to the RPM's in 'test_target_speed', and then after 'spin_up_ticks', then they will try to orient themselves in the direction of 'test_target_quaternion'

    I added 'spin_up_seconds', which replaces 'spin_up_ticks' by using the time since the program started instead of cycles, I also commented out the counter variable in main.cpp and added in one for 'spin_up_seconds', so that it works. My idea with this is that it might be easier to know what's going on with testing if we have a time instead of a counter.
    200 ticks probably takes a lot less than a second, so I think the current value of 5 seconds for 'spin_up_seconds' is overkill but idk.

    I also think that this is without the imu's magnetometer, meaning that the the initial rotation quaternion is wherever the system was pointing when it started.

    If I understand this code I think I could try and implement some system that hits a series of quaternions in a row,
    */

    const float torque_PD_params[] = {1.f, 0.f};
    const float wheel_speed_PD_params[] = {1e-3, 0.f};

    float test_target_speed[] = {15000.f, 15000.f, 15000.f, 15000.f};
    imu::Quaternion test_target_quaternion(0.0, 0.0, 0.0, 1.0);

    const int spin_up_ticks = 200;

    const float spin_up_seconds = 5.f;

    /*
    I'm going to include a bunch of variables that I see are used as constants but I don't know if/to what extent they actually are:
    Note: be super careful about messing with these because as of writing this I haven't gotten to mess with the testing setup yet, so what I think are arbitrary constants could be actual hard system limits.
    */

    // ORIGINAL PARAM IN 'lib/RwStatus.hpp':
    // static constexpr float kZeroRpm = 7500;  // midpoint of min and max  // true 15500
    const int rpm_midpoint = 7500;

    

}
#endif // RW_SRC_MAIN_HPP_

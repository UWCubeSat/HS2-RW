#ifndef TESTING_HPP
#define TESTING_HPP

#include <../../include/quaternion.hpp>

namespace test_parameters
{
    /*
    so my understanding is the motors will spin up to the RPM's in 'target_speed', and then after 'spin_up_ticks', then they will try to orient themselves in the direction of 'target_quaternion'

    I added 'spin_up_seconds', which replaces 'spin_up_ticks' by using the time since the program started instead of cycles, I also commented out the counter variable in main.cpp and added in one for 'spin_up_seconds', so that it works. My idea with this is that it might be easier to know what's going on with testing if we have a time instead of a counter.
    200 ticks probably takes a lot less than a second, so I think the current value of 5 seconds for 'spin_up_seconds' is overkill but idk.

    I also think that this is without the imu's magnetometer, meaning that the the initial rotation quaternion is wherever the system was pointing when it started.

    If I understand this code I think I could try and implement some system that hits a series of quaternions in a row,
    */

    unsigned long timeout_sec = 300;
    constexpr float test_delay = 0.f;

    int test_index = 0;
    long test_init_time = 0;

    constexpr float torque_PD_params[2] = {1.f, 0.f};
    constexpr float wheel_speed_PD_params[2] = {1e-3, 0.f};

    struct individual_test
    {
        bool is_indefinite;
        float delay_time;

        bool is_using_quaternion;
        float test_value[4];
    };

    constexpr int number_of_tests = 3;
    individual_test list_of_tests[number_of_tests] = {
        {false, 30.f, false, {100.f, 100.f, 100.f, 100.f}},
        {false, 120.f, true, {1.f, 0.f, 0.f, 0.f}},
        {true, -1.f, true, {1.f, 0.f, 0.f, 1.f}},
    };

} // namespace test_parameters
#endif

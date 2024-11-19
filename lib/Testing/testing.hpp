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

    constexpr float test_delay = 0.f; // time for program to wait after initiating to start testing
    unsigned long timeout_sec = 300;  // time after 'test_delay' the program will run before stopping, so if the reaction wheel explodes it'll eventually stop spinning

    int test_index = 0;      // index of current test
    long test_init_time = 0; // time current test started in millis

    constexpr float torque_PD_params[2] = {1.f, 0.f};       // quaternion torque controller PD params
    constexpr float wheel_speed_PD_params[2] = {1e-3, 0.f}; // wheel speed controller PD params

    // outline of the basic information that makes up a single test object
    struct individual_test
    {
        bool is_indefinite;
        float delay_time;

        bool is_using_quaternion;
        float test_value[4];
    };

    constexpr int number_of_tests = 3; // total number of tests

    // data for each test, remember values are in the order of the 'individual_test' struct
    individual_test list_of_tests[number_of_tests] = {
        {false, 30.f, false, {100.f, 100.f, 100.f, 100.f}},
        {false, 120.f, true, {1.f, 0.f, 0.f, 0.f}},
        {true, -1.f, true, {1.f, 0.f, 0.f, 1.f}},
    };

} // namespace test_parameters
#endif

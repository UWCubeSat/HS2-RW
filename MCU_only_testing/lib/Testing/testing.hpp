#ifndef TESTING_HPP
#define TESTING_HPP

#include <../../include/quaternion.hpp>

namespace test_parameters
{

    //----basic params----//

    constexpr float test_delay = 0.f; // time for program to wait after initiating to start testing
    unsigned long timeout_sec = 300;  // time after 'test_delay' the program will run before stopping and not reseting until hardware reset, so if the reaction wheel explodes it'll eventually stop spinning

    int test_index = 0;      // index of current test
    long test_init_time = 0; // time current test started in millis

    constexpr float torque_PD_params[2] = {1.f, 0.f};       // quaternion torque controller PD params
    constexpr float wheel_speed_PD_params[2] = {1e-3, 0.f}; // wheel speed controller PD params

    //----test params----//
    struct individual_test // outline of the basic information that makes up a single test object
    {
        bool is_indefinite; // whether or not the current test should actually timeout
        float delay_time;   // time the test will take to timeout, if above is false

        bool is_using_quaternion; // whether or not the test is using quaternion orientation control or RPM control
        float test_value[4];      // quaternion or wheel RPM values
    };

    constexpr int number_of_tests = 3; // total number of tests

    // data for each test, remember values are in the order of the 'individual_test' struct
    individual_test list_of_tests[number_of_tests] = {
        //      {is_indefinite, delay_time, is_using_quaternion, test_value},
        {false, 30.f, false, {100.f, 100.f, 100.f, 100.f}},
        {false, 120.f, true, {1.f, 0.f, 0.f, 0.f}},
        {true, -1.f, true, {1.f, 0.f, 0.f, 1.f}},
    };

    //----printing and debugging----//
    constexpr int cycles_per_print = 20; // number of loops to occur for a print statement, this is done because 'Serial' functions are pretty slow so we can reduce the amount of printing

    constexpr bool print_current_quaternion = true;
    constexpr bool print_target_quaternion = true;

    constexpr bool print_current_RPM = true;
    constexpr bool print_target_RPM = true;

    constexpr bool print_current_PWM = true;

    constexpr bool print_debug = true;
} // namespace test_parameters
#endif

#ifndef TESTING_HPP
#define TESTING_HPP

#include <../../include/quaternion.hpp>

namespace test_parameters
{

    //----basic params----//

    constexpr float test_delay_secs = 2.f; // time for program to wait after initiating to start testing
    unsigned long timeout_secs = 300;   // time after 'test_delay_secs' the program will run before stopping and not reseting until hardware reset, so if the reaction wheel explodes it'll eventually stop spinning

    int test_index = 0;      // index of current test
    long test_init_time_ms = 0; // time current test started in millis

    constexpr float torque_PD_params[2] = {1.f, 0.f};       // quaternion torque controller PD params
    constexpr float wheel_speed_PD_params[2] = {1e-3, 0.f}; // wheel speed controller PD params

    //----test params----//
    struct individual_test // outline of the basic information that makes up a single test object
    {
        bool is_indefinite; // whether or not the current test should actually timeout
        float delay_time_secs;   // time the test will take to timeout, if above is false

        bool is_using_quaternion; // whether or not the test is using quaternion orientation control or RPM control
        float test_value[4];      // quaternion or wheel RPM values
    };

    constexpr int number_of_tests = 4; // total number of tests

    // data for each test, remember values are in the order of the 'individual_test' struct
    individual_test list_of_tests[number_of_tests] = {
        //      {is_indefinite, delay_time_secs, is_using_quaternion, test_value},
        {false, 2.5, false, {500.f, 0, 0, 0}},
        {false, 2.5, false, {100.f, 0, 0, 0}},
        {false, 2.5, false, {1000.f, 0, 0, 0}},
        {false, 2.5, false, {100.f, 0, 0, 0}},

    };

    //----printing and debugging----//
    constexpr int cycles_per_print = 100; // number of loops to occur for a print statement, this is done because 'Serial' functions are pretty slow so we can reduce the amount of printing

    constexpr bool print_current_quaternion = true;
    constexpr bool print_target_quaternion = false;

    constexpr bool print_current_RPM = false;
    constexpr bool print_target_RPM = false;

    constexpr bool print_current_PWM = false;

    constexpr bool print_debug = false;
} // namespace test_parameters
#endif

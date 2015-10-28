#include <iostream>
#include <pigpio.h>
#include <chrono>
#include <thread>

#define PWM_NEUTRAL 1500
#define PWM_REAL_RANGE 500
#define PWM_RANGE 100
#define PWM_PIN 17

using namespace std::chrono;
using namespace std;

void arm_quad()
{
    cout << "Arming Quadcopter... ";
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    this_thread::sleep_for(milliseconds(1000));
    gpioServo(PWM_PIN, PWM_NEUTRAL - PWM_REAL_RANGE);
    this_thread::sleep_for(milliseconds(2000));
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    cout << "Done." << endl;
}

void disarm_quad()
{
    cout << "Disarming Quadcopter... ";
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    this_thread::sleep_for(milliseconds(1000));
    gpioServo(PWM_PIN, PWM_NEUTRAL + PWM_REAL_RANGE);
    this_thread::sleep_for(milliseconds(2000));
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    cout << "Done." << endl;
}


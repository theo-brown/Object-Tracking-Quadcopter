#include <iostream>
#include <pigpio.h>

#define PWM_NEUTRAL 1500
#define PWM_REAL_RANGE 500
#define PWM_ADJUST_RANGE 75
#define PWM_PIN 17

void arm_quad()
{
    cout << "Arming Quadcopter... ";
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    sleep(1);
    gpioServo(PWM_PIN, PWM_NEUTRAL - PWM_REAL_RANGE);
    sleep(2);
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    cout << "Done." << endl;
}

void disarm_quad()
{
    cout << "Disarming Quadcopter... ";
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    sleep(1);
    gpioServo(PWM_PIN, PWM_NEUTRAL + PWM_REAL_RANGE);
    sleep(2);
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    cout << "Done." << endl;
}


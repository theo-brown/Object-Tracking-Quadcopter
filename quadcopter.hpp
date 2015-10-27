#include <iostream>
#include <pigpio.h>

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


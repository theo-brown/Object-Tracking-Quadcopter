#include <iostream>
#include <pigpio.h>
#include <chrono>
#include <thread>

#define PWM_NEUTRAL 1500
#define PWM_REAL_RANGE 500
#define PWM_RANGE 150
#define PWM_PIN 17

using namespace std::chrono;
using namespace std;

int gpio_setup()
{
	if (gpioInitialise() < 0)
	{
		cout << "PIGPIO failed to initialise." << endl;
		return 0;
	}
	else
	{
	    cout << "Initialising PIGPIO and neutral throttle... ";
        // Set yaw control pin to neutral
        gpioServo(PWM_PIN, PWM_NEUTRAL);

        // Configure switch inputs
        gpioSetMode(13, PI_INPUT);
        gpioSetPullUpDown(13, PI_PUD_UP);
        gpioSetMode(12, PI_INPUT);
        gpioSetPullUpDown(12, PI_PUD_UP);
        gpioSetMode(6, PI_INPUT);
        gpioSetPullUpDown(6, PI_PUD_UP);
        gpioSetMode(5, PI_INPUT);
        gpioSetPullUpDown(5, PI_PUD_UP);

        // Configure PWM-ADC-Comparator inputs
        gpioSetMode(17, PI_INPUT);
        gpioSetPullUpDown(17, PI_PUD_DOWN);
        gpioSetMode(27, PI_INPUT);
        gpioSetPullUpDown(27, PI_PUD_DOWN);

        this_thread::sleep_for(milliseconds(200));
        cout << "Done." << endl;
        return 0;
	}
}

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


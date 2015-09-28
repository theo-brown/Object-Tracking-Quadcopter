#include <pigpio.h>
#include <iostream>

using namespace std;

#define PWM_PIN 0
#define PWM_NEUTRAL 1500

int main()
{
  // Set up GPIO for mimicking radio output (using gpioServo() function)
  if (gpioInitialise() < 0)
  {
    cout << "pigpio initialisation failed." << endl;
    return -1;
    break;
  }
  gpioSetMode(PWM_PIN, PI_OUTPUT);
  gpioServo(PWM_PIN, PWM_NEUTRAL);
  
  // Do things

  gpioTerminate();
  return 0;
}

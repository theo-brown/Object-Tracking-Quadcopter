#include <iostream>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <thread>

#define I_LIMIT 50

using namespace std;
using namespace std::chrono;

milliseconds sample_rate_ms(202);

struct pid
{
    float input;
    float set_pt;
    float error;
    float error_sum = 0;
    float prev_error;
    float P;
    float I;
    float D;
    float kp = 1.0;
    float ki = 1.0;
    float kd = 1.0;
    int output_adjust;
};


pid pid_calculate(pid pid_a, milliseconds time_elapsed)
{
    // Ensure that sample rate has elapsed
    milliseconds t_diff = sample_rate_ms - time_elapsed;
    if(t_diff.count() > 0)
    {
        this_thread::sleep_for(t_diff);
    }
    else if(t_diff.count() < 0)
    {
        cout << "Time difference is negative; sample rate too low" << endl;
        return pid_a;
    }

    // Calculate P value (error)
    if(pid_a.input == 0)
    {
        pid_a.error == 0;
        cout << "No input to PID." << endl;
    }
    else
    {
        // Calculate error
        pid_a.error = pid_a.set_pt - pid_a.input;

        // Calculate P value
        pid_a.P = pid_a.error * pid_a.kp;
        cout << "P: " << pid_a.P;
        // Catch any P values outside of determined range
        if(pid_a.P > PWM_RANGE)
        {
            pid_a.P = PWM_RANGE;
        }
        if(pid_a.P <  -PWM_RANGE)
        {
            pid_a.P = -PWM_RANGE;
        }
    }

    // Add error to total error sum
    pid_a.error_sum += pid_a.error * static_cast<float>(sample_rate_ms.count());
    // Calculate I value
    pid_a.I = pid_a.error_sum * pid_a.ki;
    // Catch any error_sum values resulting in I outside of determined range
    if(pid_a.I > I_LIMIT)
    {
        pid_a.error_sum = I_LIMIT/pid_a.ki;
    }
    else if(pid_a.error_sum < -I_LIMIT and pid_a.ki != 0)
    {
        pid_a.error_sum = -I_LIMIT/pid_a.ki;
    }
    cout << " I: " << pid_a.I;

    // Calculate D value (rate of change of error)
    pid_a.D = pid_a.kd * (pid_a.P - pid_a.prev_error) / (static_cast<float>(sample_rate_ms.count()));
    cout << " D: " << pid_a.D << endl;
    // Set previous error
    pid_a.prev_error = pid_a.error;

    // Set output
    pid_a.output_adjust = static_cast<int>(pid_a.P + pid_a.I + pid_a.D);
    if(pid_a.output_adjust < 2 and pid_a.output_adjust > -2)
    {
        pid_a.output_adjust = 0;
    }

    cout << "PID adjustment: " << pid_a.output_adjust << endl;

    return pid_a;
}

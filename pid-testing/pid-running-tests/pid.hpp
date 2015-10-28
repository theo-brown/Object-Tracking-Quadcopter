#include <iostream>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <thread>

using namespace std;
using namespace std::chrono;

milliseconds sample_rate_ms(75);

struct pid_float
{
	float P;
	float I;
	float D;
	float kp = 1;
	float ki = 1;
	float kd = 1;
	float input;
	float set_pt;
	float prev_error = 0;
	float error_sum = 0;
};

pid_float pid_calculate(pid_float pid_a, milliseconds time_elapsed)
{
	// Ensure that sample rate has elapsed
	milliseconds t_diff = sample_rate_ms - time_elapsed;
	if(t_diff.count() > 0)
	{
		this_thread::sleep_for(t_diff);
	}
	else if(t_diff.count() < 0)
	{
	    cout << "Time difference is negative; sample rate too high" << endl;
	    return pid_a;
	}

	// Calculate P value (error)
	pid_a.P = pid_a.set_pt - pid_a.input;
	cout << "P: " << pid_a.P;

	// Add error to total error sum
	pid_a.error_sum += pid_a.P;

	// Calculate I,D values
	pid_a.I = pid_a.error_sum * sample_rate_ms.count();
	cout << " I: " << pid_a.I;

	pid_a.D = (pid_a.P - pid_a.prev_error) * sample_rate_ms.count();
	cout << " D: " << pid_a.D << endl;
	
	// Set previous error
	pid_a.prev_error = pid_a.P;

    return pid_a;
}


#include <iostream>
#include <ctime>
#include <unistd.h>
#include <chrono>
#include <thread>

using namespace std;
using namespace std::chrono;

milliseconds sample_rate_ms(300);

struct pid
{
	float P = 0;
	float I = 0;
	float D = 0;
	float kp = 1;
	float ki = 1;
	float kd = 1;
	float input;
	float set_pt;
	float prev_error = 0;
	float prev_input = 0;
	float output;
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
	pid_a.P = pid_a.set_pt - pid_a.input;
	cout << "P: " << pid_a.P;

	// Add error to total error sum
	pid_a.I += pid_a.P * sample_rate_ms.count();
	cout << " I: " << pid_a.I;
	
	// Calculate rate of change of error
	//pid_a.D = (pid_a.P - pid_a.prev_error) / sample_rate_ms.count();
	// Alternative method - rate of change of input
	pid_a.D = -(pid_a.input - pid_a.prev_input) / sample_rate_ms.count();
	cout << " D: " << pid_a.D << endl;
	
	// Set previous error
	pid_a.prev_error = pid_a.P;
	pid_a.prev_input = pid_a.input;
	
	pid_a.output = (pid_a.P*pid_a.kp) + (pid_a.I*pid_a.ki) + (pid_a.D*pid_a.kd);
	
	cout << "PID: " << pid_a.output << endl;

    return pid_a;
}


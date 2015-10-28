#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include "pid.hpp"

using namespace std;
using namespace std::chrono;

int main()
{
    /****************/
    /** SET UP PID **/
    /****************/
    pid_float pid_yaw;
    pid_yaw.set_pt = 0; // Set setpoint as the image centre

    int coord = 0;

    while (1)
    {
        // Get start time
        time_point<high_resolution_clock> start_t = high_resolution_clock::now();
    
        this_thread::sleep_for(milliseconds(25));

        /*********/
        /** PID **/
        /*********/
        pid_yaw.input = coord;
        coord += 10;

        // Get end time
        time_point<high_resolution_clock> end_t = high_resolution_clock::now();
        
        milliseconds elapsed_t = duration_cast<milliseconds>(end_t - start_t);

        // Calculate pid values
        pid_calculate(pid_yaw, elapsed_t);

        /*****************/
        /** YAW CONTROL **/
        /*****************/
        //int yaw_val = 1500 - (pid_yaw.P*pid_yaw.kp) - (pid_yaw.I*pid_yaw.ki) - (pid_yaw.D*pid_yaw.kd);
        //cout << "Yaw: " << yaw_val << endl;
    }

    return 0;
}

#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>
#include "opencv.hpp"
#include "camera.hpp"
#include "quad.hpp"
#include "pid.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono;

#define PI 3.14

int main()
{
    /****************/
    /** SET UP PID **/
    /****************/
    pid pid_yaw;
    pid_yaw.set_pt = IMG_WIDTH/2; // Set setpoint as the image centre
    pid_yaw.kp = 0.702;
    pid_yaw.ki = 0.00006;
    pid_yaw.kd = 4.9;
    int yaw_output = PWM_NEUTRAL;

    /*******************************/
    /** INITIALISE CAPTURE DEVICE **/
    /*******************************/
    frame frame1;
    camera_init();

    /********************/
    /** CREATE WINDOWS **/
    /********************/
    cout << "Creating windows... ";
    namedWindow("OCTO-pi", WINDOW_NORMAL);

    int threshHue=0, threshSat=0, threshVal=0;
    createTrackbar("Threshold Hue", "OCTO-pi", &threshHue, 180);
    createTrackbar("Threshold Sat", "OCTO-pi", &threshSat, 255);
    createTrackbar("Threshold Val", "OCTO-pi", &threshVal, 255);
    cout << "Done." << endl;

    /***********************/
    /** INITIALISE PIGPIO **/
    /***********************/
    cout << "Initialising PIGPIO and neutral throttle... ";
    gpioInitialise();
    // Set yaw control pin to neutral
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    // Configure switch inputs
    gpioSetMode(13, PI_INPUT);
    gpioSetMode(12, PI_INPUT);
    gpioSetMode(6, PI_INPUT);
    gpioSetMode(5, PI_INPUT);
    // Set pull up resistors
    gpioSetPullUpDown(13, PI_PUD_UP);
    gpioSetPullUpDown(12, PI_PUD_UP);
    gpioSetPullUpDown(6, PI_PUD_UP);
    gpioSetPullUpDown(5, PI_PUD_UP);
    this_thread::sleep_for(milliseconds(2000));
    cout << "Done." << endl;

    bool rept_training = true;
    bool rept_tracking = false;
    bool quad_armed = false;

    while (rept_training or rept_tracking)
    {
        /*********************/
        /** COLOUR TRAINING **/
        /*********************/
        while (rept_training)
        {
            cout << "Taking preliminary image for colour recognition... (q to continue, r to restart, a to arm, d to disarm) ";
            frame1 = frame_capture(frame1);
            cout << "Done." << endl;

			while (1)
			{
				frame1 = detect_obj(frame1, threshHue, threshSat, threshVal);

				// Draw circle on mean point
				Mat drawn_circle = frame1.captured.clone();
				circle(drawn_circle, frame1.object.pt, sqrt(frame1.object.size/PI), Scalar(0,255,0));
				imshow("OCTO-pi", drawn_circle);
				waitKey(15); // Needed to display image

				// Read switches
				this_thread::sleep_for(milliseconds(15));
				if (!gpioRead(13))
				{
					cout << "Proceeding to object tracking. (q to exit, a to arm, d to disarm) " << endl;
					rept_training = false;
					rept_tracking = true;
					break;
				}
				if (!gpioRead(12))
				{
					cout << "Restarting... " << endl;
					rept_training = true;
					break;
				}
				if (!gpioRead(6))
				{
					if (not quad_armed) arm_quad();
					else disarm_quad;
				}
			}
		}

		/*********************/
		/** OBJECT TRACKING **/
		/*********************/
		// Get start time
		time_point<high_resolution_clock> start_t = high_resolution_clock::now();
		time_point<high_resolution_clock> end_t;
		milliseconds elapsed_t;
		while (rept_tracking)
		{
			// Capture frame
			frame1 = frame_capture(frame1);
			// Detect objects in frame
			frame1 = detect_obj(frame1, threshHue, threshSat, threshVal);

			this_thread::sleep_for(milliseconds(15));
                        if (!gpioRead(13))
		        {
                            cout << "User exit." << endl;
                            rept_tracking = false;
                            rept_training = false;
                            gpioServo(PWM_PIN, PWM_NEUTRAL);
                            break;
			}
			if (!gpioRead(12))
			{
			    cout << "Returning to colour training... " << endl;
			    rept_training = true;
			    rept_tracking = false;
			    break;
			}
			if (!gpioRead(6))
			{
			    if (not quad_armed) arm_quad();
			    else disarm_quad();
			}
			if (!gpioRead(5))
			{
                            pid_yaw.error_sum = 0; // Reset summing so integral does not be massive
			}

			/*********/
			/** PID **/
			/*********/
			// Input x coordinate as process variable to PID
			pid_yaw.input = frame1.object.pt.x;

			// Get end time
			end_t = high_resolution_clock::now();
			elapsed_t = duration_cast<milliseconds>(end_t - start_t);
			cout << "LOOP TIME: " << elapsed_t.count() << endl;
			// Restart timer
			start_t = high_resolution_clock::now();
			// Calculate pid values
			pid_yaw = pid_calculate(pid_yaw, elapsed_t);

			/*****************/
			/** YAW CONTROL **/
			/*****************/
			yaw_output = PWM_NEUTRAL - pid_yaw.output_adjust;
			cout << "Yaw: " << yaw_output << endl;
			gpioServo(PWM_PIN, yaw_output); // Send PWM pulses
		}
	}

    // Clean up
    cout << "Releasing camera... " << endl;
    pi_camera.release();
    cout << "Releasing windows... " << endl;
    destroyWindow("OCTO-pi");
    cout << "Stopping PIGPIO..." << endl;
    gpioTerminate();
    cout << "Done." << endl;
    cout << "Current kP value: " << pid_yaw.kp << endl;
    cout << "Current kI value: " << pid_yaw.ki << endl;
    cout << "Current kD value: " << pid_yaw.kd << endl;

    return 0;
}

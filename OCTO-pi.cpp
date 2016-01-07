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
            cout << "Taking preliminary image for colour recognition... (RSW 2 to continue, TFT 12 to restart, TFT 6 to arm/disarm) ";
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
				if (gpioRead(17) && !gpioRead(27)) // 17 and not 27 = Radio switch position 2
				{
					cout << "RSW 2; Proceeding to object tracking. (RSW 3 to exit, RSW 2 to return to training, TFT 6 to arm/disarm) " << endl;
					rept_training = false;
					rept_tracking = true;
                                        this_thread::sleep_for(milliseconds(150));
					break;
				}
				if (!gpioRead(12)) // 12 = TFT switch
				{
					cout << "TFT 12 pressed; Restarting... " << endl;
					rept_training = true;
                                        this_thread::sleep_for(milliseconds(15));
					break;
				}
				if (!gpioRead(6)) // 6 = TFT switch
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

                        if (gpioRead(17) && gpioRead(27)) // 17 and 27 = Radio switch position 3
		        {
                            cout << "RSW 3; User exit." << endl;
                            rept_tracking = false;
                            rept_training = false;
                            gpioServo(PWM_PIN, PWM_NEUTRAL);
                            this_thread::sleep_for(milliseconds(50));
                            break;
			}
			if (!gpioRead(17) && !gpioRead(27)) // !17 and !27 = Radio switch position 1
			{
			    cout << "RSW 1; Returning to colour training... " << endl;
			    rept_training = true;
			    rept_tracking = false;
                            this_thread::sleep_for(milliseconds(15));
			    break;
			}
			if (!gpioRead(6))
			{
			    if (not quad_armed) arm_quad();
			    else disarm_quad();
			}
			if (!gpioRead(5))
			{
                            this_thread::sleep_for(milliseconds(15));
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

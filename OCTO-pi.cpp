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
				// Get keypress from user
            			char c = waitKey(100);
    			        if (c == 113) // q pressed
            			{
					cout << "Proceeding to object tracking. (q to exit, a to arm, d to disarm) " << endl;
					rept_training = false;
					rept_tracking = true;
					break;
		                }
                                else if (c == 114) // r pressed
		                {
			                cout << "Restarting... " << endl;
					rept_training = true;
					break;
		                }
           			else if (c == 97) // a pressed
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
       			char c = waitKey(5);
		        if (c == 113) // q pressed
      			{
				cout << "Exiting... " << endl;
				rept_training = false;
				rept_tracking = false;
				break;
		        }
                        else if (c == 114) // r pressed
	                {
		                cout << "Returning to colour training... " << endl;
				rept_training = true;
				rept_tracking = false;
				break;
		        }
           		else if (c == 97) // a pressed
			{
			       if (not quad_armed) arm_quad();
				else disarm_quad;
			}
			else if (c == 112) // p pressed
			{
				pid_yaw.kp += 0.001;
			}
			else if (c == 111) // o pressed
			{
				pid_yaw.kp -= 0.001;
			}
			else if (c == 105) // i pressed
			{
				pid_yaw.ki += 0.00001;
			}
			else if (c == 117) // u pressed
			{
				pid_yaw.ki -= 0.00001;
			}
			else if (c == 121) // y pressed
			{
				pid_yaw.kd += 0.1;
			}
			else if (c == 116) // t pressed
			{
				pid_yaw.kd -= 0.1;
			}
			else if (c == 114) // r pressed
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

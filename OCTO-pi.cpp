#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>
#include "opencv.hpp"
#include "camera.hpp"
#include "gpio.hpp"
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
    pid pid_yaw; // Create PID structure
    pid_yaw.set_pt = IMG_WIDTH/2; // Set setpoint as the image centre
    pid_yaw.kp = 0.702; // P tuning parameter
    pid_yaw.ki = 0.00006; // I tuning parameter
    pid_yaw.kd = 4.9; // D tuning parameter
    int yaw_output = PWM_NEUTRAL; // Set the output to 0

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
    gpio_setup();

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
                imshow("OCTO-pi", drawn_circle); // Display image
                char c = waitKey(15); // Get keypress (needed to display image)

                // Read switches
                //if (c == 113 or tft_switch == 13 or (gpioRead(17) and !gpioRead(27))) // 113 = q pressed, TFT 13, or 17 and !27 = RSW 2
                if (c == 113 or tft_switch == 13) // 113 = q pressed, TFT 13
                {
                    tft_switch = 0;
                    cout << "Proceeding to object tracking. (13 to exit, 12 to return to training, 6 to arm/disarm) " << endl;
                    rept_training = false;
                    rept_tracking = true;
                    this_thread::sleep_for(milliseconds(250));
                    break;
                }
		if (c == 114 or tft_switch == 12) // 114 = r pressed, TFT 12
		{
                    tft_switch = 0;
		    cout << "Restarting... " << endl;
		    rept_training = true;
                    this_thread::sleep_for(milliseconds(20));
		    break;
		}
		if (c == 97 or tft_switch == 6) // 97 = a pressed, TFT 6
		{
                    tft_switch = 0;
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

            // Read keypress from user
            char c = waitKey(15);

            if (c == 113 or tft_switch == 13 or (gpioRead(17) and !gpioRead(27))) // 113 = q pressed, TFT 13 or 17 and 27 = RSW 2
            {
                cout << "User exit." << endl;
                rept_tracking = false;
                rept_training = false;
                gpioServo(PWM_PIN, PWM_NEUTRAL);
                this_thread::sleep_for(milliseconds(50));
                break;
            }
            //if (c == 114 or tft_switch == 13 or (!gpioRead(17) and !gpioRead(27)) ) // 114 = r pressed, TFT 13 or !17 and !27 = RSW 1
            if (c == 114 or tft_switch == 13) // 114 = r pressed or TFT 13 (testing)
            {
                tft_switch = 0;
                cout << "Returning to colour training... " << endl;
                rept_training = true;
                rept_tracking = false;
                this_thread::sleep_for(milliseconds(15));
                break;
            }
            if (c == 97 or tft_switch == 6) // 97 = a pressed or TFT 6
	    {
                tft_switch = 0;
	        if (not quad_armed) arm_quad();
	        else disarm_quad();
	    }
	    if (c == 105 or tft_switch == 5) // 105 = i pressed or TFT 5
	    {
                tft_switch = 0;
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

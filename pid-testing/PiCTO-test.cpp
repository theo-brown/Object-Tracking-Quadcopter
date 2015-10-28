#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>
#include "opencv.hpp"
#include "pi_camera.hpp"
#include "pid.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono;

int main()
{
    /****************/
    /** SET UP PID **/
    /****************/
    pid pid_yaw;
    pid_yaw.set_pt = IMG_WIDTH/2; // Set setpoint as the image centre
    pid_yaw.kp = 5;
    pid_yaw.ki = 0.0001;
    pid_yaw.kd = 1;

    /*******************************/
    /** INITIALISE CAPTURE DEVICE **/
    /*******************************/
    frame frame1;
    camera_init();

    /********************/
    /** CREATE WINDOWS **/
    /********************/
    cout << "Creating windows... ";
    // Image display window
    namedWindow("Preview", WINDOW_NORMAL);
    // HSV adjust window
    namedWindow("Threshold", WINDOW_NORMAL);

    int threshHue=0, threshSat=0, threshVal=0;
    createTrackbar("Threshold Hue", "Threshold", &threshHue, 180);
    createTrackbar("Threshold Sat", "Threshold", &threshSat, 255);
    createTrackbar("Threshold Val", "Threshold", &threshVal, 255);
    cout << "Done." << endl;

    /*********************/
    /** COLOUR TRAINING **/
    /*********************/
    int rept = 1;
    while (rept == 1)
    {
        cout << "Taking preliminary image for colour recognition... (q to continue, r to restart) ";

        frame1 = frame_capture(frame1);

        cout << "Done." << endl;

        while (1)
        {
            frame1 = detect_obj(frame1, threshHue, threshSat, threshVal);

            // Draw circle on mean point
            Mat drawn_circle = frame1.captured.clone();
            circle(drawn_circle, frame1.object.pt, sqrt(frame1.object.size/3.14), Scalar(0,0,0));

            imshow("Threshold", frame1.thresholded);
            imshow("Preview", drawn_circle);

            // Get keypress from user
            char c = waitKey(100);
            if (c == 113) // q pressed
            {
                cout << "Proceeding to object tracking. (q to exit)" << endl;
                destroyWindow("Preview");
                rept = 0;
                break;
            }
            else if (c == 114) // r pressed
            {
                cout << "Restarting... " << endl;
                rept = 1;
                break;
            }
		}
	}

    /*********************/
    /** OBJECT TRACKING **/
    /*********************/
    int yaw_val = 1500;
    while (1)
    {
        // Get start time
        time_point<high_resolution_clock> start_t = high_resolution_clock::now();

        // Capture frame
        frame1 = frame_capture(frame1);
        // Detect objects in frame
        frame1 = detect_obj(frame1, threshHue, threshSat, threshVal);

        // Draw circle on object point
        circle(frame1.thresholded, frame1.object.pt, sqrt(frame1.object.size/3.14), Scalar(255,0,0));
        imshow("Threshold", frame1.thresholded);

        // Get keypress from the user
        char c = waitKey(20);

        if(c == 113) // q pressed
        {
            cout << "User exit." << endl;
            break;
        }
        
        /*********/
        /** PID **/
        /*********/
        pid_yaw.input = frame1.object.pt.x;

        // Get end time
        time_point<high_resolution_clock> end_t = high_resolution_clock::now();
        
        milliseconds elapsed_t = duration_cast<milliseconds>(end_t - start_t);

        // Calculate pid values
        pid_calculate(pid_yaw, elapsed_t);

        /*****************/
        /** YAW CONTROL **/
        /*****************/
        yaw_val += pid_yaw.output;
        cout << "Yaw: " << yaw_val << endl;
    }

    // Clean up
    cout << "Releasing camera... " << endl;
    pi_camera.release();
    cout << "Releasing windows... " << endl;
    destroyWindow("Threshold");
    cout << "Done." << endl;

    return 0;
}

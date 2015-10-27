#include <iostream>
#include <math.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "raspicam/raspicam_cv.h"
#include <pigpio.h>
#include "opencv.hpp"
#include "quadcopter.hpp"
#include "pi_camera.hpp"

#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define PWM_NEUTRAL 1500
#define PWM_REAL_RANGE 500
#define PWM_ADJUST_RANGE 75
#define PWM_PIN 17
#define PI  3.14

using namespace cv;
using namespace std;

int main()
{
    frame frame1;

    /*******************************/
    /** INITIALISE CAPTURE DEVICE **/
    /*******************************/
    camera_init();

    /********************/
    /** CREATE WINDOWS **/
    /********************/
    cout << "Creating windows... ";
    // Image display window
    namedWindow("Preview", WINDOW_NORMAL);
    // HSV adjust window
    namedWindow("Threshold", WINDOW_NORMAL);
    //namedWindow("Contours", WINDOW_NORMAL);

    int threshHue=0, threshSat=0, threshVal=0;
    createTrackbar("Threshold Hue", "Threshold", &threshHue, 180);
    createTrackbar("Threshold Sat", "Threshold", &threshSat, 255);
    createTrackbar("Threshold Val", "Threshold", &threshVal, 255);
    cout << "Done." << endl;

   /***********************/
    /** INITIALISE PIGPIO **/
    /***********************/
    cout << "Initialising PIGPIO and neutral throttle... ";
    gpioInitialise();
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    sleep(2);
    cout << "Done." << endl;


    /*********************/
    /** COLOUR TRAINING **/
    /*********************/
    int rept = 1;
    while (rept == 1)
    {
        cout << "Taking preliminary image for colour recognition... (q to continue, r to restart, a to arm, d to disarm) ";

        frame1 = frame_capture(frame1);

        cout << "Done." << endl;

        while (1)
        {
            frame1 = detect_obj(frame1, threshHue, threshSat, threshVal);

            // Draw circle on mean point
            Mat drawn_circle = frame1.captured.clone();
            circle(drawn_circle, frame1.object.pt, sqrt(frame1.object.size/PI), Scalar(0,0,0));

            imshow("Threshold", frame1.thresholded);
            imshow("Preview", drawn_circle);

            // Get keypress from user
            char c = waitKey(100);
            if (c == 113) // q pressed
            {
                cout << "Proceeding to object tracking. (q to exit, a to arm, d to disarm) " << endl;
                rept = 0;
                break;
            }
            else if (c == 114) // r pressed
            {
                cout << "Restarting... " << endl;
                rept = 1;
                break;
            }
            else if (c == 97) // a pressed
            {
            	arm_quad();
            }
            else if (c == 100) // d pressed
            {
            	disarm_quad();
            }
        }
    }

    destroyWindow("Preview");

    /*********************/
    /** OBJECT TRACKING **/
    /*********************/
    while (1)
    {
        frame1 = frame_capture(frame1);
        frame1 = detect_obj(frame1, threshHue, threshSat, threshVal);

        // Return keypoint distance from centre
        Point2f centre = Point(IMG_WIDTH/2, IMG_HEIGHT/2);
        Point2f pt_err = centre - frame1.object.pt;
        // If the object is off the screen, mean_point.pt == 0 so pt_err == centre.
        // This line eliminates the false result to keep the quad under control.
        if(pt_err == centre) {pt_err = Point(0,0);}

        cout << "Diff X: " << pt_err.x << " Y: " << pt_err.y << endl;

        // Draw circle on mean point
        circle(frame1.thresholded, frame1.object.pt, sqrt(frame1.object.size/PI), Scalar(255,0,0));
        imshow("Threshold", frame1.thresholded);

        /*****************/
        /** YAW CONTROL **/
        /*****************/
        int yaw_val = PWM_NEUTRAL - (pt_err.x * PWM_ADJUST_RANGE / (IMG_WIDTH/2)); // Scales the yaw value
        cout << "Yaw: " << yaw_val << endl;
        gpioServo(PWM_PIN, yaw_val); // Send PWM pulses

        // Get keypress from the user
        char c = waitKey(20);
        if(c == 113) // q pressed
        {
            cout << "User exit." << endl;
            disarm_quad();
            break;
        }
        else if(c == 97) // a pressed
        {
            arm_quad();
        }
        else if (c == 100) // d pressed
        {
            disarm_quad();
        }
    }

    // Clean up
    cout << "Releasing camera... " << endl;
    pi_camera.release();
    cout << "Releasing windows... " << endl;
    destroyWindow("Threshold");
    cout << "Stopping PIGPIO..." << endl;
    gpioTerminate();
    cout << "Done." << endl;

    return 0;
}

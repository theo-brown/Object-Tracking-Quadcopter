#include <iostream>
#include <math.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "raspicam/raspicam_cv.h"
#include <pigpio.h>

#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define PWM_NEUTRAL 1500
#define PWM_REAL_RANGE 500
#define PWM_ADJUST_RANGE 75
#define PWM_PIN 17
#define PI  3.14

using namespace cv;
using namespace std;

raspicam::RaspiCam_Cv pi_camera;

struct obj_point
{
    Point2f pt;
    int size;
};

struct frame
{
    Mat captured;
    Mat hsv;
    Mat thresholded;
    Mat processed;
    Mat drawn_circle;
    vector<vector<Point> > contrs;
    obj_point object;
};


void arm_quad()
{
    cout << "Arming Quadcopter... ";
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    sleep(2);
    gpioServo(PWM_PIN, PWM_NEUTRAL - PWM_REAL_RANGE);
    sleep(2);
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    cout << "Done." << endl;
}

void disarm_quad()
{
    cout << "Disarming Quadcopter... ";
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    sleep(2);
    gpioServo(PWM_PIN, PWM_NEUTRAL + PWM_REAL_RANGE);
    sleep(2);
    gpioServo(PWM_PIN, PWM_NEUTRAL);
    cout << "Done." << endl;
}


int camera_init()
{
    // Starts up the Pi camera
    cout << "Initialising Pi camera... ";

    pi_camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);

    pi_camera.open();
    sleep(1);

    if (!pi_camera.isOpened())
    {
        cout << "Failed to access webcam" << endl;
        return -1;
    }
    sleep(2);
    cout << "Done." << endl;

    return 0;
}

frame frame_capture(frame img)
{
    // Captures a frame
    pi_camera.grab();
    pi_camera.retrieve(img.captured);

    if (img.captured.empty())
    {
        cout << "Error retrieving frame" << endl;
        return img;
    }

    resize(img.captured, img.captured, Size(IMG_WIDTH,IMG_HEIGHT), 0, 0, CV_INTER_AREA);
    flip(img.captured, img.captured, 0);

    return img;
}

Mat morph_mat(Mat src, Mat dst)
{
	Mat erode_rect = getStructuringElement(MORPH_RECT,Size(2,2));
	Mat dilate_rect = getStructuringElement(MORPH_RECT,Size(5,5));
	erode(src, dst, erode_rect, Point(-1,-1), 2);
	dilate(dst, dst, dilate_rect, Point(-1,-1), 2);
	return dst;
}

frame detect_obj (frame img, int hue, int sat, int val)
{
    // Convert to HSV
    img.hsv.create(img.captured.size(), img.captured.type());
    cvtColor(img.captured, img.hsv, CV_BGR2HSV);

    // Threshold the frame so only specified HSV displayed
    inRange(img.hsv, Scalar(hue-7, sat, val), Scalar(hue+7, 255, 255), img.thresholded);
    // Erode and dilate image
    morph_mat(img.thresholded, img.thresholded);
    // Copy it for modifying
    img.thresholded.copyTo(img.processed);

    // Contour the image
    findContours(img.processed, img.contrs, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    int contour_no=0, largest_contour_no, largest_contour_area=0;
    // Iterate through the contours
    for(contour_no; contour_no < img.contrs.size(); contour_no++)
    {
		// Find the largest contour
		if(contourArea(img.contrs[contour_no]) > largest_contour_area)
		{
			largest_contour_area = contourArea(img.contrs[contour_no]);
			largest_contour_no = contour_no;
		}
	}

    // If no contours found then mean point is 0
    if(largest_contour_area == 0)
    {
	    img.object.pt = Point2f(0,0);
	    img.object.size = 0;
    }
    else
    {
		// Find moments of largest contour
		Moments obj_momts = moments(img.contrs[largest_contour_no], false);
		// Calculate moment centre
        img.object.pt = Point2f(obj_momts.m10 / obj_momts.m00, obj_momts.m01 / obj_momts.m00);
        img.object.size = largest_contour_area;
    }
    return img;
}

/***************************************/

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
            frame1.drawn_circle = frame1.captured.clone();
            circle(frame1.drawn_circle, frame1.object.pt, sqrt(frame1.object.size/PI), Scalar(0,0,0));

            //imshow("Contours", frame1.processed);
            imshow("Threshold", frame1.thresholded);
            imshow("Preview",frame1.drawn_circle);

            // Get keypress from user
            char c = waitKey(100);
            if (c == 113) // q pressed
            {
                cout << "Proceeding to object tracking. (q to exit, a to arm, d to disarm) " << endl;
                destroyWindow("Threshold");

                waitKey(20);
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
    arm_quad();


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
        circle(frame1.captured, frame1.object.pt, sqrt(frame1.object.size/PI), Scalar(0,0,0));

        /*****************/
        /** YAW CONTROL **/
        /*****************/
        int yaw_val = PWM_NEUTRAL - (pt_err.x * PWM_ADJUST_RANGE / (IMG_WIDTH/2)); // Scales the yaw value
        cout << "Yaw: " << yaw_val << endl;
        gpioServo(PWM_PIN, yaw_val); // Send PWM pulses


        // Display frame
        //imshow("Contours", frame1.processed);
        //imshow("Preview", frame1.captured);

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
    //destroyWindow("Contours");
    //destroyWindow("Preview");
    //destroyWindow("Threshold");
    cout << "Stopping PIGPIO..." << endl;
    gpioTerminate();
    cout << "Done." << endl;

    return 0;
}

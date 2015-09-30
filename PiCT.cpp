#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "raspicam/raspicam_cv.h"
#include <pigpio.h>

#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define PWM_NEUTRAL 1500
#define PWM_RANGE 500
#define PWM_PIN 17

using namespace cv;
using namespace std;

raspicam::RaspiCam_Cv pi_camera;

struct frame
{
    Mat captured;
    Mat hsv;
    Mat thresholded;
    Mat processed;
    vector<KeyPoint> keypoints;
};

struct obj_point
{
    Point2i pt;
    int size;
};

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
    sleep(3);
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

frame detect_obj (frame img, Ptr<SimpleBlobDetector> detector, int hue, int sat, int val)
{
    // Convert to HSV
    img.hsv.create(img.captured.size(), img.captured.type());
    cvtColor(img.captured, img.hsv, CV_BGR2HSV);

    // Threshold the frame so only specified HSV displayed
    inRange(img.hsv, Scalar(hue-7, sat, val), Scalar(hue+7, 255, 255), img.thresholded);

    // Detect blobs
    detector->detect(img.thresholded, img.keypoints);

    return img;
}

obj_point find_mean_point (frame img, obj_point mean_point)
{
    // Find mean (x,y) and size of first 5 keypoints:
    float tot_x=0, tot_y=0, tot_size=0, no_kpts=0;
    for(no_kpts; no_kpts < img.keypoints.size() and no_kpts<=5; no_kpts++)
    {
        tot_x += img.keypoints[no_kpts].pt.x;
        tot_y += img.keypoints[no_kpts].pt.y;
        tot_size += img.keypoints[no_kpts].size;
    }
    mean_point.pt = Point(tot_x / no_kpts, tot_y / no_kpts);
    mean_point.size = tot_size / no_kpts;

    return mean_point;
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
    namedWindow("Preview", CV_WINDOW_AUTOSIZE);
    // HSV adjust window
    namedWindow("Threshold", CV_WINDOW_AUTOSIZE);
    int threshHue=0, threshSat=0, threshVal=0;
    createTrackbar("Threshold Hue", "Threshold", &threshHue, 180);
    createTrackbar("Threshold Sat", "Threshold", &threshSat, 255);
    createTrackbar("Threshold Val", "Threshold", &threshVal, 255);

    /***************************/
    /** SET UP BLOB DETECTION **/
    /***************************/
    // Initialise parameters
    SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 40;
    params.filterByArea = true;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.filterByCircularity = false;
    // Create the blob detector
    Ptr<SimpleBlobDetector> blobdetect = SimpleBlobDetector::create(params);
    
    /***********************/
    /** INITIALISE PIGPIO **/
    /***********************/
    gpioInitialise();


    /*********************/
    /** COLOUR TRAINING **/
    /*********************/
    int rept = 1;
    while (rept == 1)
    {
        cout << "Done." << endl << "Taking preliminary image for colour recognition... ";

        frame1 = frame_capture(frame1);

        cout << "Done." << endl;

        while (1)
        {
            frame1 = detect_obj(frame1, blobdetect, threshHue, threshSat, threshVal);

            drawKeypoints(frame1.captured, frame1.keypoints, frame1.processed, Scalar(0,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            imshow("Threshold", frame1.thresholded);
            imshow("Preview",frame1.processed);

            char c = waitKey(100);
            if (c == 113) // q pressed
            {
                cout << "Proceeding to object tracking. (q to exit)" << endl;
                destroyWindow("Threshold");

                waitKey(20);
                rept = 0;
                break;
            }
            else if (c == 114) // r pressed
            {
                cout << "Restarting..." << endl;
                rept = 1;
                break;
            }
        }
    }


    /*********************/
    /** OBJECT TRACKING **/
    /*********************/
    obj_point mean_pt;
    while (1)
    {
        frame1 = frame_capture(frame1);
        frame1 = detect_obj(frame1, blobdetect, threshHue, threshSat, threshVal);
        mean_pt = find_mean_point(frame1, mean_pt);

        // Return keypoint distance from centre
        Point2i centre = Point(IMG_WIDTH/2, IMG_HEIGHT/2);
        Point2i pt_err = centre - mean_pt.pt;
        // If the object is off the screen, mean_pt.pt == 0 so pt_err == centre.
        // This line eliminates the false result to keep the quad under control.
        if(pt_err == centre) {pt_err = Point(0,0);} 

        cout << "Diff X: " << pt_err.x << " Y: " << pt_err.y << endl;

        // Draw circle on mean point
        circle(frame1.captured, mean_pt.pt, mean_pt.size, Scalar(0,0,0));
        
        /*****************/
        /** YAW CONTROL **/
        /*****************/
        int yaw_val = PWM_NEUTRAL + (pt_err.x * PWM_RANGE / (IMG_WIDTH/2)); // Scales the yaw value to 1000-2000, 1500 as neutral
        cout << "Yaw: " << yaw_val << endl;
        gpioServo(PWM_PIN, yaw_val); // Send PWM pulses
        

        // Display frame
        imshow("Preview", frame1.captured);

        // Exit if q pressed
        char c = waitKey(20);
        if( c == 113 )
        {
            cout << "User exit." << endl;
            break;
        }
    }

    // Clean up
    cout << "Releasing camera... " << endl;
    pi_camera.release();
    cout << "Releasing windows... " << endl;
    destroyWindow("Preview");
    destroyWindow("Threshold");
    cout << "Stopping PIGPIO..." << endl;
    gpioTerminate();
    cout << "Done." << endl;

    return 0;
}

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "raspicam/raspicam_cv.h"

using namespace cv;
using namespace std;

int main()
{
    // Images
    Mat frame, frame_hsv, frame_thresh, frame_processed;
    vector<KeyPoint> keypoints;

    // INITIALISE CAPTURE DEVICE
    cout << "Initialising PiCamera... ";
    raspicam::RaspiCam_Cv capture;
    capture.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    capture.open();
    sleep(1);
    if (!capture.isOpened())
    {
        cout << "Failed to access webcam. \n";
        return -1;
    }
    sleep(3);

    // CREATE WINDOWS
    cout << "Done." << endl << "Creating windows... ";
    // Image display window
    namedWindow("Preview", CV_WINDOW_AUTOSIZE);
    // HSV adjust window
    namedWindow("Threshold", CV_WINDOW_AUTOSIZE);
    int threshHue=171, threshSat=125, threshVal=143;
    createTrackbar("Threshold Hue", "Threshold", &threshHue, 180);
    createTrackbar("Threshold Sat", "Threshold", &threshSat, 255);
    createTrackbar("Threshold Val", "Threshold", &threshVal, 255);

    // SET UP BLOB DETECTION
    SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 40;
    params.filterByArea = true;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.filterByCircularity = false;
    // Create the blob detector
    Ptr<SimpleBlobDetector> blobdetect = SimpleBlobDetector::create(params);

    // COLOUR TRAINING
    int rept = 1;
    while (rept == 1)
    {
        cout << "Done." << endl << "Taking preliminary image for colour recognition... ";

        // Get frame
        capture.grab();
        capture.retrieve(frame);
        if (frame.empty())
        {
            cout << "Error retrieving frame" << endl;
            return -1;
        }
        resize(frame,frame,Size(320,240),0,0,CV_INTER_AREA);
        //resize(frame,frame,Size(240,160),0,0,CV_INTER_AREA);
        flip(frame, frame, 0);
        imshow("Preview", frame);

        // Convert to HSV
        frame_hsv.create(frame.size(), frame.type());
        cvtColor(frame,frame_hsv,CV_BGR2HSV);
        cout << "Done." << endl << "Proceed with colour training. (r to repeat, q to continue)" << endl;

        while (1)
        {
            // Threshold the frame
            inRange(frame_hsv, Scalar(threshHue-7,threshSat,threshVal), Scalar(threshHue+7,255,255),frame_thresh);
            imshow("Threshold", frame_thresh);

            // Detect blobs
            blobdetect->detect(frame_thresh, keypoints);
            drawKeypoints(frame, keypoints, frame_processed, Scalar(0,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
            imshow("Preview",frame_processed);

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
                rept = 1;
                break;
            }
        }
    }

    // OBJECT TRACKING
    while (1)
    {
        // Grab frame from webcam
        capture.grab();
        capture.retrieve (frame);
        if(frame.empty()) break;
        resize(frame,frame,Size(320,240),0,0,CV_INTER_AREA);
        //resize(frame,frame,Size(240,160),0,0,CV_INTER_AREA);
        flip(frame,frame,0);

        // Convert to HSV
        frame_hsv.create(frame.size(), frame.type());
        cvtColor(frame,frame_hsv,CV_BGR2HSV);

        // Threshold the frame
        inRange(frame_hsv, Scalar(threshHue-7,threshSat,threshVal), Scalar(threshHue+7,255,255),frame_thresh);

        // Detect blobs
        blobdetect->detect(frame_thresh, keypoints);

        // Find mean of first 5 keypoints:
        float tot_x=0, tot_y=0, mean_x, mean_y, tot_size=0, mean_size;
        int no_kpts=0;
        for(no_kpts; no_kpts<keypoints.size() && no_kpts<=5; no_kpts++)
        {
            tot_x += keypoints[no_kpts].pt.x;
            tot_y += keypoints[no_kpts].pt.y;
            tot_size += keypoints[no_kpts].size;
        }
        mean_x = tot_x / no_kpts;
        mean_y = tot_y / no_kpts;
        mean_size = tot_size / no_kpts;


        // Return keypoint distance from centre
        Point2f centre = Point(frame.cols/2, frame.rows/2);
        Point2f mean_pt = Point(mean_x, mean_y);
        Point2f pt_err = centre - mean_pt;
        if(pt_err.x == 160 && pt_err.y == 120);
        {
            pt_err.x = 0;
            pt_err.y = 0;
        }

        cout << "Diff X:" << pt_err.x << " Y: " << pt_err.y << endl;

        // Draw on mean point
        circle(frame, mean_pt, mean_size, Scalar(0,0,0));

        // Display frame
        imshow("Preview", frame);

        // Exit if q pressed
        char c = waitKey(20);
        if( c == 113 )
        {
            cout << "User exit." << endl;
            break;
        }
    }
    cout << "Releasing camera... ";
    capture.release();
    cout << "Done." << endl;
    return 0;
}

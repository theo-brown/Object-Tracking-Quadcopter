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
    if (!capture.isOpened())
    {
        cout << "Failed to access webcam \n";
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
    int arealow=700,areahigh=30000,bloblow=240,blobhigh=780,blobd=100;
    createTrackbar("Threshold Hue", "Threshold", &threshHue, 180);
    createTrackbar("Threshold Sat", "Threshold", &threshSat, 255);
    createTrackbar("Threshold Val", "Threshold", &threshVal, 255);

    // SET UP BLOB DETECTION
    SimpleBlobDetector::Params params;
    //params.minDistBetweenBlobs = blobd;
    //params.minThreshold = bloblow;
    //params.maxThreshold = blobhigh;
    params.filterByArea = true;
    //params.minArea = arealow;
    //params.maxArea = areahigh;
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
        resize(frame,frame,Size(480,320),0,0,CV_INTER_AREA);
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
        resize(frame,frame,Size(480,320),0,0,CV_INTER_AREA);

        // Convert to HSV
        frame_hsv.create(frame.size(), frame.type());
        cvtColor(frame,frame_hsv,CV_BGR2HSV);

        // Threshold the frame
        inRange(frame_hsv, Scalar(threshHue-7,threshSat,threshVal), Scalar(threshHue+7,255,255),frame_thresh);

        // Detect blobs
        blobdetect->detect(frame_thresh, keypoints);
        drawKeypoints(frame, keypoints, frame_processed, Scalar(0,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        // Get coordinates
        if(keypoints.size() > 0 && keypoints.size() < 3) // Some large objects split into 2 keypoints, include these as 1
        {
            Point2f centre = Point(frame.cols/2,frame.rows/2);
            Point2f kpt = keypoints[0].pt;

            Point2f kpt_err;
            kpt_err = centre - kpt;

            cout << "Diff X:" << kpt_err.x << " Y: " << kpt_err.y << endl;
        }

        // Display frame
        imshow("Preview", frame_processed);

        // Exit if q pressed
        char c = waitKey(20);
        if( c == 113 )
        {
            cout << "User exit." << endl;
            break;
        }
    }
    capture.release();
    return 0;
}


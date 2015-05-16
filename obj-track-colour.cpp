#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace cv;
using namespace std;

int main()
{
    // Initialise capture device
    VideoCapture capture(0);
    if(!capture.isOpened())
    {
        cout << "Failed to access webcam \n";
        return -1;
    }
    sleep(3);
    cout << "Done." << endl << "Creating windows... ";

    // Image display window
    namedWindow("Preview", CV_WINDOW_AUTOSIZE);

    // HSV adjust window
    namedWindow("HSV", CV_WINDOW_AUTOSIZE);
    int threshHue=171, threshSat=125, threshVal=143;
    int arealow=50,areahigh=10000,bloblow=120,blobhigh=240,blobd=50;
    createTrackbar("Threshold Hue", "HSV", &threshHue, 180);
    createTrackbar("Threshold Sat", "HSV", &threshSat, 255);
    createTrackbar("Threshold Val", "HSV", &threshVal, 255);

    // Images
    Mat frame, frame_hsv, frame_thresh, frame_processed;

    // Blob detection
    SimpleBlobDetector::Params params;
    // Set area filtering
    params.minDistBetweenBlobs = blobd;
    params.minThreshold = bloblow;
    params.maxThreshold = blobhigh;
    params.filterByArea = true;
    params.minArea = arealow;
    params.maxArea = areahigh;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.filterByCircularity = false;
    Ptr<SimpleBlobDetector> blobdetect = SimpleBlobDetector::create(params);

    vector<KeyPoint> keypoints;

    cout << "Done." << endl << "Taking preliminary image for colour recognition... ";
    // Training
    capture.read(frame);
    if (frame.empty()) {cout << "Error retrieving frame" << endl; return -1;}
    imshow("Preview", frame);
    // Convert to HSV
    frame_hsv.create(frame.size(), frame.type());
    cvtColor(frame,frame_hsv,CV_BGR2HSV);
    cout << "Done." << endl << "Proceed with colour training." << endl;
    while (1)
    {
        // Threshold the frame
        inRange(frame_hsv, Scalar(threshHue-7,threshSat,threshVal), Scalar(threshHue+7,255,255),frame_thresh);

        imshow("HSV", frame_thresh);
        char c = waitKey(20);
        if( c == 113 )
        {
            cout << "Proceeding to object tracking." << endl;
            destroyWindow("HSV");
            waitKey(1);
            break;
        }
    }

    while (1)
    {
        // Grab frame from webcam
        int read = capture.read(frame);
        if(read != 1) break;
        if(frame.empty()) break;
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
        //imshow("HSV", frame_thresh);

        // Exit if q pressed
        char c = waitKey(20);
        if( c == 113 )
        {
            cout << "User exit." << endl;
            break;
        }
    }
    return 0;
}


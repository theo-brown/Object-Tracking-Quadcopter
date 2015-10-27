#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "raspicam/raspicam_cv.h"

using namespace cv;
using namespace std;

raspicam::RaspiCam_Cv pi_camera;

int camera_init()
{
    // Starts up the Pi camera
    cout << "Initialising Pi camera... ";

    pi_camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);

    pi_camera.open();
    sleep(1);

    if (!pi_camera.isOpened())
    {
        cout << "Failed to access Pi Camera." << endl;
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
        cout << "Error retrieving frame." << endl;
        return img;
    }

    resize(img.captured, img.captured, Size(IMG_WIDTH,IMG_HEIGHT), 0, 0, CV_INTER_AREA);
    flip(img.captured, img.captured, 0);

    return img;
}

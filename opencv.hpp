#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace cv;
using namespace std;

// Create object for eroding (2x2 rectangle)
//Mat erode_rect = getStructuringElement(MORPH_RECT,Size(2,2));
// Create object for dilating (5x5 rectangle)
//Mat dilate_rect = getStructuringElement(MORPH_RECT,Size(5,5));

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
    vector<vector<Point> > contrs;
    obj_point object;
};

frame detect_obj (frame img, int hue, int sat, int val)
{
    // Convert to HSV
    img.hsv.create(img.captured.size(), img.captured.type());
    cvtColor(img.captured, img.hsv, CV_BGR2HSV);

    // Threshold the frame so only specified HSV displayed
    inRange(img.hsv, Scalar(hue-7, sat, val), Scalar(hue+7, 255, 255), img.thresholded);

    // Erode and dilate image
    //erode(img.thresholded, img.thresholded, erode_rect, Point(-1,-1), 2);
    //dilate(img.thresholded, img.thresholded, dilate_rect, Point(-1,-1), 2);

    // Copy it for modifying - findContours modifies the image
    img.thresholded.copyTo(img.processed);

    // Contour the image
    findContours(img.processed, img.contrs, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    int contour_no=0, largest_contour_no, largest_contour_area=0;
    // Iterate through the contours
    for(contour_no; contour_no < img.contrs.size(); contour_no++)
    {
        // Find the largest contour
        int current_area = contourArea(img.contrs[contour_no]);
	if(current_area > largest_contour_area)
	{
            largest_contour_area = current_area;
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

# obj-tracking
Object tracking using OpenCV

PiCTO (Pi Colour Tracking of Objects) uses HSV colour thresholding to track objects with the Raspberry Pi camera.

PiCTO.cpp currently will transmit PWM signals as if to control the yaw of a quadcopter (testing)


Compile with: 
g++ PiCTO.cpp -o PiCTO -I/usr/local/include/ -lraspicam -lraspicam_cv -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lpigpio -lpthread -lrt -std=c++11

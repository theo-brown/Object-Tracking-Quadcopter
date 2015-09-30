# obj-tracking
Object tracking using OpenCV

PiCT (Pi Colour Tracking) uses HSV colour thresholding to track objects with the Raspberry Pi camera.

PiCT.cpp currently will transmit PWM signals as if to control the yaw of a quadcopter (testing)
PiCT-old.cpp is totally stable and will return the distance of the object from the centre

Compile with: 
g++ PiCT.cpp -o PiCT -I/usr/local/include/ -lraspicam -lraspicam_cv -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lpigpio -lpthread -lrt

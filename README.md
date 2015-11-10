########################################
# PiCTO - Object tracking using OpenCV #
########################################

PiCTO (Pi Colour Tracking of Objects) uses colour thresholding to track objects with the Raspberry Pi camera.

Requires yaw input of quadcopter flight controller connected to pin 17, Raspberry Pi camera plugged in to Pi.

### Usage ###
Preliminary stage: Use the sliders to set the HSV value until the green circle (Preview window) is drawn around the object. Press (r) to take another image
When finished, arm the quadcopter (a) and press (q) to continue.
PiCTO then will run until (q) is pressed again, controlling the yaw of the quadcopter to track the object of that colour.

### PiCTO structure: ###
 - Take frame from PiCamera
 - Threshold image based on user-input colour (HSV) values
 - Detect objects using image contouring and moment calculations
 - Use calculated object centre to find error (distance from centre)
 - Make PWM adjustments to move object to centre

### Files: ###
- PiCTO.cpp  - Main object tracking file
- camera.hpp - Header file containing code for controlling Raspberry Pi camera
- quad.hpp   - Header file containing code that relates to quadcopter control (PIGPIO)
- opencv.hpp - Header file containing all code relating to OpenCV image processing
- pid.hpp    - Header file containing the PID algorithm


Compile with: 
g++ PiCTO.cpp -o PiCTO -I/usr/local/include/ -lraspicam -lraspicam_cv -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lpigpio -lpthread -lrt -std=c++11

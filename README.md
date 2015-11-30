##########################################
# OCTO-pi - Object tracking using OpenCV #
##########################################

OCTO-pi (OpenCV Coloured Object Tracking) uses colour thresholding to track objects with the Raspberry Pi camera.

### Preliminary setup ###

- Connect Raspberry Pi Camera to CSI port on the Pi
- Connect yaw signal input (white) of flight controller to GPIO pin 17
- Connect yaw ground (black) from the flight controller to GPIO GND pin
- Power up the Pi using the microUSB power port
- Run PiCTO on the Raspberry Pi (`sudo ./PiCTO` - superuser privileges are required for GPIO control)
- After the Pi begins to ouput neutral PWM (signified by `Initialising PIGPIO and neutral throttle... Done`) connect the yaw power output (red) from the flight controller to 5V pin on the Pi, and remove microUSB power


### Usage ###

_Setup stage:_ 

To track an object, OCTO-pi uses specific colour values - Hue, Saturation and Value. These identify a precise shade of the colour so objects of that colour are identified from the image. 

Once OCTO-pi has been run, two windows will appear - `Preview` and `Threshold` - alongside the message `Taking preliminary image for colour recognition...`. OCTO-pi captures a frame from the camera and displays it in `Preview`. Use the sliders in `Threshold` to set the HSV value until the green circle is drawn around the object on the `Preview` window. Press (r) to take another image.

When finished, arm the quadcopter (a) and press (q) to continue (disarm the quadcopter using the (d) key).


_Run stage:_

OCTO-pi will then control the yaw of the quadcopter using PWM to point in the direction of the coloured object.

    To arm the quadcopter, press (a). 
    To disarm the quadcopter, press (d).
    To quit and end the program, press (q).

_PID adjustment:_

OCTO-pi supports PID tweaking while running.

    Increase P gain: (p)    Decrease P gain: (o)
    Increase I gain: (i)    Decrease I gain: (u)
    Increase D gain: (y)    Decrease D gain: (t)


### OCTO-pi structure: ###
 - Take frame from PiCamera
 - Threshold image based on user-input colour (HSV) values
 - Detect objects using image contouring and moment calculations
 - Use calculated object centre to find error (distance from centre)
 - Make PWM adjustments to move object to centre


### Files: ###
- OCTO-pi      - Executable program file
- OCTO-pi.cpp  - Main object tracking file
- camera.hpp - Header file containing code for controlling Raspberry Pi camera
- quad.hpp   - Header file containing code that relates to quadcopter control (using PIGPIO)
- opencv.hpp - Header file containing all code relating to OpenCV image processing
- pid.hpp    - Header file containing the PID algorithm


Compile with: 
g++ OCTO-pi.cpp -o OCTO-pi -I/usr/local/include/ -lraspicam -lraspicam_cv -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lpigpio -lpthread -lrt -std=c++11

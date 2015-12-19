##########################################
# OCTO-pi - Object tracking using OpenCV #
##########################################

OCTO-pi (OpenCV Coloured Object Tracking) uses colour thresholding to track objects with the Raspberry Pi camera. It's convenient to use with a PiTFT from adafruit - just use `sudo startx /path/to/OCTO-pi` to run from the terminal/command prompt.

### Preliminary setup ###

- Connect Raspberry Pi Camera to CSI port on the Pi
- Plug in PiTFT (if it's being used) - if not, connect three pushbuttons to GND on GPIO #6, #12 and #13 (BCM numbers)
- Connect yaw signal input (white) of flight controller to GPIO #17
- Connect yaw ground (black) from the flight controller to GPIO GND
- Power up the Pi using the microUSB power port
- Run OCTO-pi on the Raspberry Pi (`sudo ./OCTO-pi` - superuser privileges are required for GPIO control)
- After the Pi begins to ouput neutral PWM (signified by `Initialising PIGPIO and neutral throttle... Done`) connect the yaw power output (red) from the flight controller to 5V pin on the Pi, and remove microUSB power

If you want it to start automagically at boot, add `sudo startx /path/to/OCTO-pi` to the file /etc/rc.local.

### Usage ###

_Setup stage:_ 

To track an object, OCTO-pi uses specific colour values - Hue, Saturation and Value. These identify a precise shade of the colour so objects of that colour are identified from the image. 

Once OCTO-pi has been run, a window will appear alongside the message `Taking preliminary image for colour recognition...`. OCTO-pi captures a frame from the camera and displays it in the window. Use the slider to set the HSV value until the green circle is drawn around the object on the little preview. Press the button on pin 12 - (#12) - to take another image.

When finished, arm the quadcopter (#6) and press (#13) to continue (disarm the quadcopter using the (#6) again).


_Run stage:_

OCTO-pi will then control the yaw of the quadcopter using PWM to point in the direction of the coloured object.

    To arm the quadcopter, press (#6). 
    To disarm the quadcopter, press (#6) again.
    To quit and end the program, press (#13).
    To return to colour training - if you want to track another object - press (#12)

_PID adjustment:_

OCTO-pi supports PID tweaking while running. (This requires an attached keyboard)

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

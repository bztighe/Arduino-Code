# Walking Robot Motors
## Overall Concept
This is a collection of programs used to control various combinations of motors and control techniques for a cable actuated walking robot. It also contains programs used to test various components of the build.
## Programs
Click on each code for a 
# [Quad motors](#quad-pid)
d
dd
d
d

d
d
d

d
d

d

d
d
d
d
d

d
d
d
d
d

d
dd
d

dd

dd

dd

dd
d




# Quad Walking Motors PID <a name="quad-pid"></a>
## Concept
This code uses two feedback loop control systems known as PID controllers to precisely control the velocity and position of four 12v DC gear motors.The control system is named for the Proportional, Integral, and Derivative components of input error which are used to adjust a specific output. You can learn more about PID controllers [here](https://en.wikipedia.org/wiki/PID_controller "wikipedia link"). The code uses magnetic quadrature encoders as an input for the feedback loop system and two dual H-bridge motor drivers to control motor output. 
## Sub-routines
Included are sub-routines for:
- Symetric (simultainous) steps
- Staggered steps
- Left/right staggered steps
- Gripping and crawling motion
- Test motor function
- Printing serial data
- Plotting serial data
- Encoder function
- Velocity PID
- Position PID

## Hardware notes
This code cannot be used with an Arduino Uno, as it requires 4 interrupt pins (one for each motor encoder). Arduino MICRO, LEONARDO, and MEGA are all viable options. Note that if using a MICRO, two of the interrupt pins are located on the default serial communication pins (TX/RX) which causes the initial code upload to fail after using serial communication. This is remedied by simply uploading the code a second time. Also, PID constants Kp,Ki,Kd are specific to each system and will need to be redefined.This process is called "tunning" and can be done using Arduino's serial plotter functionality. 

# Walking Robot Motors
## Overall Concept
This is a collection of programs used to control various combinations of motors and control techniques for a cable actuated walking robot. It also contains programs used to test various components of the build.
## Programs
Click on each name to jump to the description:

- [Quad Motors Only](#quad-motors-only)
- [Dual Walking Robot Encoder](#dual-encoder)
- [Dual Walking Robot Encoder- alternate](#dual-encoder-alt)
- [Dual Walking Robot PID](#dual-pid)
- [Quad Walking robot PID](#quad-pid)
- [Five Motor PID Control](#five-pid)

# Quad Motors Only <a name="quad-motors-only"></a>
## Concept
Useful for debugging, troubleshooting, and instrutional purposes, this program is stripped down to the minimal code required to operate the four motors via two dual motor drivers. It allows for PWM control of velocity and digital control for direction. 

# Dual Walking Robot Encoder <a name="dual-encoder"></a>
## Concept
This code uses a dual motor driver, two quadrature encoders to control two micro gear motors. The only feedback for the system is motor (cable) displacement, so if the motors have a varying load, they will lose syncronization. 

# Dual Walking Robot Encoder - alternate <a name="dual-encoder-alt"></a>
## Concept
This code uses a dual motor driver, two quadrature encoders to control two micro gear motors. The only feedback for the system is motor (cable) displacement, so if the motors have a varying load, they will lose syncronization. This code attempts to solve that problem using loops and if statements, but is inefficient at best.

# Dual Walking Robot PID <a name="dual-pid"></a>
## Concept
This code uses a dual motor driver, two quadrature encoders to control two micro gear motors. It also utilizes two nested feedback control systems known as PID controllers to precisely control the velocity and position of the motors. This allows accurately timed syncronization of motors, and leg actuation which is useful to attain different types of locomotion (i.e. straight, turn left, turn right, etc.)

## Methods
This program includes built in methods for:
- Symetric (simultainous) steps
- Test motor function
- Printing serial data
- Plotting serial data
- Encoder function
- Velocity PID
- Position PID

# Quad Walking Motors PID <a name="quad-pid"></a>
## Concept
This code uses two feedback loop control systems known as PID controllers to precisely control the velocity and position of four 12v DC gear motors.The control system is named for the Proportional, Integral, and Derivative components of input error which are used to adjust a specific output. You can learn more about PID controllers [here](https://en.wikipedia.org/wiki/PID_controller "wikipedia link"). The code uses magnetic quadrature encoders as an input for the feedback loop system and two dual H-bridge motor drivers to control motor output. 

## Methods
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

# Five Motor PID Control <a name="five-pid"></a>
## Concept
This code will be uploaded upon completion.

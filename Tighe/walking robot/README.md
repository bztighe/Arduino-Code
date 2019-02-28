# Walking Robot Motors
## Concept
This code uses two feedback loop control systems known as PID controllers to precisely control the velocity and position of four 12v DC gear motors.The control system is named for the Proportional, Integral, and Derivative components of input error which are used to adjust a specific output. You can learn more about PID controllers [here](https://en.wikipedia.org/wiki/PID_controller "wikipedia link"). The code uses magnetic quadrature encoders as an input for the feedback loop system and two dual H-bridge motor drivers to control motor output. 

## Hardware notes
This code cannot be used with an Arduino Uno, as it requires 4 interrupt pins (one for each motor encoder). Arduino MICRO, LEONARDO, and MEGA are all viable options. Note that if using a MICRO, two of the interrupt pins are located on the default serial communication pins (TX/RX) which causes the initial code upload to fail after using serial communication. This is remedied by simply uploading the code a second time. 

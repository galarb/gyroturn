# Robot PID control function

This Library provides a simple PID control function for Robot control (using Gyro, Ultrasonic, Optical Ecoder, Line tracking sensor, as inputs to the PID)
 
Supports HC-06 Bloutooth module with Clicli embedded (see github.com/galarb/clicli). Currently Clicli they do not work together. (bug which I could not solve yet)

Class Functionalities:
- the Object constructor, gyroturn() expects the following integers: in1-4, enA (L Motor) and enB (R Motor), pin for encoder sensor, Echo and Trig, Line tracking Analog Pin
Note: enA and enB must be PWM pins!
these correspond to L298N motor driver pins for direction and speed control.
note that pins 11 and 13 are reserved for the BT module. rxPin = 11, and txPin = 13

- void begin(bdrate) is used to initiate the serial monitor at the specified baud rate.
Bluethooth is also started at the same baud rate.

- gotoang(deg, timer) is the main method. It expects the setpoint angle (deg), and the number of corrections you wish to have.
- steer(deg, speed, Kp, Ki, Kd) provides a simple interface to drive the robot on a steady angle.
- goUlrasonic
- goencoder



HMI:
LCD support, showing the error, Setpoint and current readings of various functions.
Bluetooth commands via similar interface as Clicli.
NRF24L01 radio communications with Joystick

PID control:
In the Current Version full P, I and D are implemented (Kp = 1, Ki=2, Kd=0).

# Robot Gyro Turn function with PID control

This Library provides a simple PID control function for Robot turning 
using MPU6050 Gyro control

Class Functionalities:
- the Object constructor, gyroturn() expects 6 integers, in1-4,enA and enB.
Note: enA and enB must be PWM pins!

these correspond to L298N motor driver pins for direction and speed control.
the Setpoint (sp) is set by the user, Gyro monitors the angle, and the speed of the motors is controled by the PID output, 

- void begin(PRO, INT, DIF) is used to initiate the PID algorithm, with PRO for Proportional, INT for integral, and DIFF for Diferential
It also starts the serial monitor at 115200 baud rate.

- gotoang(deg, timer) is the main method. It expects the setpoint angle (deg), and the number of corrections you wish to have.

HMI:
LCD support, showing the error and the current Gyro location as well as the Setpoint
Serial Print with the motors speed and direction, currently suppressed.

PID control:
In the Current Version P, I and D are implemented (Kp = 2, Ki=2, Kd=20).

the examples:
now includes a flag that ensures this command is only executed once.

Here are cumError values (Ki=1), for reference:
cumError =-1.26
cumError =-8.37
cumError =-12.87
cumError =-19.44
cumError =-24.57
cumError =-31.59
cumError =-36.18
cumError =-42.75
cumError =-47.97
cumError =-54.45

# Robot PID control function

This Library provides a simple PID control function for Robot turning 
using MPU6050 Gyro control
Supports HC-06 Bloutooth module with Clicli embedded (see github.com/galarb/clicli)

Class Functionalities:
- the Object constructor, gyroturn() expects the following integers: in1-4, enA (L Motor) and enB (R Motor), pin for encoder sensor, Echo and Trig
Note: enA and enB must be PWM pins!

these correspond to L298N motor driver pins for direction and speed control.

- void begin(bdrate) is used to initiate the serial monitor at the specified baud rate.
Bluethooth is also started at the same baud rate.

- gotoang(deg, timer) is the main method. It expects the setpoint angle (deg), and the number of corrections you wish to have.
- steer(deg, speed, Kp, Ki, Kd) provides a simple interface to drive the robot on a steady angle.


HMI:
LCD support, showing the error and the current Gyro location as well as the Setpoint
Serial Print with the motors speed and direction, currently suppressed.

PID control:
In the Current Version P, I and D are implemented (Kp = 2, Ki=2, Kd=20).

the examples:
now includes a flag that ensures this command is only executed once.

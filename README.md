# Gyro Turn with PID control

This Library provides a simple PID control function for Robot turning 
using MPU6050 Gyro control

the Object constructor, gyroturn() expects 6 integers, in1-4,enA and enB.
Note: enA and enB must be PWM pins!

these correspond to L298N motor driver pins for direction and speed control.
the Setpoint (sp) is set by the user, Gyro monitors the angle, and the speed of the motors is controled by the PID output, 

HMI:
LCD support, showing the error and the current Gyro location.
Serial Print with the motors speed and direction.

PID control:
In the Current Version P and D are implemented (Kp = 1.5, Ki=0, Kd=100).
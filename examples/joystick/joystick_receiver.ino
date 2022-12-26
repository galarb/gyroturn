#include "gyroturn.h"
gyroturn myturn(2, 4, 7, 8, 3, 6, 0, 0, 0, 0);  
//in1, in2, in3, in4, enA, enB in L298N, Encoder Pin, Trig, Echo, Reflected Light Sensor
//note that  BT HC-06 is reserved rxPin = 11, and txPin = 13;

void setup() {
myturn.begin(115200); //Baudrate for both Bluetooth and Serial Monitor
myturn.joystickradio(true);
}

void loop() { 
 myturn.joystickRadioCheck();
 //delay (000);
 //myturn.gotoang(20, 260); //Angle, and how many iterations to get there.
 ////myturn.steer(0, 120, 2, 1.5, 0) ; // Heading, Power (0-255), Kp, Ki, Kd for steering
 //myturn.btcheck();
 //myturn.goUltrasonic(14, 0, 200, 3, 0, 0); 
 //myturn.goline(127, 100, 2, 1.5, 0);//color, power, Kp, Ki, Kd 
 //Serial.println(myturn.getColor());
 //Serial.println(myturn.getDis());
 //myturn.goencoder(100, 1, 0, 0); //clicks, Kp, Ki, Kd for encoder response
 delay(50);
 }


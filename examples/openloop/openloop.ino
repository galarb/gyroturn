#include <gyroturn.h>

gyroturn myturn(2, 4, 7, 8, 3, 6, 9, 12, 10, 0);  
//in1, in2, in3, in4, enA, enB in L298N, Encoder Pin, Trig, Echo, Reflected Light Sensor
//note that  BT HC-06 is reserved rxPin = 11, and txPin = 13;
//make sure enA and enB are PWM pins

int sp = 90; // setpoint at 90
void setup() {
myturn.begin(9600); 
}

void loop() { 
  
   myturn.right(100); //go right
   delay(30);  
	 myturn.stop();
   delay(2000);

  
}
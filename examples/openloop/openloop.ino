#include "gyroturn.h"

gyroturn myturn(2, 4, 7, 8, 3, 6);  //in1, in2, in3, in4, enA, enB in L298N. 
//make sure enA nd enB are PWM pins

int sp = 90; // setpoint at 90
void setup() {
myturn.begin(0, 0, 0); // starts serial monitor at 115200
}

void loop() { 
  
   myturn.right(100); //go right
   delay(30);  
  
}
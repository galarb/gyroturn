

#include "gyroturn.h"
gyroturn myturn(2, 4, 7, 8, 3, 6);  //in1, in2, in3, in4, enA, enB in L298N. 


void setup() {
myturn.begin(3, 0.007, 1); //Kp, Ki, Kd

}

void loop() { 
  
 myturn.gotoang(-90);
  delay(100);
 
   
 }
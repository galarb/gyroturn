

#include "gyroturn.h"
gyroturn myturn(2, 4, 7, 8, 3, 5);  //in1, in2, in3, in4, enA, enB in L298N. 


void setup() {
myturn.begin(); //starts serial monitor at 115200
}

void loop() { 
  
 myturn.gotoang(90);
  delay(100);
   
 }
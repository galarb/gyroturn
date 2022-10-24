
#include "gyroturn.h"
gyroturn myturn(2, 4, 7, 8, 3, 6, 9);  //in1, in2, in3, in4, enA, enB in L298N, Encoder Pin

void setup() {
myturn.begin(2, 0, 0); //Kp, Ki, Kd defaults for Spin Turns

}

void loop() { 
 delay (30);
 myturn.goencoder(100, 1, 0, 0); //clicks, Kp, Ki, Kd for encoder response
 }


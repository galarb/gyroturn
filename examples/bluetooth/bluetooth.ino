
#include <gyroturn.h>
gyroturn myturn(2, 4, 7, 8, 3, 6, 9);  //in1, in2, in3, in4, enA, enB in L298N, Encoder Pin

void setup() {
myturn.begin(2, 0, 0); //Kp, Ki, Kd defaults for Spin Turns

}

void loop() { 
 myturn.btcheck(true);
 delay (30);
 }


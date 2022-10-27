
#include <gyroturn.h>
gyroturn myturn(2, 4, 7, 8, 3, 6, 9);  //in1, in2, in3, in4, enA, enB in L298N, Encoder Pin

void setup() {
myturn.begin(2, 0, 0); //Kp, Ki, Kd defaults for Spin Turns

}

void loop() { 
 delay (30);
 //myturn.gotoang(-20, 260); //Angle, and how many iterations to get there.
 myturn.steer(0, 200, 2, 0, 0); // Heading, Power (0-255), Kp, Ki, Kd for steering
 }


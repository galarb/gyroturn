
#include <gyroturn.h>
gyroturn myturn(2, 4, 7, 8, 3, 6, 9, 12, 10);  
//in1, in2, in3, in4, enA (L motor), enB (R Motor)in L298N, Encoder Pin, Rx, Tx 
//note that in current version BT HC-06 is reserved rxPin = 11, and txPin = 13;

void setup() {
myturn.begin(9600); //Baudrate for both Bluetooth and Serial Monitor
}

void loop() { 
 delay (30);
 myturn.steer(0, 100, 2, 1, 0); // Heading, Power (0-255), Kp, Ki, Kd for steering
 }


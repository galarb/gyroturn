

#include <gyroturn.h>

gyroturn myturn(2, 4, 7, 8, 3, 6, 9, 12, 10);  
//in1, in2, in3, in4, enA (L motor), enB (R Motor)in L298N, Encoder Pin, Rx, Tx 
//note that in current version BT HC-06 is reserved rxPin = 11, and txPin = 13;

void setup() {
myturn.begin(9600); 

}

void loop() { 
  
 myturn.gotoang(-90);
  delay(100);
 
   
 }
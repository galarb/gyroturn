#include "gyroturn.h"

gyroturn myturn(2, 4, 7, 8, 3, 6);  //in1, in2, in3, in4, enA, enB in L298N. 
//make sure enA nd enB are PWM pins

int sp = 90; // setpoint at 90
void setup() {
myturn.begin(0, 0, 0); // starts serial monitor at 115200
}

void loop() { 
  int currentYaw = myturn.getYaw();
  Serial.println(currentYaw);
  myturn.lcdershow(0, 0, currentYaw); 
  delay(50);  
  if (currentYaw < sp){
   myturn.right(200); //go right
   delay(30);  
  }
  if (currentYaw > sp){
    Serial.println("going left");
   myturn.left(200); //go left
  }
  if (currentYaw == sp){
   myturn.stop(); //go left
   delay(30);  
  }
}
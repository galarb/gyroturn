

#include "gyroturn.h"
gyroturn myturn;  

void setup() {
myturn.begin();
}

void loop() { 
  //myturn.run();
  
  //myturn.gotodeg(90);
  Serial.println(myturn.getYaw());
   delay(1000);
 }
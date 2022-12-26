

/* ****    Wireless Joystick 
**** Working on 2.4GHz with nRF24L01 Radio Module
*  /////       Transmitter Code for Arduino Nano (or Uno)

**** By: Gal Arbel
**** 2022-2023
*/

# include "GalJoystic.h"
GalJoystic myjoystic(100);

void setup() {

myjoystic.begin(115200);
}



void loop() {

  myjoystic.joyrun();
  delay(100);
}

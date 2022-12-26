#include "HardwareSerial.h"
#include "GalJoystic.h"
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include "Arduino.h"
#include <Wire.h>
#include "WString.h"
#include <SPI.h>

int control[6];//Array for sending the direction command, speed and BTN states in one packet
//control[0]: direction, [1]: speed, [2]: btn1, [3]: btn2, [4,5]: reserved
int x1Value = 0 ;
int y1Value = 0 ; 
bool b1Value = 0 ;
int x2Value = 0 ;
int y2Value = 0 ; 
bool b2Value = 0 ;

int VRx1 = A1; //nano pin
int VRy1 = A0; //nano pin
int BTN1 = 7;
int VRx2 = A2; //nano pin
int VRy2 = A3; //nano pin
int BTN2 = 6;
int speed = 0;
int direction = 0;
const byte address[6] = "00001";   

RF24 radio(8, 9); // CE, CSN on the radio



GalJoystic::GalJoystic(int sp) {
  speed = sp;
}


void GalJoystic::begin(double bdrate) {
 Serial.begin(bdrate); 
 pinMode(BTN1,INPUT_PULLUP);
 pinMode(BTN2,INPUT_PULLUP);
 while(!radio.begin()){Serial.println("radio failed  to begin");}                 
  radio.setPALevel(RF24_PA_MIN);
  radio.openWritingPipe(address);
  radio.stopListening();  
  //radio.printDetails();
  Serial.println("Joystick Srarted");
}

void GalJoystic::joyrun(){
 direction = 0; //reset direction at beginning  
/* if (radio.failureDetected) {
    radio.failureDetected = false;
    delay(250);
    Serial.println("Radio failure detected, restarting radio");
    radio.begin();
  }*/
 
 //Serial.println("Right Joustick.  X = Steering");
 x2Value = analogRead(VRx2);  //x2Value
 y2Value = analogRead(VRy2);  //y2Value
 b2Value = digitalRead(BTN2);  //b2Value
 //Serial.print("x2Value=");
 //Serial.println(x2Value);
 //Serial.print("b2Value=");
 //Serial.println(b2Value);
 x1Value = analogRead(VRx1); 
 y1Value = analogRead(VRy1); 
 b1Value = digitalRead(BTN1);  
 //Serial.print("y1Value=");
 //Serial.println(y1Value);
 //Serial.print("b1Value=");
 //Serial.println(b1Value);

 while (x2Value < 400 ) { // Steer left
  direction = direction - sqrt(abs(map(x2Value, 430, 0, 0, -180))); // changing the the output proportionally
  if (direction < -179) {
       direction = -179;
     }
  Serial.print("x2Value:");
  Serial.println(x2Value);
  Serial.print("direction:");
  Serial.println(direction);
  control[0] = direction;
  x2Value = analogRead(VRx2); 
  radio.write(&control, sizeof(control));
  delay(30);
 } 
 while (x2Value > 500) {// Steer Right
  direction = direction + sqrt(map(x2Value, 430, 880, 0, 180)); // 
  if (direction > 179) {
      direction = 179;
     }
  Serial.print("x2Value:");
  Serial.println(x2Value);
  Serial.print("direction:");
  Serial.println(direction);
  control[0] = direction;
  radio.write(&control, sizeof(control));
  x2Value = analogRead(VRx2); 
  delay(30);

  }

  // Serial.println("Left Joystick. Y = speed"); // SETTING THE SPEED
 
  if (!b2Value) {  
     Serial.println("Right Button Pressed");
     control[2] = 1; //Button definition in the array (True - pushed)
     radio.write(&control, sizeof(control));
     delay(30);

     } 
   else{
          control[2] = 0; //Button definition in the array (not pushed)

   } 
   if (!b1Value) {  
     Serial.println("Left Button Pressed");
     control[3] = 1; //Button definition in the array (True - pushed)
     radio.write(&control, sizeof(control));
     delay(30);
 
     } 
   else{
          control[3] = 0; //Button definition in the array (not pushed)

   }         
       
 while (y1Value < 400) { //reduce power
  speed = speed - sqrt(abs(map(y1Value, 440, 0, 0, -100))); //proportional change of speed
  if (speed < -100) {
       speed = -100;
     }
  Serial.print("y1Value:");
  Serial.println(y1Value);
  Serial.print("speed:");
  Serial.println(speed);
  control[1] = speed;
  y1Value = analogRead(VRy1); 
  radio.write(&control, sizeof(control));
  delay(30);

  }
 while (y1Value > 480) { // increse power
  speed = speed + sqrt(map(y1Value, 440, 870, 0, 100)); //proportional change of speed
  if (speed > 100) {
       speed = 100;
     }
  Serial.print("y1Value:");
  Serial.println(y1Value);
  Serial.print("speed:");
  Serial.println(speed);
  control[1] = speed;
  y1Value = analogRead(VRy1); 
  radio.write(&control, sizeof(control));
  delay(30);

 }
  
  Serial.print("transmitting...");
  if(radio.write(&control, sizeof(control))){Serial.println("...successfully");}
  else{
    Serial.println("...failure");
    radio.flush_tx();
    radio.reUseTX();
}
  delay(50);
}


void GalJoystic::printg(char dirR, char dirL, int speedR, int speedL){
  Serial.println("-------------------------------------");
  Serial.println("               | Direction |  Speed");
    Serial.print("Right Motor    |    ");
  Serial.print(dirR);
  Serial.print("      |  ");
  Serial.println(speedR);
  Serial.print("Left Motor     |    ");
  Serial.print(dirL);
  Serial.print("      |  ");
  Serial.println(speedL);
}


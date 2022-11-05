/**************************************************************************

            Robot Control software package for Arduino
						Featuring my very own	PID	Control
			      			 	 Using: Gyro MPU-6050
                            LCD 16x2
                            HC-06 Bluethooth
                            LM298N Motor Driver
                            hc-sr04 Ultrasonic module
                            LM393 Optical encoder
               
      by Gal Arbel
      2022


****************************************************************************/

#ifndef GYROTURN_H
#define GYROTURN_H
 
 class gyroturn {
  private:
   void lcdershow(int e, int g, int s); //show error 
   void lcdenshow(int c, int o, int t); //show encoder 
   void lcdusshow(int d, int s); //show Ultrasonic 
   void spin(int output);
   void right(int speedR, int speedL);
   void left(int speeR, int speedL);
   int in1 = 0; 
   int in2 = 0;
   int in3 = 0;
   int in4 = 0;
   int enA = 0;
   int enB = 0;
   int txPin = 0;
   int rxPin = 0;
   int encoderPin = 0;
   int trigPin = 0;
   int echoPin = 0;
   void printg(char dirR, char dirL, int speedR, int speedL);//internal info printing procedure
  public:
   gyroturn(int dirRA, int dirRB, int dirLA, int dirLB, int speedR, int speedL, int encodPin, int TrigPin, int EchoPin);
   void begin(int bdrate); //will start both Bluetooth and Serial at this Baud rate
   void gotoang(int deg, int timer, double KP, double KI, double KD); //turn to a setpoint (angle), for #of correctios
   int getYaw(); //return the Yaw angle
   int getDis(); //return the Ultrasonic distance
   int getSteps(); 
   int getTemp();
   double PIDcalc(double inp, int sp);
   void stop();
   void fwd(int pwrR, int pwrL);
   void bckwd(int pwrR, int pwrL);
   void steer(int deg, int power, double KP, double KI, double KD);
   void goencoder(int clicks, double KP, double KI, double KD);
   void btcheck(bool onoff);
   void goUltrasonic(int dis, int deg, int power, double KP, double KI, double KD);
   void stripled (int lednum, int red, int green, int blue);
   void neopixels (int red, int green, int blue);

 };
#endif 
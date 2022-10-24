#ifndef GYROTURN_H
#define GYROTURN_H
 
 class gyroturn {
  private:
   void lcdershow(int e, int g, int s); //show error 
   void lcdenshow(int c, int o, int t);//show encoder 
   void spin(int output);
   void right(int speedR, int speedL);
   void left(int speeR, int speedL);
   int in1 = 0; 
   int in2 = 0;
   int in3 = 0;
   int in4 = 0;
   int enA = 0;
   int enB = 0;
   int encoderPin = 0;
   void printg(char dirR, char dirL, int speedR, int speedL);//internal info printing procedure
  public:
   gyroturn(int dirRA, int dirRB, int dirLA, int dirLB, int speedR, int speedL, int encodPin);
   void begin(double PRO, double INT, double DIF);   
   void gotoang(int deg, int timer); //turn to a setpoint (angle), for #of correctios
   int getYaw(); //return the Yaw angle
   int getSteps(); 
   int getTemp();
   double PIDcalc(double inp, int sp);
   void stop();
   void fwd(int pwrR, int pwrL);
   void steer(int deg, int power, double KP, double KI, double KD);
   void goencoder(int clicks, double KP, double KI, double KD);
   void btcheck(bool onoff);

 };
#endif 
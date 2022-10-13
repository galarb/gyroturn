#ifndef GYROTURN_H
#define GYROTURN_H
 
 class gyroturn {
  private:
   void lcdershow(int e, int g);
   double PIDcalc(double inp, int sp);
   void spin(int output);
   void right(int speed);
   void left(int speed);
   int in1 = 0; 
   int in2 = 0;
   int in3 = 0;
   int in4 = 0;
   int enA = 0;
   int enB = 0;
   void printg(char dirR, char dirL, int speedR, int speedL);//internal info printing procedure
  public:
   gyroturn(int dirRA, int dirRB, int dirLA, int dirLB, int speedR, int speedL);
   void begin(double PRO, double INT, double DIF);   
   void gotoang(int deg); //turn to a setpoint (angle)
   int getYaw(); //return the Yaw angle
   void gyroreset();
   

 };
#endif 
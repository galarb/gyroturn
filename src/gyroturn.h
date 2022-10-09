#ifndef GYROTURN_H
#define GYROTURN_H
 
 class gyroturn {
  private:
   int deg = 0;
  public:
   gyroturn();
   void begin();   
   int gotodeg(int deg); //turn to degree
   float getTemp();
   int getYaw(); //return the Yaw angle
   void gyroreset();
  

 };
#endif 
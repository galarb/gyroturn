#ifndef GYROTURN_H
#define GYROTURN_H
 
 class gyroturn {
  private:
    int deg = 0;
    int axelerometer_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat);
  public:
   gyroturn();
   void begin();   
   int gotodeg(int deg); //turn to degree
   float getTemp();
   int getYaw();

  

 };
#endif 
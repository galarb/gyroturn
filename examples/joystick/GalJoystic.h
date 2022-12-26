/**************************************************************************

            Robot Joystic Transmitter software package for Arduino Nano
						
                            
               
      by Gal Arbel
      2022


****************************************************************************/

#ifndef GALJOYSTIC_H
#define GALJOYSTIC_H
 
 class GalJoystic {
  private:
   void printg(char dirR, char dirL, int speedR, int speedL);//internal info printing procedure

  public:
   GalJoystic(int sp);
   void begin(double bdrate); //will start Serial at this Baud rate
   void joyrun(); //
   

 };
#endif 
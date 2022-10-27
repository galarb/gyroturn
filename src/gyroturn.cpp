/*


                   Robot Control function 
								PID	Software Control for Arduino
			      			 	 Using: Gyro MPU-6050
                            LCD 16x2
                            HC-06 Bluethooth
                            LM298N Motor Driver
                      
               
      by Gal Arbel
      2022


*/


#include "gyroturn.h"
#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <LiquidCrystal_I2C.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <clicli.h>

unsigned long currentTime;
unsigned long previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double cumError, rateError;
double kp = 0;
double ki = 0; 
double kd = 0;
bool flag = true;
int steps = 0;
const byte rxPin = 11;
const byte txPin = 13;
int btdata;

MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27,16,2);
SoftwareSerial BTSerial(rxPin, txPin); // RX TX
clicli mycli;  


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector





gyroturn::gyroturn(int dirRA, int dirRB, int dirLA, int dirLB, int speedR, int speedL, int encodPin) {
  in1 = dirRA;
  in2 = dirRB;
  in3 = dirLA;
  in4 = dirLB;
  enA = speedR;
  enB = speedL;
  encoderPin = encodPin;
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(encoderPin, INPUT_PULLUP);
}


void gyroturn::begin(double PRO, double INT, double DIF) {
   // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);
    BTSerial.begin(9600);

    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    // Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-1376);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-37);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

    
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
      
    }

  lcd.init();  
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Started");
  
  Serial.print("in1 = ");  
  Serial.println(in1);
  Serial.print("in2 = ");  
  Serial.println(in2);
  Serial.print("in3 = ");  
  Serial.println(in3);
  Serial.print("in4 = ");  
  Serial.println(in4);
  Serial.print("enA = ");  
  Serial.println(enA);
  Serial.print("enB = ");  
  Serial.println(enB);
  Serial.print("Encoder Pin = ");  
  Serial.println(encoderPin);
  
  mycli.begin();

  kp = PRO;
  ki = INT; 
  kd = DIF;
  previousTime = millis(); //otherwise the first Itegral value will be very high
}
void gyroturn::steer(int deg, int power, double KP, double KI, double KD){
  kp = KP;
  ki = KI; 
  kd = KD;
  int tempyaw = getYaw(); //input value
  int output = PIDcalc(tempyaw, deg); //output value = the calculated error
  lcdershow(deg, output, tempyaw);
  delay(30);
  if (output > 0){// right correction 
  int powerR = power - output;
  int powerL = power + output;
  if (powerR > 255) {powerR = 255;}
  if (powerR < 0) {powerR = 0;}
  if (powerL > 255) {powerL = 255;}
  if (powerL < 0) {powerL = 0;}
    printg('F', 'F', powerR, powerL);
    fwd(powerR, powerL);

  }
  else if (output < 0){//left correction 
  int powerR = power + output;
  int powerL = power - output;
  if (powerR > 255) {powerR = 255;}
  if (powerR < 0) {powerR = 0;}
  if (powerL > 255) {powerL = 255;}
  if (powerL < 0) {powerL = 0;}
    printg('F', 'F', powerR, powerL);
    fwd(powerR, powerL);

  }
  else {//go straight
    fwd(power, power);
    Serial.println("going FWD");
  }
}
void gyroturn::gotoang(int deg, int timer) {
  if(flag){
    for(int i = 0; i < timer;){
    int tempyaw = getYaw(); //input value
    int output = PIDcalc(tempyaw, deg); //output value, the calculated error
    lcdershow(deg, output, tempyaw);
    //Serial.print(output); //for serial plotter
    //Serial.print("\t"); //for serial plotter
    //Serial.print(deg); //for serial plotter
    //Serial.print("\t"); //for serial plotter


    spin(output);
    i++;
  }
  flag = false;
  stop();

 }
}

double gyroturn::PIDcalc(double inp, int sp){
   currentTime = millis();                //get current time
   elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
   //Serial.print(currentTime); //for serial plotter
   //Serial.println("\t"); //for serial plotter
   error = sp - inp;                                  // determine error
   cumError += error * elapsedTime;                   // compute integral
   rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
   double out = kp*error + ki*cumError + kd*rateError; //PID output               
 
   lastError = error;                                 //remember current error
   previousTime = currentTime;                        //remember current time
   return out;                                        //the function returns the PID output value 
  
}



void gyroturn::spin(int output){ //point spin (both motors at the same speed)


 if (output > 0){//right turn 
   //printg('B', 'F', output, output);
   right(output, output);

 }
 if (output < 0){//left turn 
  // printg('F', 'B', output, output);
   left(output, output);
 }

}

void gyroturn::right(int speedR, int speedL){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);   //R backward
  analogWrite(enA, abs(speedR));
  digitalWrite(in3, LOW);   //L forward
  digitalWrite(in4, HIGH);
  analogWrite(enB, abs(speedL));
}
void gyroturn::left(int speedR, int speedL){
  digitalWrite(in1, LOW);     //R forward
  digitalWrite(in2, HIGH);
  analogWrite(enA, abs(speedR));
  digitalWrite(in3, HIGH);    //L backward
  digitalWrite(in4, LOW);
  analogWrite(enB, abs(speedL));
}

void gyroturn::stop(){
  digitalWrite(in1, LOW);    
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
  digitalWrite(in3, LOW);   
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}

void gyroturn::fwd(int pwrR, int pwrL){
  digitalWrite(in1, HIGH);    
  digitalWrite(in2, LOW);
  analogWrite(enA, pwrR);
  digitalWrite(in3, HIGH);   
  digitalWrite(in4, LOW);
  analogWrite(enB, pwrL);
}

void gyroturn::goencoder(int clicks, double KP, double KI, double KD){
  kp = KP;
  ki = KI; 
  kd = KD;
  int tempsteps = getSteps(); //input value
  int output = PIDcalc(tempsteps, clicks); //output value = the calculated error
  lcdenshow(clicks, output, tempsteps);
  delay(30);
  if (output > 0){  // 
   fwd(output, output);
  }
  if (output < 0){  // 
   stop();
  }  
}
int gyroturn::getSteps(){
  if(digitalRead(encoderPin)){
     steps = steps +1;
     Serial.print("steps =");
     Serial.println(steps);
     while(digitalRead(encoderPin));
   }
  return steps;
}

int gyroturn::getYaw(){
  
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
             
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            return(ypr[0]* 180/M_PI);
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif
    }
}

int gyroturn::getTemp(){
  int t = mpu.getTemperature();
  return(t);
}

void gyroturn::lcdenshow(int c, int o, int t){ //setpoint, error, gyro
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SP | PID |encod");
  lcd.setCursor(0,1);
  lcd.print(c);  
  lcd.setCursor(3,1);
  lcd.print("|"); 
  lcd.setCursor(5,1);
  lcd.print(o); 
  lcd.setCursor(9,1);
  lcd.print("|");  
  lcd.setCursor(11,1);
  lcd.print(t); 
}

void gyroturn::lcdershow(int s, int e, int g){ //setpoint, error, gyro
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SP | PID | gyro");
  lcd.setCursor(0,1);
  lcd.print(s);  
  lcd.setCursor(3,1);
  lcd.print("|"); 
  lcd.setCursor(5,1);
  lcd.print(e); 
  lcd.setCursor(9,1);
  lcd.print("|");  
  lcd.setCursor(11,1);
  lcd.print(g); 
}
void gyroturn::printg(char dirR, char dirL, int speedR, int speedL){
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

void gyroturn::btcheck(bool onoff){
  
  
 /* while(Serial.available()){
    btdata = Serial.read();
    BTSerial.write(btdata);
  }*/
  while(BTSerial.available()) {
    btdata = BTSerial.read();
    Serial.write(btdata);
  }
  mycli.run();
}

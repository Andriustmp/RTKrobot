/*
 RTK robot fm ver: 1.2
 Autor: Andrius Besusparis 
 2024-02-21

 Arduino verssion 1.8.16
 
 used librarys for quadrature encoders : 
 https://github.com/rogerclarkmelbourne/Arduino_STM32

 https://github.com/jrowberg/i2cdevlib

 customization DFRobot_LSM303 library:
   DFRobot_LSM303()  -->  added &Wire2 obj
   readAcc(void)     -->  added LPF and AVG filter for acc readings
 
*/

#include "HardwareTimer.h"
//-------------------------
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


TwoWire Wire2(2,I2C_FAST_MODE );  // I2C_FAST_MODE (400 kHz), I2C port 2 ( PB10, PB11 )
MPU6050 mpu(0x68, &Wire2);        // <-- use for AD0 low, but 2nd Wire (TWI/I2C) object

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL

#define INTERRUPT_PIN PB1         // for MPU 6050 int

// MPU control/status vars
bool     dmpReady = false;  // set true if DMP init was successful
uint8_t  mpuIntStatus;      // holds actual interrupt status byte from MPU
uint8_t  devStatus;         // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;        // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;         // count of all bytes currently in FIFO
uint8_t  fifoBuffer[64];    // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//---- MPU6050 interupt---
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//------------------------------
//#include <OneWireSTM.h>         // for DS18
#include <DFRobot_LSM303.h>       // I2C mag sensor
#include <Wire.h>
#include <math.h>

//magnetometer_min = (DFRobot_LSM303::vector<int16_t>){-32767, -32767, -32767};
//magnetometer_max = (DFRobot_LSM303::vector<int16_t>){+32767, +32767, +32767};

DFRobot_LSM303 compass(&Wire2);

void interuptas(void);
byte Ebutton=0;           // E stop from button
byte Esensor=0;           // E stop from sensor
byte ESTOP=0;             // All motors Estop bit

int  PWM1=0;              // L motor pwm
int  PWM2=0;              // R motor pwm

bool M1dirSet=true;       // Command from serial input     
bool M2dirSet=true;       // true - fwd; false - rev 
bool M1dir=true;          // true - fwd; false - rev 
bool M2dir=true;
bool M1stop=false;
bool M2stop=false;

bool M1dirold=true;       // For direction change delay.
bool M2dirold=true;

int Dir1cnt=2;            // fast fix
int Dir2cnt=2;
int Dir1cnt2=3;
int Dir2cnt2=3;

bool cutMot=false;        // Cutting motor  ON/OFF
int  rxWdt=0;             // If serial not active turn motors off

bool WdtStop=true;        // Serial com check 

int setimp1=0;            // M1 speed encoder imp/100ms ( 80~ imp/100ms - 0.8m/s - 2.88 kmh )
int setimp2=0;

int BatU=0;               // Battery voltage (avg ) Fixed point
int BatI=0;  
int Batlow=100;           // 10.0V  Battery voltage minimum V
unsigned long t100ms=0;   // 100 ms tick 
unsigned long t100ms2=0;  // 100 ms tick2 

unsigned long time = 0;  

float  diravg= -1;           // Compass direction avg 
int    avgcnt=1; 
float  dir1=0;               // magnetometer direction 
unsigned int  dir1fused=0;   // Fused magnetometer and IMU direction
  
//------------- Compass  -----------
char  report[120]; 
float old=0;
float new2=0;
int   diff=0;
int   angle=0;
int   b=0;

void Compass1()  // 3 ms
{
  compass.read();    // Read MAG + ACC (LPF) reg
  avgcnt++;
  if(avgcnt>=45)
  {
   float new1= compass.getNavigationAngle();  // After ~ 135 ms get Heading angle
   //new2=int(0.7*old)+int(0.3*head1);        // LPF
   //old=new2;
  
   //diff = (( int(new1) - b + 180 + 360) % 360) - 180;
   // //angle = (360 + b + ( int(diff / 2) )) % 360;
   //angle = (360 + b + ( int(0.5*(diff / (0.5+0.5))) )) % 360;   // angular averange ( int(0.5*(diff / (0.5+0.5)))
   //b=angle;

   dir1=new1;  //new1;
   //Serial2.print("Dir Angle: ");
   //Serial2.println(dir1);  
   avgcnt=0;   
  }    
}

DFRobot_LSM303::vector<int16_t> magmin;   //compass.magnetometer.magnetometer_min;
// magmin=compass.magnetometer_min;
DFRobot_LSM303::vector<int16_t> magmax;   //compass.magnetometer.magnetometer_min;
// magmin=compass.magnetometer_min;
// 11:38:56.179 -> Xmin: -650 Ymin: -705 Zmin: -489

void CompassCal()
{
  compass.read();
 // snprintf(report, sizeof(report), "Acceleration:(X:%6d Y:%6d Z:%6d) Magnetometer:(X:%6d Y:%6d Z:%6d)",
 // compass.accelerometer.x, compass.accelerometer.y, compass.accelerometer.z,
 // compass.magnetometer.x, compass.magnetometer.y, compass.magnetometer.z);
 // Serial2.println(report);
  if (compass.magnetometer.x < magmin.x) magmin.x = compass.magnetometer.x; 
  if (compass.magnetometer.x > magmax.x) magmax.x = compass.magnetometer.x ;

  if (compass.magnetometer.y < magmin.y) magmin.y = compass.magnetometer.y;
  if (compass.magnetometer.y > magmax.y) magmax.y = compass.magnetometer.y ;

  if (compass.magnetometer.z < magmin.z) magmin.z = compass.magnetometer.z;
  if (compass.magnetometer.z > magmax.z) magmax.z = compass.magnetometer.z ;
  delay(200);
}
//------------ Encoder stuff --------------
#define PPR   600              // Encoder pulses/rev
HardwareTimer timer(1);        // Optical Encoder 1 ( L motor )
                               // Optical Encoder 2 ( R motor ) Implemented in maple Core
unsigned long ints = 10;
void func(){
if (timer.getDirection()){
    ints--;
  } else{
    ints++;
    }
}
unsigned long ints2 = 10;
void func2(){
if (Timer2.getDirection()){
    ints2--;
    } else{
      ints2++;
      }
}
//----------------------------------------------
// MPU6050 setup,  100 HZ update rate, DMP mode
//----------------------------------------------
void MPU6050setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire2.begin();
        Wire2.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    //Serial2.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    //Serial2.println(F("Testing device connections..."));
    //Serial2.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    //mpu.testConnection();
  
    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    //------
    //mpu.setRate(9);  // 4- 100 hz; 9 - 50 Hz
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial2.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        digitalPinToInterrupt(INTERRUPT_PIN);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial2.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        //-------- Packet size
       // mpu.setTempFIFOEnabled(0);
       // mpu.setAccelFIFOEnabled(0);
       // delay(50);
        
        // get expected DMP packet size for later comparison
        //packetSize = mpu.dmpGetFIFOPacketSize();
        //Serial2.println("pasket size:");
        //Serial2.println(packetSize);
        
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial2.print(F("DMP Initialization failed (code "));
        //Serial2.print(devStatus);
        //Serial2.println(F(")"));
    }
  
}
//------------- Init ------------------
void setup() {
  //Timer1 encoder input
  pinMode(PA9, INPUT);           // TIM1 channel A 
  pinMode(PA8, INPUT);           // TIM1 channel B 
  
  //Timer2 encoder input
  pinMode(PA0, INPUT);           // TIM2 channel A  
  pinMode(PA1, INPUT);           // TIM2 channel B   

  //------------ I/O pin ---------
  pinMode(PC13, OUTPUT);         // STM onboard LED green
  pinMode(PA5, OUTPUT);          // Cutting motor ON/OFF
  pinMode(PA7, OUTPUT);          // ERR LED red
  pinMode(PA6, OUTPUT);          // RUN LED green 

  pinMode(PA11, INPUT_PULLUP);   // E- STOP Button
  pinMode(PA12, INPUT_PULLUP);   // E- STOP Sensor 
  
  //--- External Status LED ----
  digitalWrite(PA7, HIGH);       // LOW - ON
  digitalWrite(PA6, HIGH);

  //---- Analog In ---------
  pinMode(PA4, INPUT_ANALOG);     // U Bat
  //pinMode(PA5, INPUT_ANALOG);   // I Bat

  //--- Timer3  100 ms interupt ----
  Timer3.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
  Timer3.setPeriod(100000);              // in microseconds
  Timer3.setCompare(TIMER_CH1, 1);       // overflow might be small
  Timer3.attachInterrupt(TIMER_CH1, interuptas);

  //----- Motor PWM Driver -----
  pinMode(PB6, OUTPUT);          // M1 Dir ( L motor )
  digitalWrite(PB6, HIGH); 
  
  pinMode(PB7, OUTPUT);          // M2 Dir  ( R motor )
  digitalWrite(PB7, HIGH); 
  
  pinMode(PB8, PWM);             // M1 PWM  timer4
  pinMode(PB9, PWM);             // M2 PWM  timer4

  Timer4.pause();                                    // stop the timer
  Timer4.setCount(0);
 // Timer4.setMode(TIMER_CH3, TIMER_OUTPUT_COMPARE); // T4 CH3
  Timer4.setPrescaleFactor(32);                      // input clock 
  Timer4.setOverflow(300);                           //  ~ 7.4  kHz PWM
  Timer4.setCompare(TIMER_CH3, (300/2));             // PWM value MAX 295 ! Motor driver limitation PWM max 98 %
  Timer4.refresh();
  Timer4.resume();

  pwmWrite(PB8, 0 );                                 // PWM OFF;  295 Driver limitation max 98 % PWM
  pwmWrite(PB9, 0 );  
                            
  //---configure timer1 as encoder ( L motor )
  timer.setMode(1, TIMER_ENCODER); //set mode, the channel is not used when in this mode (but it must be [1..4]). 
  timer.pause();                   //stop... 
  timer.setPrescaleFactor(4);      //normal for encoder to have the lowest or no prescaler. 
  timer.setOverflow(PPR);          //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  timer.setCount(0);               //reset the counter. 
  timer.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  timer.attachInterrupt(0, func);  //channel must be 0 here 
  timer.resume();                  //start the encoder... 
  timer.refresh();                 // Start position fix.

 //---configure timer2 as encoder ( R motor )
  Timer2.setMode(1, TIMER_ENCODER); //set mode, the channel is not used when in this mode (but it must be [1..4]). 
  Timer2.pause();                   //stop... 
  Timer2.setPrescaleFactor(4);      //normal for encoder to have the lowest or no prescaler. 
  Timer2.setOverflow(PPR);          //use this to match the number of pulse per revolution of the encoder. Most industrial use 1024 single channel steps. 
  Timer2.setCount(0);               //reset the counter. 
  Timer2.setEdgeCounting(TIMER_SMCR_SMS_ENCODER3); //or TIMER_SMCR_SMS_ENCODER1 or TIMER_SMCR_SMS_ENCODER2. This uses both channels to count and ascertain direction. 
  Timer2.attachInterrupt(0, func2); //channel must be 0 here 
  Timer2.resume();                  //start the encoder... 
  Timer2.refresh();                 // Start position fix.

 //---------------
  Serial2.begin(19200);          // UART2  (PA4, PA3)
  delay(500);
  MPU6050setup();                // TODO : tilt and impact emergency stop; com check stop !!!
  delay(200);

  Serial2.println("HW_started V1.2");
  delay(50);

 //----- Initialize Lsm303 ------- 
  compass.init(DFRobot_LSM303::device_DLHC, DFRobot_LSM303::sa0_high); // device_DLH, device_DLM, device_DLHC, device_D, or device_auto
                                                                       // device_auto - hangs cpu ( NACK response lib problem )
  delay(200);
  compass.enable();
  delay(200);
}

//----------- Support var ----------
unsigned long interval=0;  
unsigned long interval2=0;  
char received = 0;
uint32 a;

long  imp1=0;
long  imp2=0;
long  imp1old=0;
long  imp2old=0;
float rpm1=0;
float rpm2=0;

unsigned long Encoder1cnt=0;  // Full encoder pulses
unsigned long Encoder2cnt=0;

//--------- Simple PI for motors ----
// 745 imp/s -> 60 RPM 
float P=2.5;        // PID P
char  I=2;          // PID I 
long delta1 =0;
long delta2 =0;
int dirspeed=10;

//  Left motor
void Motor1Speed()
{
  if (M1dirSet!=M1dir)   // for direction change
  {
    setimp1=dirspeed; 
      if(imp1<=20) 
        {
          M1dir=M1dirSet;
        }
  }
  delta1=abs(imp1-setimp1); 
//----- P -----
  if ( (imp1 <setimp1)&&(delta1>=P))
     PWM1=PWM1+int(delta1*P);
   
  if ((imp1 >setimp1)&&(delta1>=P))
     PWM1=PWM1-int(delta1*P); 
//---- I ----
  if (imp1 <setimp1)
     PWM1=PWM1+I;
  if (imp1 >setimp1)
     PWM1=PWM1-I;
 
  if (PWM1>292) { PWM1=292; }      // H bridge limit max 292
  if (PWM1<0)   { PWM1=0; }  

  // Serial2.print("pwm: "); 
  // Serial2.println(PWM1);
  // Serial2.print("imp: "); 
  // Serial2.println(imp1);
}

// Right motor
void Motor2Speed()
{
   if (M2dirSet!=M2dir)  // for direction change
  {
    setimp2=dirspeed; 
      if(imp2<=20) 
        {
          M2dir=M2dirSet;
        }
  }
  delta2=abs(imp2-setimp2); 
//----- P -----
  if ((imp2 <setimp2)&&(delta2>=P))
   PWM2=PWM2+int(delta2*P);
   
  if ((imp2 >setimp2)&&(delta2>=P))
   PWM2=PWM2-int(delta2*P); 
//---- I ----
  if (imp2 <setimp2)
   PWM2=PWM2+I;
   if (imp2 >setimp2)
   PWM2=PWM2-I;
    
  if (PWM2>292) { PWM2=292; }  
  if (PWM2<0)   { PWM2=0; }     
}

//------- RPM + kelio skaiciavimas--------
void Rpm1()  // @100 ms
{
    long abstmp1=0;
  // if (timer.getDirection()==0)
     Encoder1cnt=ints * PPR + timer.getCount();
     abstmp1=Encoder1cnt-imp1old;
     imp1=abs(abstmp1);       // arduino macros...
     imp1old=Encoder1cnt;   
}
void Rpm2()  // @100 ms
{   
    long abstmp2=0; 
 // if (timer.getDirection()==0)
     Encoder2cnt=ints2 * PPR + Timer2.getCount();
     abstmp2=Encoder2cnt-imp2old;
     imp2=abs(abstmp2);
     imp2old=Encoder2cnt;
}

//---------- 100 ms interupt ------
void interuptas(void)
{
  Rpm1();        // M1 encoder pulses calc
  Rpm2();        // M2 encoder pulses calc
  Motor1Speed();  
  Motor2Speed();
  Wdt();
  Motors();
  t100ms=t100ms+1;

  if(t100ms2>=0) t100ms2=t100ms2+1;    // For status led
  if(t100ms2>=10) t100ms2=0;  
}

//--------- Serial com watchdog  2s ----
void Wdt()
{
  rxWdt--;        // Count down
  if(rxWdt<=0)    // Serial com no RX or failure, STOP
  {
    rxWdt=0;
    WdtStop=true; 
  }
  if(rxWdt>=20) WdtStop=false;  // Serial com resumed
}

//-------- Emergency STOP input -----
void Avariniai()
{
  if( digitalRead(PA11) >=1 ) {      // "0" - ok "1"- stop
     Ebutton = 1;  
     ESTOP=1;
  } 
  if( digitalRead(PA12) >=1 ) {
     Esensor = 1;  
     ESTOP=1;
  }  
}

//----------- Analog U/I -------------
int  Uavg=0;
int  Umes=0;
void AnalogIn()
{  
  Uavg = Uavg + analogRead(PA4);        // 945 - 10.0 V;  (12 bit 4096) 1 bit  0.010582 V
  Umes++;
  if (Umes>=10)
   {
     BatU=((Uavg/10)* 0.010582)*10;     // DEC fixed point 10.0V
     Umes=0;
     Uavg=0;
     //Serial2.print("U: ");
     //Serial2.println(BatU);
   }
}
int ramp1=0;
int ramp2=0;
int stopped1=0;

int  DelayStart=0;
bool DelayBuz=false;

//---------- Motors controll and safety stop  @100 ms --------
void Motors()
{
   if((ESTOP==0) &&( WdtStop==false)) 
   {     
    digitalWrite(PB6, !M1dir);   // M1 direction, invert direction       
    pwmWrite(PB8, PWM1 );        // M1 PWM
    
    digitalWrite(PB7, !M2dir);   // M2 direction, invert direction   
    pwmWrite(PB9, PWM2 );        // M2 PWM
       
    //digitalWrite(PA5, cutMot);   // Cutting motor Start/Stop 
    CuttingMotor(cutMot);  
   }
   else 
   {
    setimp1=0;
    setimp2=0;
    PWM1=0;
    PWM2=0;
    pwmWrite(PB8, 0 );         // M1 PWM off
    pwmWrite(PB9, 0 );         // M2 PWM off
    CuttingMotor(false);
    digitalWrite(PA5, LOW);    // Cutting motor STOP  
    DelayStart=0;  
    DelayBuz=false;
   }  
}

void CuttingMotor(bool StartStop)   // Todo: motor current(load) monitoring
{
  if (StartStop == true)
  {
    DelayStart++;
    if (DelayStart<50)              // Enable buzzer before starting motor
    {
      DelayBuz=true;
    }
    if (DelayStart>=50)              // Buzzer off, start cutting motor
    {
      DelayBuz=false;
      //Serial2.println(DelayStart);
      digitalWrite(PA5, StartStop);   // Cutting motor Start/Stop 
    }
    if (DelayStart>=51) DelayStart=50;
  } 
  else
  {
    digitalWrite(PA5, LOW);    // Cutting motor STOP
    DelayStart=0;
    DelayBuz=false;
  }
   
}
//------------- Status LED -----------
// Green ON       - Working OK
// Grenn flashing - waiting for coms
// Red flashing   - fault or starting cutting motor
void LED() 
{ 
  if (ESTOP>=1)
    {
      digitalWrite(PA6, HIGH);  // Green LED OFF 
      setimp1=0;
      setimp2=0;
      if(t100ms2<4 ) digitalWrite(PA7, LOW);   // Red LED ON
      if(t100ms2>4 ) digitalWrite(PA7, HIGH);  // Red LED OFF   
    }
    else
      {
       digitalWrite(PA7, HIGH);  // Red LED OFF
      }

  if ((WdtStop>=1)&&(ESTOP==0))
    {
      if(t100ms2<5 ) digitalWrite(PA6, LOW);   // Green LED ON
      if(t100ms2>5 ) digitalWrite(PA6, HIGH);  // Green LED OFF
    }
    
  if ((WdtStop==0)&&(ESTOP==0)&&(DelayBuz==0)) 
    {  
     digitalWrite(PA6, LOW);    // Green LED ON 
    }

  if (DelayBuz==true)           // For starting cut. motor sound
    {
      digitalWrite(PA6, HIGH);  // Green LED OFF 
      if(t100ms2<4 ) digitalWrite(PA7, LOW);   // Red LED ON
      if(t100ms2>4 ) digitalWrite(PA7, HIGH);  // Red LED OFF
      
    }
    else
    {
      digitalWrite(PA7, HIGH);  // Red LED OFF
    }
    
}

//----------- Serial Comunication RX ---------
// 7 bit packet:  "S"; Lmotor F/R; Lmotor speed;  Rmotor F/R; Rmotor speed; Run/Stop; "E"    // todo + CRC
// test packet R  0X53 0x46 0x10 0x46 0x10 0x01 0x45 
//             F  0X53 0x52 0x10 0x52 0x10 0x00 0x45 
int inByte = 0;
bool packetStart =false;
bool packetStop  =false;
bool dataReady   =false;     // Full packet recived
unsigned char rxData[7];
unsigned char poz =0;        // packet index

void UartRX()
{ 
     if (Serial2.available() > 0) 
     {
        inByte = Serial2.read();            // read one byte from serial buffer

        if( (inByte=='S') || (poz !=0) )    // get packet start symbol
        {
          rxData[poz]=inByte; 
         // Serial2.println(inByte);
         // Serial2.write(inByte);
          poz++;  
         
          if ((poz==7)&& inByte!='E')
          {
           //Serial2.println("blogas paketas"); 
           poz=0;
           
          }
          if ((poz==7)&& inByte=='E')           // Full packet recived "SxxxxxE"
          {   
           poz=0; 
           if (rxData[1]=='F')  M1dirSet=true;   
           if (rxData[1]=='R')  M1dirSet=false;       
           if (rxData[1]=='S')  M1stop=true; 
           setimp1=rxData[2];                   // M1 set speed

           if (rxData[3]=='F')  M2dirSet=true;       
           if (rxData[3]=='R')  M2dirSet=false;       
           if (rxData[3]=='S')  M2stop=true; 
           setimp2=rxData[4];                   // M2 set speed

           if((rxData[5]&0x01)==true)  cutMot=true;   // Start cutting motor
           if((rxData[5]&0x01)==false) cutMot=false;  // Stop cutting motor

           if(rxWdt<=20 ) rxWdt=21;            // Reset serial WDT cnt
           
           /*
           Serial2.write(rxData[0]);
           Serial2.write(rxData[1]);
           Serial2.write(rxData[2]);
           Serial2.write(rxData[3]);
           Serial2.write(rxData[4]);
           Serial2.write(rxData[5]);
           Serial2.write(rxData[6]);
           */         
          } 
          
        }
      }    
}

//------------- Serial Comunication TX --------
void UartTX()
{
 char buf[17]; 
 memset(buf, 0, sizeof(buf));
 
 buf[0]='T';
 buf[1]=ESTOP;                               // Bendras Avarinis
 buf[1]=buf[1] | (Ebutton<<1);               // Avarinis mygtukas
 buf[1]=buf[1] | (Esensor<<2);               // Bamperis
 
 buf[2]=buf[2] | (BatU>>4);                  // H byte
 buf[3]=buf[3] | (BatU & 0x0f);              // L byte
 buf[4]=buf[4] | (BatI >> 4);                // H byte
 buf[5]=buf[5] | (BatI & 0x0f);              // L byte
 buf[6]=buf[6] | (dir1fused >> 4);           // H byte
 buf[7]=buf[7] | (dir1fused & 0x0f);         // L1 byte

 buf[8]=buf[8] | (Encoder1cnt >>24) & 0xff;  // M1 encoder cnt
 buf[9]=buf[9] | (Encoder1cnt >>16) & 0xff;
 buf[10]=buf[10] | (Encoder1cnt >>8) & 0xff;
 buf[11]=buf[11] | (Encoder1cnt & 0xff); 

 buf[12]=buf[12] | (Encoder2cnt >>24) & 0xff; // M2 encoder cnt
 buf[13]=buf[13] | (Encoder2cnt >>16) & 0xff;
 buf[14]=buf[14] | (Encoder2cnt >>8) & 0xff;
 buf[15]=buf[15] | (Encoder2cnt & 0xff); 

 buf[16]='E';
 //buf[9]=0x00;
  for(int i=0; i<17; i++)
  {
    Serial2.print(buf[i]);
  }
  //Serial2.println();
 memset(buf, 0, sizeof(buf));
 
 // snprintf(report, sizeof(report), "Acceleration:(X:%6d Y:%6d Z:%6d) Magnetometer:(X:%6d Y:%6d Z:%6d)",
 // compass.accelerometer.x, compass.accelerometer.y, compass.accelerometer.z,
 // compass.magnetometer.x, compass.magnetometer.y, compass.magnetometer.z); 
}
//------------------ Magnetometer + IMU Sensor data fussion -----------------------------------------------
float yawimu=0;
float yawimu1=0;
float yawimuold=0;
float yaw2old=0;
float delta3;
float dif1=0;
float yaw2=0;
float yawdif=0;
int   steady=0;

void Fusion()
{
   yawimu1=yawimu;       //+180;  // MPU6050 (mpu -180+180)   
   if (steady <1000)     // Wait 3s, 
    {
      yawdif=dir1;
      yaw2=dir1;
      yawimuold=yawimu;
    }
        
   if (steady >=1000)
    {
       yawdif=(yawimu1-yawimuold); 
       yaw2=(0.995*(yawdif+yaw2old))+(0.005*dir1);   // Fuse IMU and MAG angle  0.995

       if(abs(yaw2-dir1)>100)                        // for 0-360 rapid angle change
        {
          yaw2old=dir1;
          yawimuold=yawimu1;  
        }
       else 
        {
          yaw2old=yaw2;
          yawimuold=yawimu1; 
        }

       dir1fused=int(yaw2);
       //Serial2.print(dir1);
       //Serial2.print(',');
       //Serial2.print(yawimu1);
       //Serial2.print(',');
       //Serial2.println(yaw2);   
       //Serial2.println(CpuTime());
    }

  steady ++;
  if (steady >1000) steady =1000;
     
}
// ----------------------- IMU (DMP mode ) --------------
void MPU6050run(){
// if programming failed, don't try to do anything
   if (!dmpReady) return;
   if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
       #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //Serial2.print("ypr\t");
            //Serial2.println(ypr[0] * 180/M_PI);
             yawimu=ypr[0] * 180/M_PI;
            //Serial2.print("\t");
           // Serial2.print(",");
           // Serial2.print(ypr[1] * 180/M_PI);
            //Serial2.print("\t");
           // Serial2.print(",");
           // Serial2.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            //Serial2.print("areal\t");
            Serial2.print(aaReal.x);
            //Serial2.print("\t");
            Serial2.print(",");
            Serial2.print(aaReal.y);
            //Serial2.print("\t");
            Serial2.print(",");
            Serial2.println(aaReal.z);
        #endif
   }  
    
}
//----------------------------
int t3=0;
int t4=0;
int t5=0;
int CpuTime()   // for debug.
{
  t4=micros();
  t5=t4-t3;
  t3=t4;
  return t5;
}
//-----------------------------------------
//---------------- Main loop --------------
//-----------------------------------------
int cnt5=0;
void loop()   // Loop time ~ 153-1732 us
{
  Avariniai();
  LED();
  UartRX();
  
//CompassCal();   // only for mag calibration
  if (millis() - interval2 >= 200)
  { 
   AnalogIn();
   UartTX();               // Trasnsmit telemetry data 
   interval2 = millis();  
  }
  
  if (millis() - interval >= 3)     // update @~3ms    MAG-7.5 Hz (133ms ); ACC-400 hz (2.5 ms)
  { 
    //CpuTime();   
     Compass1();
     MPU6050run();
     Fusion();
     interval = millis();                     // update interval 
     digitalWrite(PC13, !digitalRead(PC13));  // STM Board Led blink
  }
  
}

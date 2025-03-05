//
#include <DFRobot_LSM303.h>
#include <Wire.h>

TwoWire Wire2(2,I2C_FAST_MODE );   
DFRobot_LSM303 compass(&Wire2);

char report[120];

void setup()
{
  pinMode(PB6, OUTPUT);          // M1 Dir  ( L motor )  to stop motors
  digitalWrite(PB6, HIGH); 
  pinMode(PB7, OUTPUT);          // M2 Dir  ( R motor )
  digitalWrite(PB7, HIGH); 

  pinMode(PB8, OUTPUT);             // M1 PWM  timer4
  pinMode(PB9, OUTPUT);             // M2 PWM  timer4

  Serial2.begin(115200);
  while(!Serial2) ;
  compass.init(DFRobot_LSM303::device_DLHC, DFRobot_LSM303::sa0_high);
  delay(200);
  compass.enable();
}

float xv, yv, zv;
void loop()
{
  int ax, ay, az;
  int gx, gy, gz;
  int mx, my, mz;

   compass.read();
   mx= compass.magnetometer.x;
   my= compass.magnetometer.y;
   mz= compass.magnetometer.z;

  // --- Send data to PJRC Motioncal software ---
    Serial2.print("Raw:");
    Serial2.print(ax);
    Serial2.print(',');
    Serial2.print(ay);
    Serial2.print(',');
    Serial2.print(az);
    Serial2.print(',');
    Serial2.print(gx);
    Serial2.print(',');
    Serial2.print(gy);
    Serial2.print(',');
    Serial2.print(gz);
    Serial2.print(',');
    Serial2.print(mx);
    Serial2.print(',');
    Serial2.print(my);
    Serial2.print(',');
    Serial2.print(mz);
    Serial2.println();
    delay(100); 
 
}

/*
    Arduino Brushless Motors Control with custom MPU Data first try
*/

#include <Servo.h>                         //Include the Servo.h librar so we can command the ESCs
#include <Wire.h>                          //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM


#define A 0.962
#define dt 0.020

int calx,caly;

double accel_x_cal,accel_y_cal,accel_z_cal;
double accelX, accelY, accelZ;

float rollangle,pitchangle;
float roll,pitch,yaw;

double gyroX, gyroY, gyroZ;
double gyro_x_cal,gyro_y_cal,gyro_z_cal;


Servo ESC1;  // Create Servo Object to control the ESC1
Servo ESC2;  // Create Servo Object to control the ESC2
Servo ESC3;  // Create Servo Object to control the ESC3
Servo ESC4;  // Create Servo Object to control the ESC4

int i = 0;
int sig = 0;

void setup() {
  // put your setup code here, to run once:
  // Attach the ESC on pin 4,5,6,7
  ESC1.attach(4,1000,2000); // (pin, min pulse width, max pulse width in microsec)
  ESC2.attach(5,1000,2000); // (pin, min pulse width, max pulse width in microsec)
  ESC3.attach(6,1000,2000); // (pin, min pulse width, max pulse width in microsec)
  ESC4.attach(7,1000,2000); // (pin, min pulse width, max pulse width in microsec)


  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
 
  Serial.println("Calibrating Accelerometer......");
  for(calx=1;calx<=2000;calx++)
  {
     recordAccelRegisters();
     accel_x_cal += accelX;                      
     accel_y_cal += accelY;      
     accel_z_cal += accelZ;
  }
  Serial.println("Calibrating Gyroscope......");
  for(caly=1;caly<=2000;caly++)
  {
    recordGyroRegisters();
    gyro_x_cal += gyroX;                      
    gyro_y_cal += gyroY;      
    gyro_z_cal += gyroZ;
  }
  Serial.println("Calibration Done..!!!");
  gyro_x_cal /= 2000;                             
  gyro_y_cal /= 2000;                            
  gyro_z_cal /= 2000;      
  
}

void loop() {
  // put your main code here, to run repeatedly:
  recordAccelRegisters();
  recordGyroRegisters();

  accelX = accelX / 16384.0;
  accelY = accelY / 16384.0; 
  accelZ = accelZ / 16384.0;

  gyroX = gyroX / 131.0;
  gyroY = gyroY / 131.0; 
  gyroZ = gyroZ / 131.0;

  rollangle=atan2(accelY,accelZ)*180/PI; // FORMULA FOUND ON INTERNET
  pitchangle=atan2(accelX,sqrt(accelY*accelY+accelZ*accelZ))*180/PI; //FORMULA FOUND ON INTERNET
  
  //Using Complemetary Filter
  roll=A*(roll+gyroX*dt)+(1-A)*rollangle;
  pitch=A*(pitch+gyroY*dt)+(1-A)*pitchangle;
  
  yaw=gyroZ;

//  Serial.print(" ROLL=");
//  Serial.print(roll);
//  Serial.print(" angle\n");
//  Serial.print("    PITCH=");
//  Serial.print(pitch);
//  Serial.print(" angle");
  Serial.print("    YAW=");
  Serial.print(yaw);
  Serial.println(" deg/s");
  
  if(!i)
  {
    ESC1.write(0); // Send the signal to ESC (send something between 0,180)
    ESC2.write(0); // Send the signal to ESC (send something between 0,180)
    ESC3.write(0); // Send the signal to ESC (send something between 0,180)
    ESC4.write(0); // Send the signal to ESC (send something between 0,180)
    delay(1000);
    i = 1;
  }
//  ESC1.write(80);
//  ESC2.write(80);
//  ESC3.write(80);
//  ESC4.write(80);


  if (yaw > 0  && yaw < 70)
  {
    ESC1.write(120);
    ESC2.write(120);
    ESC3.write(120);
    ESC4.write(120);
  }
  else if (yaw < 0)
  {
    ESC1.write(60);
    ESC2.write(60);
    ESC3.write(60);
    ESC4.write(60);
  }
  else
  {
    ESC1.write(0);
    ESC2.write(0);
    ESC3.write(0);
    ESC4.write(0);            
  }
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); 
  if(calx == 2000)accelX -= accel_x_cal; 
  accelY = Wire.read()<<8|Wire.read(); 
  if(calx == 2000)accelY -= accel_y_cal; 
  accelZ = Wire.read()<<8|Wire.read(); 
  if(calx == 2000)accelZ -= accel_z_cal; 
  
}


void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read();
  if(caly == 2000)gyroX -= gyro_x_cal; 
  gyroY = Wire.read()<<8|Wire.read(); 
  if(caly == 2000)gyroY -= gyro_y_cal; 
  gyroZ = Wire.read()<<8|Wire.read();
  if(caly == 2000)gyroZ -= gyro_z_cal; 
  
}

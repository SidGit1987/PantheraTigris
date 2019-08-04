#include <Wire.h>

#define A 0.962
#define dt 0.020

int calx,caly;

double accel_x_cal,accel_y_cal,accel_z_cal;
double accelX, accelY, accelZ;

float rollangle,pitchangle;
float roll,pitch,yaw;

double gyroX, gyroY, gyroZ;
double gyro_x_cal,gyro_y_cal,gyro_z_cal;


void setup() {
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

  Serial.print(" ROLL=");
  Serial.print(roll);
  Serial.print(" angle");
  Serial.print("    PITCH=");
  Serial.print(pitch);
  Serial.print(" angle");
  Serial.print("    YAW=");
  Serial.print(yaw);
  Serial.println(" deg/s");
  
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

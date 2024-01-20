#include<Wire.h>  // I2C Libary
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>


//Constants
// MPU-6050
const int MPU_6050_Addr=0x68; // I2C Address of the MPU-6050
const byte PWR_MGMT_1 = 0x6B; // PWR_MGMT_1 register address of the MPU-6050
const byte ACCEL_Config = 0x1C; //ACCEL_Config Register address of the MPU-6050
const byte ACCEL_XOUT_H = 0x3B; //ACCEL_XOUT_H Register address of the MPU-6050
const byte GYRO_CONFIG = 0x1B;  // GYRO_CONFIG register address of MPU-6050

// Variables
// MPU-6050
float MPU_6050_Temp;
float MPU_6050_ACCX;
float MPU_6050_ACCY;
float MPU_6050_ACCZ;
float MPU_6050_GyroX;
float MPU_6050_GyroY;
float MPU_6050_GyroZ;

// BMP280
Adafruit_BMP280 bme;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); //set to Zero (wakes up the MPU-6050)
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(ACCEL_Config);
  Wire.write(0x10);  //setting the full scale range to plus minus 8g
  Wire.endTransmission();
  Wire.write(GYRO_CONFIG);
  Wire.write(0x08);  //setting the full scale range to plus minus 8g
  Wire.endTransmission();

  bme.begin();
  Serial.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(ACCEL_XOUT_H);  //ACCEL_XOUT_H register
  Wire.endTransmission();
  Wire.requestFrom(MPU_6050_Addr, 14);  // request 2 bytes from the MPU-6050
  MPU_6050_ACCX = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_ACCY = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_ACCZ = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_Temp = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_GyroX = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_GyroY = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_GyroZ = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_ACCX = MPU_6050_ACCX/4096;
  MPU_6050_ACCY = MPU_6050_ACCY/4096;
  MPU_6050_ACCZ = MPU_6050_ACCZ/4096;
  MPU_6050_Temp = MPU_6050_Temp/ 340 + 36.53;
  MPU_6050_GyroX = MPU_6050_GyroX/65.5;
  MPU_6050_GyroY = MPU_6050_GyroY/65.5;
  MPU_6050_GyroZ = MPU_6050_GyroY/65.5;
  Serial.print("MPU-6050 Acc X- Axis  - "); Serial.println(MPU_6050_ACCX); Serial.println(" [g]");
  Serial.print("MPU-6050 Acc Y- Axis  - "); Serial.println(MPU_6050_ACCY); Serial.println(" [g]");
  Serial.print("MPU-6050 Acc Z- Axis  - "); Serial.println(MPU_6050_ACCZ); Serial.println(" [g]");
  Serial.print("MPU-6050 Gyro X- Axis - "); Serial.println(MPU_6050_GyroX); Serial.println(" [deg/s]");
  Serial.print("MPU-6050 Gyro Y- Axis - "); Serial.println(MPU_6050_GyroY); Serial.println(" [deg/s]");
  Serial.print("MPU-6050 Gyro Z- Axis - "); Serial.println(MPU_6050_GyroZ); Serial.println(" [deg/s]");
  Serial.print("MPU-6050 Temp         - "); Serial.println(MPU_6050_Temp); Serial.println(" [degrees C]");
  Serial.print("---------");
  Serial.print("BMP280 Temperature    - "); Serial.println(bme.readTemperature()); Serial.println(" [degrees C]");
  Serial.print("BMP280 Preasure       - "); Serial.println(bme.readPressure()/100); Serial.println(" [hPa]");
  Serial.print("BMP280 Aprox. Alt.    - "); Serial.println(bme.readAltitude(1013.25)); Serial.println(" []");
  delay(10000); 
}




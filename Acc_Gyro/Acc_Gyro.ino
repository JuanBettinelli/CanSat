#include<Wire.h>  // I2C Libary

const int MPU_6050_Addr=0x68; // I2C Address of the MPU-6050
const byte PWR_MGMT_1 = 0x6B; // PWR_MGMT_1 register address of the MPU-6050
const byte ACCEL_Config = 0x1C; //ACCEL_Config Register address of the MPU-6050
const byte ACCEL_XOUT_H = 0x3B; //ACCEL_XOUT_H Register address of the MPU-6050
const byte GYRO_CONFIG = 0x1B;  // GYRO_CONFIG register address of MPU-6050

byte TWMP_Out_H_data;
byte TEM_OUT_L_data;

float MPU_6050_Temp;
float MPU_6050_AccX;
float MPU_6050_AccY;
float MPU_6050_AccZ;
float MPU_6050_GyroX;
float MPU_6050_GyroY;
float MPU_6050_GyroZ;


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); //set to Zero (wakes up the MPU-6050)
  Wire.endTransmission();
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(ACCEL_Config);
  Wire.write(0x10);  //setting the full scale range to plus minus 8g
  Wire.endTransmission();
  Wire.write(GYRO_CONFIG);
  Wire.write(0x08);  //setting the full scale range to plus minus 8g
  Wire.endTransmission();
  Serial.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(ACCEL_XOUT_H);  //ACCEL_XOUT_H register
  Wire.endTransmission();
  Wire.requestFrom(MPU_6050_Addr, 14);  // request 2 bytes from the MPU-6050
  MPU_6050_AccX = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_AccY = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_AccZ = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_Temp = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_GyroX = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_GyroY = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_GyroZ = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_AccX = MPU_6050_AccX/4096;
  MPU_6050_AccY = MPU_6050_AccY/4096;
  MPU_6050_AccZ = MPU_6050_AccZ/4096;
  MPU_6050_Temp = MPU_6050_Temp/ 340 + 36.53;
  MPU_6050_GyroX = MPU_6050_GyroX/65.5;
  MPU_6050_GyroY = MPU_6050_GyroY/65.5;
  MPU_6050_GyroZ = MPU_6050_GyroY/65.5;
  Serial.print("MPU-6050 Acc X- Axis - "); Serial.println(MPU_6050_AccX); Serial.println(" [g]");
  Serial.print("MPU-6050 Acc Y- Axis - "); Serial.println(MPU_6050_AccY); Serial.println(" [g]");
  Serial.print("MPU-6050 Acc Z- Axis - "); Serial.println(MPU_6050_AccZ); Serial.println(" [g]");
  Serial.print("MPU-6050 Gyro X- Axis - "); Serial.println(MPU_6050_GyroX); Serial.println(" [deg/s]");
  Serial.print("MPU-6050 Gyro Y- Axis - "); Serial.println(MPU_6050_GyroY); Serial.println(" [deg/s]");
  Serial.print("MPU-6050 Gyro Z- Axis - "); Serial.println(MPU_6050_GyroZ); Serial.println(" [deg/s]");
  Serial.print("MPU-6050 Temp - "); Serial.println(MPU_6050_Temp); Serial.println(" [degrees C]");
  Serial.print("---------");
  delay(1000); 
}




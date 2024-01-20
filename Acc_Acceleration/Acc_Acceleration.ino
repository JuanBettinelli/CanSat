#include<Wire.h>  // I2C Libary

const int MPU_6050_Addr=0x68; // I2C Address of the MPU-6050
const byte PWR_MGMT_1 = 0x6B; // PWR_MGMT_1 register address of the MPU-6050
const byte ACCEL_Config = 0x1C; //ACCEL_Config Register address of the MPU-6050
const byte ACCEL_XOUT_H = 0x3B; //ACCEL_XOUT_H Register address of the MPU-6050
const byte TEM_OUT_H = 0x41;  // TEMP_OUT_H register address of MPU-6050

byte TWMP_Out_H_data;
byte TEM_OUT_L_data;
float MPU_6050_Temp;
float MPU_6050_ACCX;
float MPU_6050_ACCY;
float MPU_6050_ACCZ;


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
  Serial.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(ACCEL_XOUT_H);  //ACCEL_XOUT_H register
  Wire.endTransmission();
  Wire.requestFrom(MPU_6050_Addr, 8);  // request 2 bytes from the MPU-6050
  MPU_6050_ACCX = Wire.read() << 8 | Wire.read() ;
  MPU_6050_ACCY = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_ACCZ = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_Temp = Wire.read()  << 8 | Wire.read() ;
  MPU_6050_ACCX = MPU_6050_ACCX/4096;
  MPU_6050_ACCY = MPU_6050_ACCY/4096;
  MPU_6050_ACCZ = MPU_6050_ACCZ/4096;
  MPU_6050_Temp = MPU_6050_Temp/ 340 + 36.53;
  Serial.print("MPU-6050 Acc X- Axis - "); Serial.println(MPU_6050_ACCX); Serial.println(" [g]");
  Serial.print("MPU-6050 Acc Y- Axis - "); Serial.println(MPU_6050_ACCY); Serial.println(" [g]");
  Serial.print("MPU-6050 Acc Z- Axis - "); Serial.println(MPU_6050_ACCZ); Serial.println(" [g]");
  Serial.print("MPU-6050 Temp - "); Serial.println(MPU_6050_Temp); Serial.println(" [degrees C]");
  Serial.print("---------");
  delay(1000); 
}




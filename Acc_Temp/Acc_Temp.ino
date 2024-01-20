#include<Wire.h>  // I2C Libary

const int MPU_6050_Addr=0x68; // I2C Address of the MPU-6050
const byte PWR_MGMT_1 = 0x6B; //PWR_MGMT_1 Register address of the MPU-6050
const byte TEM_OUT_H = 0x41;  // TEMP_OUT_H register address of MPU-6050

byte TWMP_Out_H_data;
byte TEM_OUT_L_data;
float MPU_6050_Temp;


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(PWR_MGMT_1);
  Wire.write(0); //set to Zero (wakes up the MPU-6050)
  Wire.endTransmission();
  Serial.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(TEM_OUT_H);  //TEMP_OUT_H register
  Wire.endTransmission();
  Wire.requestFrom(MPU_6050_Addr, 2);  // request 2 bytes from the MPU-6050
  TWMP_Out_H_data = Wire.read();
  TEM_OUT_L_data = Wire.read();
  MPU_6050_Temp = TWMP_Out_H_data << 8 | TEM_OUT_L_data;
  MPU_6050_Temp = MPU_6050_Temp/ 340 + 36.53;
  Serial.print("MPU-6050 Temp - "); Serial.println(MPU_6050_Temp); Serial.println(" [degrees C]");
  delay(1000); 
}




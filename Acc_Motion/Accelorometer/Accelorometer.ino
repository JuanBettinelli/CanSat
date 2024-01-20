#include<Wire.h>

const int MPU_6050_Addr=0x68; // I2C Address of the MPU-6050
const byte PWR_MGMT_1 = 0x6B; // PWR_MGMT_1 register address of the MPU-6050
const byte WHO_AM_I = 0x75; //WHO_AM_I register address of the MPU-6050

byte WHO_AM_I_data;



void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  //set to Zero (wakes up the MPU-6050)
  Wire.endTransmission();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(WHO_AM_I);
  Wire.requestFrom(MPU_6050_Addr, 1);

  WHO_AM_I_data = Wire.read();  //requesting 1 byte from the MPU-6050
  Serial.print("Who am I? - "); Serial.println(WHO_AM_I_data);
  delay(100);
}

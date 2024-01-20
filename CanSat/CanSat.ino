#include<Wire.h>  // I2C Libary
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>

float Start_Altedude = 38.00;
double AltDelta;
double Relative_Start_Altedude;

//Mission Parameteres
boolean Landed;
boolean Lanched;


// Servo Motor
Servo myservo;
boolean Latch_Closed;

//DC Motors
int LeftDC = 2;
int RightDC = 3;

//Temp sensor
const int analogInPin = A0; // analog input pin
float tempSensorValue = 0; // Temperature sensor data

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



void setup(){
  // Test Switch
  pinMode(12, INPUT);

  // DC Motor
  pinMode(LeftDC, OUTPUT);
  pinMode(RightDC, OUTPUT);
  // Switch DC Motors off
  digitalWrite(LeftDC, LOW);
  digitalWrite(RightDC, LOW);
  
  //Close Servo
  myservo.attach(4); 
  myservo.write(10);
  Latch_Closed = true;
  Serial.println("Parachute cache Closed");

  //Initial mission Status
  Landed = false;
  Lanched = false;

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

  Serial.println("Timestamp_ MPU-6050_Acc_X; MPU-6050_Acc_Y; MPU-6050_Acc_Z; MPU-6050_Gyro_X; MPU-6050_Gyro_Y; MPU-6050_Gyro_Z; MPU-6050_Temp; BMP280_Temp; BMP280_Pres; BMP280_Alt; LM35DZ_Temp; Mission_Notes");
  Serial.print(millis()); Serial.println(";;;;;;;;;;;; Start of CanSat mission!");
  AltDelta = PreStartCalibration(Start_Altedude);
  Relative_Start_Altedude = PreStartRelativeAlt();
}

//------------------------------------------------------------------------------------------------------

void loop() {
  Measument();
  if (Lanched == true){
      if (Landed == false){
      Landed = LandedTest(AltDelta);
    }
    else if (Landed == true){
      RoverMode(); 
      delay(500);
    }
  }
  else if (Lanched == false){
    LaunchCheck();
  }

  delay(500);


}

//------------------------------------------------------------------------------------------------------
int RoverMode(){
  if (Latch_Closed == true){
    myservo.write(170);
    delay(500);
    Latch_Closed = false;
    Serial.print(millis()); Serial.println(";;;;;;;;;;;; Parachute was deattached");
  }
  else {
    myservo.write(10);
    delay(500);
    myservo.write(170);
    delay(500);
    Latch_Closed = false;
    Serial.print(millis()); Serial.println(";;;;;;;;;;;; Parachute was still attached, deattached now");
  }

  for (int i = 0; i<= 1; i++){
    Serial.print(millis()); Serial.println(";;;;;;;;;;;; Ground Mission Started: Rover Driving");
    digitalWrite(LeftDC, HIGH);
    digitalWrite(RightDC, HIGH);
    delay(5000);
    
    digitalWrite(LeftDC, LOW);
    digitalWrite(RightDC, LOW);
    delay(1000);
    Measument();

    digitalWrite(LeftDC, HIGH);
    digitalWrite(RightDC, LOW);
    delay(500);

    digitalWrite(LeftDC, HIGH);
    digitalWrite(RightDC, HIGH);
    delay(5000);
    
    digitalWrite(LeftDC, LOW);
    digitalWrite(RightDC, LOW);
    delay(1000);
    Measument();

    digitalWrite(LeftDC, LOW);
    digitalWrite(RightDC, HIGH);
    delay(500);

    digitalWrite(LeftDC, HIGH);
    digitalWrite(RightDC, HIGH);
    delay(5000);


    digitalWrite(LeftDC, LOW);
    digitalWrite(RightDC, LOW);
    delay(1000);
    Measument();

    Serial.print(millis()); Serial.println(";;;;;;;;;;;; Ground Mission Finisched: Rover Stoped");
  }
}


//------------------------------------------------------------------------------------------------------
int LandedTest(double AltDelta){
  
  boolean Quick_Test = false;
  double PreasureList[10];
  double sum = 0;
  double mean;
  double QIt = 10;
  double Qsteps = 100;

  Serial.print(millis()); Serial.println(";;;;;;;;;;;; Quick altitude check");

  for (int j = 0; j <= (QIt - 1); j++){
    PreasureList[j] = bme.readAltitude(1013.25);
    sum += PreasureList[j];
    delay(100);
  }
  mean = sum / QIt;

  mean = mean + AltDelta;

  int minValue = PreasureList[0];
  int maxValue = PreasureList[0];

  // Find the minimum and maximum values
  for (int i = 1; i < (QIt - 1); ++i) {
    if (PreasureList[i] < minValue) {
      minValue = PreasureList[i];
    } else if (PreasureList[i] > maxValue) {
      maxValue = PreasureList[i];
    }
  }
  double Speed = (maxValue - minValue)/(QIt*(Qsteps/1000));

  if ((mean > 40) || (Speed > 0.5)) {
    Serial.print(millis()); Serial.print(";;;;;;;;;;;; Flighing: Altitud: "); Serial.print(mean); Serial.print(", Speed - "); Serial.println(Speed);
    Quick_Test = false;
  }
  else if ((mean <= 40) && (Speed <= 0.5)) {
    Serial.print(millis()); Serial.print(";;;;;;;;;;;; Quick test past: Altitud: "); Serial.print(mean); Serial.print(", Speed - "); Serial.println(Speed);
    Quick_Test = true;
  }


  if (Quick_Test == true){
    double S_PreasureList[10];
    double S_sum = 0;

    Serial.print(millis()); Serial.println(";;;;;;;;;;;; Perform long safty test (1 min) measument");
    for (int n = 0; n <= 9; n++){
      S_PreasureList[n] = bme.readAltitude(1013.25);
      S_sum += S_PreasureList[n];
      delay(6000);
    }
    double S_mean = S_sum / 10;
    S_mean = S_mean + AltDelta;

    double S_minValue = S_PreasureList[0];
    double S_maxValue = S_PreasureList[0];
    // Find the minimum and maximum values
    for (int m = 1; m < 9; ++m) {
      if (S_PreasureList[m] < S_minValue) {
        S_minValue = S_PreasureList[m];
      } else if (S_PreasureList[m] > maxValue) {
        S_maxValue = S_PreasureList[m];
      }
    }
    double S_Speed = (S_maxValue - S_minValue)/60;
  
    if ((S_mean > 40) && (S_Speed > 0.5)) {
      Serial.print(millis()); Serial.println(";;;;;;;;;;;; Safety test Failed, CanSat Flighing!");
      Landed = false;
    }
    else if ((S_mean <= 40) || (S_Speed <= 0.5)){
      Landed = true;
      Serial.print(millis()); Serial.println(";;;;;;;;;;;; Safety test passed, CanSat Landed!");
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    else{
      Serial.print(millis()); Serial.println(";;;;;;;;;;;; Error with Landing Check");
      Landed = false;
    }
  }
  
  return Landed;
}
//------------------------------------------------------------------------------------------------------
int Measument(){
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

  tempSensorValue = analogRead(analogInPin);
  // convert temperature data
  tempSensorValue = tempSensorValue * 5000/1023 * 1/10;
  //Transmit temperature sensor data, 1 decimal place
  //Serial.println("Timestamp_ MPU-6050_Acc_X; MPU-6050_Acc_Y; MPU-6050_Acc_Z; MPU-6050_Gyro_X; MPU-6050_Gyro_Y; MPU-6050_Gyro_Z; MPU-6050_Temp; BMP280_Temp; BMP280_Pres; BMP280_Alt; LM35DZ_Temp; Mission_Notes");
  Serial.print(millis()); Serial.print("; "); Serial.print(MPU_6050_ACCX); Serial.print("; "); Serial.print(MPU_6050_ACCY); Serial.print("; "); Serial.print(MPU_6050_ACCZ); Serial.print("; "); Serial.print(MPU_6050_GyroX); Serial.print("; "); Serial.print(MPU_6050_GyroY); Serial.print("; "); Serial.print(MPU_6050_GyroZ);
  Serial.print("; "); Serial.print(MPU_6050_Temp); Serial.print("; "); Serial.print(bme.readTemperature()); Serial.print("; "); Serial.print(bme.readPressure()/100); Serial.print("; "); Serial.print(bme.readAltitude(1013.25)); Serial.print("; "); Serial.print(tempSensorValue,1); Serial.println("; ");
  
  //Serial.print("MPU-6050 Acc X- Axis  - "); Serial.print(MPU_6050_ACCX); Serial.println(" [g]");
  //Serial.print("MPU-6050 Acc Y- Axis  - "); Serial.print(MPU_6050_ACCY); Serial.println(" [g]");
  //Serial.print("MPU-6050 Acc Z- Axis  - "); Serial.print(MPU_6050_ACCZ); Serial.println(" [g]");
  //Serial.print("MPU-6050 Gyro X- Axis - "); Serial.print(MPU_6050_GyroX); Serial.println(" [deg/s]");
  //Serial.print("MPU-6050 Gyro Y- Axis - "); Serial.print(MPU_6050_GyroY); Serial.println(" [deg/s]");
  //Serial.print("MPU-6050 Gyro Z- Axis - "); Serial.print(MPU_6050_GyroZ); Serial.println(" [deg/s]");
  //Serial.print("MPU-6050 Temp         - "); Serial.print(MPU_6050_Temp); Serial.println(" [degrees C]");
  //Serial.print("BMP280 Temperature    - "); Serial.print(bme.readTemperature()); Serial.println(" [degrees C]");
  //Serial.print("BMP280 Preasure       - "); Serial.print(bme.readPressure()/100); Serial.println(" [hPa]");
  //Serial.print("BMP280 Aprox. Alt.    - "); Serial.print(bme.readAltitude(1013.25)); Serial.println(" [m]");
  //Serial.print("External Temp.        - "); Serial.print(tempSensorValue,1); Serial.println(" [degrees C]");
}

//------------------------------------------------------------------------------------------------------

double PreStartCalibration (double StartAlt) {

  Serial.print(millis()); Serial.println(";;;;;;;;;;;; Preform prestart calibration");
  double PreStartData[2];
  double AltList[10];
  double AltSum = 0;
  for (int j = 0; j <= 9; j++){
    AltList[j] = bme.readAltitude(1013.25);
    AltSum += AltList[j];
    delay(1000);
  }
  double AltMean = AltSum / 10;
  double DeltaAltetude = StartAlt - AltMean;


  Serial.print(millis()); Serial.print(";;;;;;;;;;;; Mean Altitude: "); Serial.print(AltMean); Serial.println(" [m]");
  Serial.print(millis()); Serial.print(";;;;;;;;;;;; Altitude delta: "); Serial.print(DeltaAltetude); Serial.println(" [m]");
  return DeltaAltetude;
}

//------------------------------------------------------------------------------------------------------

double PreStartRelativeAlt () {
    double AltSum = 0;

  Serial.print(millis()); Serial.println(";;;;;;;;;;;; Preform prestart relative altitude measument");
  double PreStartData[2];
  double AltList[10];
  for (int j = 0; j <= 9; j++){
    AltList[j] = bme.readAltitude(1013.25);
    AltSum += AltList[j];
    delay(1000);
  }
  double AltMean = AltSum / 10;

  return AltMean;
}

//------------------------------------------------------------------------------------------------------

double LaunchCheck() {

  double AltDiff = bme.readAltitude(1013.25) - Relative_Start_Altedude;
  
  if ((AltDiff > 10) || (digitalRead(12) == HIGH)){ //############## Need to remove Switch test condition
    Lanched = true;
    Serial.print(millis()); Serial.println(";;;;;;;;;;;; Rocked Lanched!!!");
  }
  else {
    Lanched = false;
  }

  return Lanched;
}

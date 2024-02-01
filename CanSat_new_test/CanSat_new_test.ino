
#include <Wire.h>  // I2C Libary
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
//#include <Servo.h>
// RX - Dout(blue) TX - Din(black), wire between Xbee and board should be blue to black
float Start_Altedude = 38.00;
double AltDelta;
double Relative_Start_Altedude;

//Mission Parameteres
boolean Landed;
boolean Lanched;

// Servo Motor
//Servo myservo;
//boolean Latch_Closed;

//Stepper motor
int stepCounter;
int steps = 1200;

//DC Motors
int DC_Motor = 2;

//Push button
int Button = 10;

//Temp sensor
const int analogInPin = A1;  // analog input pin
float tempSensorValue = 0;   // Temperature sensor data

//Constants
// MPU-6050
const int MPU_6050_Addr = 0x68;  // I2C Address of the MPU-6050
const byte PWR_MGMT_1 = 0x6B;    // PWR_MGMT_1 register address of the MPU-6050
const byte ACCEL_Config = 0x1C;  //ACCEL_Config Register address of the MPU-6050
const byte ACCEL_XOUT_H = 0x3B;  //ACCEL_XOUT_H Register address of the MPU-6050
const byte GYRO_CONFIG = 0x1B;   // GYRO_CONFIG register address of MPU-6050
const byte TEM_OUT_H = 0x41;  // TEMP_OUT_H register address of MPU-6050

// Variables
// MPU-6050
float MPU_6050_Temp;
float MPU_6050_ACCX;
float MPU_6050_ACCY;
float MPU_6050_ACCZ;
float MPU_6050_GyroX;
float MPU_6050_GyroY;
float MPU_6050_GyroZ;
byte TWMP_Out_H_data;
byte TEM_OUT_L_data;

// BMP280
Adafruit_BMP280 bme;


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Intital routine

void setup() {
  
  // Test Switch
  //pinMode(12, INPUT);

  // DC Motor
  pinMode(DC_Motor, OUTPUT);

  // Switch DC Motors off
  digitalWrite(DC_Motor, LOW);

  //Puch Button
  pinMode(10, INPUT);

  //Initilasing stepper motor
  pinMode(12, OUTPUT); // Enable
  pinMode(6, OUTPUT); // Step
  pinMode(5, OUTPUT); // Direction
  digitalWrite(6, LOW);

  //Initial mission Status
  Landed = false;
  Lanched = false;

  Wire.begin();
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  //set to Zero (wakes up the MPU-6050)
  Wire.endTransmission();
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(ACCEL_Config);
  Wire.write(0x10);  //setting the full scale range to plus minus 8g
  Wire.endTransmission();
  Wire.write(GYRO_CONFIG);
  Wire.write(0x08);  //setting the full scale range to plus minus 8g
  Wire.endTransmission();

  // Set up BMP280
  bme.begin();

  // Start Serial Port comunication
  Serial.begin(9600);

  //Print Table Header of output comunication
  Serial.println("Timestamp; MPU-6050_Acc_X; MPU-6050_Acc_Y; MPU-6050_Acc_Z; MPU-6050_Gyro_X; MPU-6050_Gyro_Y; MPU-6050_Gyro_Z; MPU-6050_Temp; BMP280_Temp; BMP280_Pres; BMP280_Alt; LM35DZ_Temp; Mission_Notes");
  Serial.print(millis()); Serial.println(";;;;;;;;;;;; Start of CanSat mission!");

  //Close Parachute latch
  CloseLatch();
  //myservo.attach(5);
  //myservo.write(6);
  //Latch_Closed = true;
  //Serial.println("Parachute latche Closed");


  // Execute "PreStartCalibrations" Function
  AltDelta = PreStartCalibration(Start_Altedude);
  // Execute "PreStartCalibrations" Function
  Relative_Start_Altedude = PreStartRelativeAlt();
  
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Run main routine
void loop() {

  Measument();                           // Preforme Measument

  if (Lanched == true) {                 // Check if CanSat was Lanched
    if (Landed == false) {               // Check If CanSat has Landed
      Landed = LandedTest(AltDelta);     // If not rund landet test
    } 
    else if (Landed == true) {           // If Landet switch to rover mode
      RoverMode();
      delay(500);
    }
  } 
  else if (Lanched == false) {           // if not landed check if Cansat has landet
    LaunchCheck();
  }

  delay(500);                           // Pause the loop by 0.5s
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Function when the CanSat is in rover mode
int RoverMode() {
    Serial.print(millis());
    Serial.println(";;;;;;;;;;;; Rover mode");

    OpenLatch();
    //myservo.write(170);               // Open the latch
    delay(500);
    Serial.print(millis());
    Serial.println(";;;;;;;;;;;; Parachute was deattached");


  for (int i = 0; i <= 10; i++) {        // The CanSat drives around with a certain routine
    Serial.print(millis());
    Serial.println(";;;;;;;;;;;; Ground Mission Started: Rover Driving");
    digitalWrite(DC_Motor, HIGH);
    delay(10000);
    digitalWrite(DC_Motor, LOW);
    Measument();
    delay(1000);
  }
    Serial.print(millis());
    Serial.println(";;;;;;;;;;;; Ground Mission Finisched: Rover Stoped");
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Finction to check if CanSat has landed
int LandedTest(double AltDelta){
  
  // Initiate local variables
  boolean Quick_Test = false;     // Variabel to indicate passed Quick Test
  double AltList[10];             // Array of Altitude measuments
  double sum = 0;                 // Variable that summes the presures array
  double mean;                    // Variable for mean preaure
  double QIt = 10;                // Quantity of measesuments
  double Qsteps = 100;            // time of steps in ms

  Serial.print(millis()); Serial.println(";;;;;;;;;;;; Quick altitude check");

  for (int j = 0; j <= (QIt - 1); j++){           //Loop for quick measument to check if CanSat has landed
    AltList[j] = bme.readAltitude(1013.25);
    sum += AltList[j];
    delay(100);
  }
  mean = sum / QIt;                               // Find the mean value

  mean = mean + AltDelta;                         // Subtract the Difference found in the calibration

  // Creat Variable for min and Max value
  int minValue = AltList[0];                 
  int maxValue = AltList[0];

  // Find the minimum and maximum altitude values
  for (int i = 1; i < (QIt - 1); ++i) {
    if (AltList[i] < minValue) {
      minValue = AltList[i];
    } else if (AltList[i] > maxValue) {
      maxValue = AltList[i];
    }
  }
  // Find the Decending speed
  double Speed = (maxValue - minValue)/(QIt*(Qsteps/1000));

  // Check if the Conditions are fulfilt to be considered landed
  if ((mean > 40) || (Speed > 0.5)) {
    Serial.print(millis()); Serial.print(";;;;;;;;;;;; Flighing: Altitud: "); Serial.print(mean); Serial.print(", Speed - "); Serial.println(Speed);
    Quick_Test = false;
  }
  else if (((mean <= 40) && (Speed <= 0.5))|| (digitalRead(10) == HIGH)) {
    Serial.print(millis()); Serial.print(";;;;;;;;;;;; Quick test past: Altitud: "); Serial.print(mean); Serial.print(", Speed - "); Serial.println(Speed);
    Quick_Test = true;
  }


  if (Quick_Test == true){        // If the Quick test past run long Savty test
    double S_AltList[10];         // Create local variables and Array for Long test
    double S_sum = 0;

    Serial.print(millis()); Serial.println(";;;;;;;;;;;; Perform long safty test (1 min) measument");
    for (int n = 0; n <= 9; n++){     //Loop for long measument
      S_AltList[n] = bme.readAltitude(1013.25);
      S_sum += S_AltList[n];
      delay(6000);
    }
    double S_mean = S_sum / 10;     // Find mean Altitude
    S_mean = S_mean + AltDelta;     // substract the delta

    double S_minValue = S_AltList[0];   // Create min and max variables
    double S_maxValue = S_AltList[0];
    // Find the minimum and maximum values
    for (int m = 1; m < 9; ++m) {     // find min and max altitude
      if (S_AltList[m] < S_minValue) {
        S_minValue = S_AltList[m];
      } else if (S_AltList[m] > maxValue) {
        S_maxValue = S_AltList[m];
      }
    }
    // Find speed
    double S_Speed = (S_maxValue - S_minValue)/60;
  
    if ((S_mean > 40) || (S_Speed > 0.5)) {   // check if Cansat has landed
      Serial.print(millis()); Serial.println(";;;;;;;;;;;; Safety test Failed, CanSat Flighing!");
      Landed = false;
    }
    else if (((S_mean <= 40) && (S_Speed <= 0.5)) || (digitalRead(10) == HIGH)){
      Landed = true;
      Serial.print(millis()); Serial.println(";;;;;;;;;;;; Safety test passed, CanSat Landed!");
    }
    else{
      Serial.print(millis()); Serial.println(";;;;;;;;;;;; Error with Landing Check");
      Landed = false;
    }
  }
  
  return Landed;
}


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Function to measure all variables and return them
int Measument() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(MPU_6050_Addr);
  Wire.write(ACCEL_XOUT_H);  //ACCEL_XOUT_H register
  Wire.endTransmission();
  Wire.requestFrom(MPU_6050_Addr, 14);  // request 2 bytes from the MPU-6050
  MPU_6050_ACCX = Wire.read() << 8 | Wire.read();
  MPU_6050_ACCY = Wire.read() << 8 | Wire.read();
  MPU_6050_ACCZ = Wire.read() << 8 | Wire.read();
  //MPU_6050_Temp = Wire.read() << 8 | Wire.read();       WHY?
  MPU_6050_GyroX = Wire.read() << 8 | Wire.read();
  MPU_6050_GyroY = Wire.read() << 8 | Wire.read();
  MPU_6050_GyroZ = Wire.read() << 8 | Wire.read();
  MPU_6050_ACCX = MPU_6050_ACCX / 4096;
  MPU_6050_ACCY = MPU_6050_ACCY / 4096;
  MPU_6050_ACCZ = MPU_6050_ACCZ / 4096;
  //MPU_6050_Temp = MPU_6050_Temp / 340 + 36.53;       WHY?
  MPU_6050_GyroX = MPU_6050_GyroX / 65.5;
  MPU_6050_GyroY = MPU_6050_GyroY / 65.5;
  MPU_6050_GyroZ = MPU_6050_GyroY / 65.5;
  TWMP_Out_H_data = Wire.read();
  TEM_OUT_L_data = Wire.read();
  MPU_6050_Temp = TWMP_Out_H_data << 8 | TEM_OUT_L_data;
  MPU_6050_Temp = MPU_6050_Temp/ 340 + 36.53;
  delay(1000);
  tempSensorValue = analogRead(analogInPin);
  // convert temperature data
  tempSensorValue = tempSensorValue * 5000 / 1023 * 1 / 10;
  //Transmit temperature sensor data, 1 decimal place
  //Serial.println("Timestamp_ MPU-6050_Acc_X; MPU-6050_Acc_Y; MPU-6050_Acc_Z; MPU-6050_Gyro_X; MPU-6050_Gyro_Y; MPU-6050_Gyro_Z; MPU-6050_Temp; BMP280_Temp; BMP280_Pres; BMP280_Alt; LM35DZ_Temp; Mission_Notes");
  Serial.print(millis());
  Serial.print("; ");
  Serial.print(MPU_6050_ACCX);
  Serial.print("; ");
  Serial.print(MPU_6050_ACCY);
  Serial.print("; ");
  Serial.print(MPU_6050_ACCZ);
  Serial.print("; ");
  Serial.print(MPU_6050_GyroX);
  Serial.print("; ");
  Serial.print(MPU_6050_GyroY);
  Serial.print("; ");
  Serial.print(MPU_6050_GyroZ);
  Serial.print("; ");
  Serial.print(MPU_6050_Temp);
  Serial.print("; ");
  Serial.print(bme.readTemperature());
  Serial.print("; ");
  Serial.print(bme.readPressure() / 100);
  Serial.print("; ");
  Serial.print(bme.readAltitude(1013.25));
  Serial.print("; ");
  Serial.print(tempSensorValue, 1);
  Serial.println("; ");

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

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Function to preform pre flight calibrations
double PreStartCalibration(double StartAlt) {

  Serial.print(millis());
  Serial.println(";;;;;;;;;;;; Preform prestart calibration");
  double PreStartData[2];
  double AltList[10];
  double AltSum = 0;
  for (int j = 0; j <= 9; j++) {
    AltList[j] = bme.readAltitude(1013.25);
    AltSum += AltList[j];
    delay(1000);
  }
  double AltMean = AltSum / 10;
  double DeltaAltetude = StartAlt - AltMean;


  Serial.print(millis());
  Serial.print(";;;;;;;;;;;; Mean Altitude: ");
  Serial.print(AltMean);
  Serial.println(" [m]");
  Serial.print(millis());
  Serial.print(";;;;;;;;;;;; Altitude delta: ");
  Serial.print(DeltaAltetude);
  Serial.println(" [m]");
  return DeltaAltetude;
}


//------------------------------------------------------------------------------------------------------
// Funktion to find Altitude Delta
double PreStartRelativeAlt() {
  double AltSum = 0;

  Serial.print(millis());
  Serial.println(";;;;;;;;;;;; Preform prestart relative altitude measument");
  double PreStartData[2];
  double AltList[10];
  for (int j = 0; j <= 9; j++) {
    AltList[j] = bme.readAltitude(1013.25);
    AltSum += AltList[j];
    delay(1000);
  }
  double AltMean = AltSum / 10;

  return AltMean;
}

//------------------------------------------------------------------------------------------------------
// Function to check if the CanSat has been lunched
double LaunchCheck() {

  double AltDiff = bme.readAltitude(1013.25) - Relative_Start_Altedude;

  if ((AltDiff > 10) || (digitalRead(10) == HIGH)) {  //############## Need to remove Switch test condition
    Lanched = true;
    Serial.print(millis());
    Serial.println(";;;;;;;;;;;; Rocked Lanched!!!");
  } else {
    Lanched = false;
  }

  return Lanched;
}

//------------------------------------------------------------------------------------------------------
// Function to close Parachute latch
int CloseLatch() {
  
  digitalWrite(5,HIGH); // Open direction
  

  for(stepCounter = 0; stepCounter < steps; stepCounter++) {
    digitalWrite(6,HIGH);
    delayMicroseconds(200);
    digitalWrite(6,LOW);
    delayMicroseconds(200);

  }
  
    Serial.print(millis());
    Serial.println(";;;;;;;;;;;; Latch is open, load the parachute");


  for (int o = 1; o <= 10; o++) {
    Serial.print(millis());
    Serial.print(";;;;;;;;;;;; Latch closes in: "); Serial.print(10-o); Serial.println(" s");
    delay(1000);
  }

  digitalWrite(5,LOW); // close direction

  for(stepCounter = 0; stepCounter < steps; stepCounter++) {
    digitalWrite(6,HIGH);
    delayMicroseconds(2000);
    digitalWrite(6,LOW);
    delayMicroseconds(2000);
  }

    Serial.print(millis());
    Serial.println(";;;;;;;;;;;; Latch is closed");

}

//------------------------------------------------------------------------------------------------------
// Function to open Parachute latch
int OpenLatch() {
  
  digitalWrite(5,HIGH); // Open direction
  

  for(stepCounter = 0; stepCounter < steps; stepCounter++) {
    digitalWrite(6,HIGH);
    delayMicroseconds(200);
    digitalWrite(6,LOW);
    delayMicroseconds(200);
  }
  
    Serial.print(millis());
    Serial.println(";;;;;;;;;;;; Latch is open, parachute released");

}

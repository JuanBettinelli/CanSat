#include <Adafruit_BMP280.h>



#include<Wire.h>  // I2C Libary
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#include <Adafruit_I2CDevice.h>

// BMP280
Adafruit_BMP280 bme;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  bme.begin(); 

  if (!bme.begin()) {
    Serial.println(F("could not find BMP"));
    while (1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(bme.readPressure());
}

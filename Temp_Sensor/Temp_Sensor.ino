const int analogInPin = A1; // analog input pin
float tempSensorValue = 0; // Temperature sensor data
int green = 2;
int yellow = 3;
int red = 4;


void setup() {
  // initialize digital pin 13 as an output
  pinMode(13,OUTPUT);
  // Initialize serial communication as 9600 bps:
  Serial.begin(9600);
  pinMode(green, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(red, OUTPUT);
}
 // the Loop function runs over and over again
void loop() {
 // turn the LED on (High is the voltage level)
digitalWrite(13,HIGH);
//read ADV Value
tempSensorValue = analogRead(analogInPin);
// convert temperature data
tempSensorValue = tempSensorValue * 5000/1023 * 1/10;
//Transmit temperature sensor data, 1 decimal place
Serial.println(tempSensorValue,1);
// wait for a 500 ms 
delay(500);
//turn the LED off
digitalWrite(13, LOW);
delay(500);
if (tempSensorValue > 25)
{
digitalWrite(green, HIGH);
}
else if(tempSensorValue < 25){
digitalWrite(red, HIGH);
} else{
digitalWrite(yellow, HIGH);
}
delay(500);
digitalWrite(green, LOW);
digitalWrite(yellow, LOW);
digitalWrite(red, LOW);
}

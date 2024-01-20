#include <Servo.h>

Servo myservo;

void setup() {
  myservo.attach(9); 
}

void loop() {
  myservo.write(40);
  delay(200); 
  myservo.write(140);
  delay(200); 
}
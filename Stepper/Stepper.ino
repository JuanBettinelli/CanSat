int stepCounter;
int steps = 1200;

void setup() {
  pinMode(6, OUTPUT); // Enable
  pinMode(5, OUTPUT); // Step
  pinMode(4, OUTPUT); // Richtung

  digitalWrite(6, LOW);
}

void loop() {

  digitalWrite(4,HIGH); // im Uhrzeigersinn

  for(stepCounter = 0; stepCounter < steps; stepCounter++) {
    digitalWrite(5,HIGH);
    delayMicroseconds(200);
    digitalWrite(5,LOW);
    delayMicroseconds(200);

     Serial.begin(9600);
  }
  
  delay(100);

  digitalWrite(4,LOW); // gegen den Uhrzeigersinn

  for(stepCounter = 0; stepCounter < steps; stepCounter++) {
    digitalWrite(5,HIGH);
    delayMicroseconds(2000);
    digitalWrite(5,LOW);
    delayMicroseconds(2000);
  }

  delay(100);

}
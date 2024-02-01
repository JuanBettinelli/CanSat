int stepCounter;
int steps = 1200;

void setup() {
  pinMode(12, OUTPUT); // Enable
  pinMode(6, OUTPUT); // Step
  pinMode(5, OUTPUT); // Richtung

  digitalWrite(6, LOW);
}

void loop() {

  digitalWrite(5,HIGH); // im Uhrzeigersinn

  for(stepCounter = 0; stepCounter < steps; stepCounter++) {
    digitalWrite(6,HIGH);
    delayMicroseconds(200);
    digitalWrite(6,LOW);
    delayMicroseconds(200);

     Serial.begin(9600);
  }
  
  delay(100);

  digitalWrite(5,LOW); // gegen den Uhrzeigersinn

  for(stepCounter = 0; stepCounter < steps; stepCounter++) {
    digitalWrite(6,HIGH);
    delayMicroseconds(2000);
    digitalWrite(6,LOW);
    delayMicroseconds(2000);
  }

  delay(1000);

}
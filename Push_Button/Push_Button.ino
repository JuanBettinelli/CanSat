

void setup() {
  // put your setup code here, to run once:

  pinMode(2, OUTPUT);
  pinMode(10, INPUT);
}

void loop() {

  if (digitalRead(10) == HIGH){
      digitalWrite(2, HIGH);
  }
  else if (digitalRead(10) == LOW){
    digitalWrite(2, LOW);
  }
 
}

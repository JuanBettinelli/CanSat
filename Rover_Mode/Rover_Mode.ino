

void setup() {
  // put your setup code here, to run once:

  pinMode(2, OUTPUT);
  pinMode(10, INPUT);
}

void loop() {
digitalWrite(2, LOW);
delay(100000);
digitalWrite(2, HIGH);
delay(10000);
digitalWrite(2, LOW);
delay(10000);

}

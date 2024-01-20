char input;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
Serial.println("Give me input");
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available()>0)
  {
    char input = Serial.read();
    Serial.println(input);
  }

}

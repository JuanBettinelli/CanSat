int led = 13;
int green = 2;
int yellow = 3;
int red = 4;

void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(yellow, OUTPUT);
  pinMode(red, OUTPUT);
  Serial.begin(9600);
  Serial.println("Hallo World");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Walk");
  digitalWrite(green, HIGH);
  digitalWrite(yellow, LOW);
  digitalWrite(red, LOW);
  delay(1000);
  Serial.println("Wait");
  digitalWrite(green, LOW);
  digitalWrite(yellow, HIGH);
  digitalWrite(red, LOW);
  delay(1000);
  Serial.println("Stop");
  digitalWrite(green, LOW);
  digitalWrite(yellow, LOW);
  digitalWrite(red, HIGH);
  delay(1000);
  Serial.println("Wait");
  digitalWrite(green, LOW);
  digitalWrite(yellow, HIGH);
  digitalWrite(red, LOW);
  delay(1000);
}

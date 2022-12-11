int buttonState = 0;
void setup() {
  Serial.begin(9600);
  pinMode(8, INPUT);
  
}

void loop() {
  buttonState = digitalRead(8);
  Serial.println(buttonState);
  delay(90);
}

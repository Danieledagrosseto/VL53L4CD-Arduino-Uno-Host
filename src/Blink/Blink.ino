// Basic blink example for Arduino Uno.

const int kLedPin = 13;

void setup() {
  pinMode(kLedPin, OUTPUT);
}

void loop() {
  digitalWrite(kLedPin, HIGH);
  delay(1000);
  digitalWrite(kLedPin, LOW);
  delay(1000);
}

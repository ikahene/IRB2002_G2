const int PIN_ENA = 9, PIN_IN1 = 7, PIN_IN2 = 8;

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW); digitalWrite(PIN_ENA, HIGH);
  digitalWrite(LED_BUILTIN, HIGH); delay(5000);

  digitalWrite(PIN_ENA, LOW); digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(LED_BUILTIN, LOW);  delay(3000);

  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, HIGH); digitalWrite(PIN_ENA, HIGH);
  digitalWrite(LED_BUILTIN, HIGH); delay(5000);

  digitalWrite(PIN_ENA, LOW); digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
  digitalWrite(LED_BUILTIN, LOW);  delay(3000);
}

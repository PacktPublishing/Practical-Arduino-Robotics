void setup() {
  // Use XBee's default baud rate.
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    // Serial echo.
    Serial.write(Serial.read());
  }
}
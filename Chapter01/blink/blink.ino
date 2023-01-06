void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Hello, world!");
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("LED on.");
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("LED off.");
  delay(500);
}

const float sin_freq = 2.0;
const float cos_freq = 4.0;
const float sin_amp = 10;
const float cos_amp = 2;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Argument for 1Hz sin and cos waves.
  float arg = 2 * PI * millis() / 1000.0;
  // Sine and cosine waves.
  float sinwave = sin_amp * sin(sin_freq * arg);
  float coswave = cos_amp * cos(cos_freq * arg);
  // Print four columns of data.
  Serial.print("sin:");
  Serial.print(sinwave);
  Serial.print('\t');
  Serial.print("sin*cos:");
  Serial.print(sinwave * coswave);
  Serial.print('\t');
  Serial.print("min:");
  Serial.print(-sin_amp * cos_amp);
  Serial.print('\t');
  Serial.print("max:");
  Serial.println(sin_amp * cos_amp);
  delay(10);
}
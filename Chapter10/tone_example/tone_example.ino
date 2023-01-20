short analog_input_pin = A0;
short speaker_pin = 10;

// Frequencies and note durations of a custom jingle.
int frequencies[] = { 440, 523, 659, 523, 440 };
int durations[] = { 200, 200, 200, 200, 450 };

void setup() {
  Serial.begin(115200);
}

void jingle() {
  for (int i = 0; i < 5; i++) {
    tone(speaker_pin, frequencies[i], durations[i]);
    delay(durations[i]);
  }
}

void loop() {
  // Create tone from analog input.
  int freq = 100 * ((analogRead(analog_input_pin) >> 6) + 1);
  tone(speaker_pin, freq);
  delay(5);

  // Play custom jingle whenever Serial data arrives.
  if (Serial.available()) {
    jingle();
    // Clear Serial input buffer.
    while (Serial.available()) {
      Serial.read();
    }
  }
}

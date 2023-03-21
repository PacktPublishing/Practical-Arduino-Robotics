// Example code to decode an RC receiver's PPM signal output.
// In a PPM packet, the channel values are encoded by the time between the start
// of consecutive pulses.
// PPM packets are separated by a long pause ("blank time" or "sync phase") between pulses.
// Below is a signal showing two PPM packets encoding two channels, A and B.
//    _     _     _         _    _   _
//___|_|___|_|___|_|_______| |__| |_| |___
//   [--A--|--B--]  sync   [--A-|-B-]

// The Arduino pin that is connected to the receiver's PPM output pin.
const int kInputPin = 2;
// The number of channels encoded in the PPM signal.
const int kNumChannels = 8;
// The minimum LOW time (in microseconds) between consecutive PPM packages.
const int kSyncDurationThreshold = 2500;


// All these variables are used by the ISR and should not be changed manually.
// The values contained in pulses[] will be updated by the interrupt service handler.
volatile int pulses[kNumChannels];
volatile unsigned long last_rise_time = 0;
volatile int channel_idx = 0;

// Interrupt service handler for rising PPM edges that will do the decoding.
void isr() {
  int last_pulse_duration = micros() - last_rise_time;
  last_rise_time = micros();
  pulses[channel_idx] = last_pulse_duration;
  channel_idx++;
  if (last_pulse_duration > kSyncDurationThreshold) {
    // This was the sync phase! Reset channel index.
    channel_idx = 0;
  }
}

// Driver code.
void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(kInputPin), isr, RISING);
}

// PPM decoding is fully interrupt-driven.
// loop() only prints the signal values.
void loop() {
  for (int i = 0; i < kNumChannels; i++) {
    Serial.print(pulses[i]);
    Serial.print('\t');
  }
  Serial.println();
  delay(20);
}

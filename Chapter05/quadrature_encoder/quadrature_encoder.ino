// Use Arduino Uno interrupt pins.
// Pins 2 and 3 are our only options.
const int pin_A = 2;
const int pin_B = 3;

// Variables that are changed inside ISRs should always
// be volatile.
// Variables input pin states.
volatile bool state_a = LOW;
volatile bool state_b = LOW;
// Encoder position.
volatile long encoder_position = 0;


void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(pin_A), risingIsrA, RISING);
  attachInterrupt(digitalPinToInterrupt(pin_B), risingIsrB, RISING);
}

void loop() {
  Serial.print("Position:");
  Serial.println(encoder_position);
  // Blocking call, but that’s OK.
  delay(50);  
}

// Encoder logic table:
// ---------------------------------------
// Event     | Other pin state | direction
// ---------------------------------------
// rising A         HIGH            -
//                  LOW             +
// falling A        HIGH            +
//                  LOW             -
// rising B         HIGH            +
//                  LOW             -
// falling B        HIGH            -
//                  LOW             +

void risingIsrA() {
  state_a = HIGH;
  attachInterrupt(digitalPinToInterrupt(pin_A), fallingIsrA, FALLING);
  switch (state_b) {
    case LOW:
     // Encoder moved forward.
      encoder_position++;
      break;
    case HIGH:
      // Encoder moved backwards.
      encoder_position--;
      break;
  }
}

void risingIsrB() {
  state_b = HIGH;
  attachInterrupt(digitalPinToInterrupt(pin_B), fallingIsrB, FALLING);
  switch (state_a) {
    case LOW:
      // Encoder moved backwards.
      encoder_position--;
      break;
    case HIGH:
      // Encoder moved forward.
      encoder_position++;
      break;
  }
}

void fallingIsrA() {
  state_a = LOW;
  attachInterrupt(digitalPinToInterrupt(pin_A), risingIsrA, RISING);
  switch (state_b) {
    case LOW:
      // Encoder moved backwards.
      encoder_position--;
      break;
    case HIGH:
      // Encoder moved forward.
      encoder_position++;
      break;
  }
}

void fallingIsrB() {
  state_b = LOW;
  attachInterrupt(digitalPinToInterrupt(pin_B), risingIsrB, RISING);
  switch (state_a) {
    case LOW:
      // Encoder moved forward.
      encoder_position++;
      break;
    case HIGH:
      // Encoder moved backwards.
      encoder_position--;
      break;
  }
}
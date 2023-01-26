#include <SoftwareSerial.h>

const int datalen = 10;
char databuf[datalen];

// Instantiate software Serial port.
// Pin 2 is for RX, pin 3 for TX.
SoftwareSerial ble(2, 3);

void setup() {
  ble.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // Forward Serial data to BLE interface.
  if (Serial.available()) {
    ble.write(Serial.read());
  }
  // Forward BLE data to Serial interface.
  if (ble.available()) {
    Serial.write(ble.read());
  }
}
#include <SoftwareSerial.h>
#include <SoftwareSerial.h>

// Instantiate software Serial port.
// Pin 2 is for RX, pin 3 for TX.
SoftwareSerial ble(2, 3);

void setup() {
  ble.begin(9600);
}

void loop() {
  // Print sine and cosine waves over BLE.
  float arg = 2 * millis() / 1000.0;
  ble.print(sin(arg));
  ble.print('\t');
  ble.println(cos(arg));
}

// Include the Software Serial library.
#include <SoftwareSerial.h>  
// Create a Software Serial interface for the GPS receiver.
// It uses pin 10 for RX and pin 11 for TX.
SoftwareSerial gpsSerial(10, 11);

void setup() {
// Start the Hardware Serial interface.
  Serial.begin(115200);
  // Start the Software Serial interface.
  // GPS receivers use a baud rate of 9600.
  gpsSerial.begin(9600);
}

void loop() {
  // If we received a byte from the GPS receiver...
  if (gpsSerial.available()) {
    // ... print it to the Serial Monitor.
    Serial.write(gpsSerial.read());
  }
}

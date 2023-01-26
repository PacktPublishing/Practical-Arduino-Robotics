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
  if (ble.available() == datalen) {
    // Complete control pad message received.
    // Read the entire message into the buffer.
    for (int i = 0; i < datalen; i++) {
      databuf[i] = ble.read();
    }
    // The third byte tells us which button was pressed.
    if (databuf[2] == '1') {
      Serial.println(1);
    } else if (databuf[2] == '2') {
      Serial.println(2);
    } else if (databuf[2] == '3') {
      Serial.println(3);
    } else if (databuf[2] == '4') {
      Serial.println(4);
    } else if (databuf[2] == '5') {
      Serial.println("UP");
    } else if (databuf[2] == '6') {
      Serial.println("DOWN");
    } else if (databuf[2] == '7') {
      Serial.println("LEFT");
    } else if (databuf[2] == '8') {
      Serial.println("RIGHT");
    }
  }
}

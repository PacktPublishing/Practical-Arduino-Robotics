// Include I2C library.
#include <Wire.h>
// I2C parameters of BNO055 chip.
const uint8_t bno_address = 0x28;
const uint8_t bno_temp_register = 0x34;
const uint8_t bno_temp_data_length = 1;

void setup() {
  // Start the Serial enterface.
  Serial.begin(115200);
  // Start the I2C bus.
  Wire.begin();
  // Give the BNO055 some time to start.
  delay(1000);
}

void loop() {
  // Periodically print the BNO055 temperature.
  if (millis() % 200 == 0) {
    Serial.print("BNO055 temperature [C]: ");
    Serial.println(bnoReadTemp());
  }
}

// Custom function to access the BNO055 temperature over I2C.
uint8_t bnoReadTemp() {
  // Begin I2C transmission with BNO055.
  Wire.beginTransmission(bno_address);
  // Write the address of the temperature register.
  // This requests temperature data from the BNO055.
  Wire.write(bno_temp_register);
  // End the I2C transmission.
  Wire.endTransmission();
  // Request one byte of data from the BNO055.
  // This will be the temperature data.
  Wire.requestFrom(bno_address, bno_temp_data_length);
  // Read and return the requested temperature data.
  return Wire.read();
}
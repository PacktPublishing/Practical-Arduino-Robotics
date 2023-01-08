#include <Wire.h> // Include the I2C library. 

const uint8_t bno_address = 0x28; 

const uint8_t bno_temp_register = 0X34; 

 

void setup() { 

  Serial.begin(115200); 

  Wire.begin(); // Start the I2C bus. 

  delay(1000); // Give the BNO055 time to boot. 

} 

 

void loop() { 

  // Periodically print the BNO’s temperature to verify that the IC2 communication is workignworking. 

  if (millis() % 200 == 0) { 

    Serial.print("BNO055 temperature [°C]: "); 

    Serial.println(bnoReadTemp()); 

  } 

} 

 

// Custom function to access the BNO’s temperature over I2C. 

uint8_t bnoReadTemp() { 

  Wire.beginTransmission(bno_address); // Begin I2C transmission to address 0x28. 

  Wire.write((uint8_t)bno_temp_register); // Write the address of the temperature register to request temperature data. 

  Wire.endTransmission(); // End the I2C transmission. 

  Wire.requestFrom(bno_address, (byte)1); // Request one byte of data from the BNO. This will be the temperature data. 

  return Wire.read(); // Read the requested temperate data and return it. 

} 
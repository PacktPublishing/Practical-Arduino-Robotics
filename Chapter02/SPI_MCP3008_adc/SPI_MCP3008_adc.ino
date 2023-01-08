// Include SPI library.
#include <SPI.h>
// Using pin 4 for chip select signal.
const int cs_pin = 4;
// MCP setting for single-ended measurements.
const int mcp_set_sgl = 1;

void setup() {
  Serial.begin(115200);
  // Start the SPI bus.
  SPI.begin();
  // Configure chip select pin as OUTPUT.
  pinMode(cs_pin, OUTPUT);
}

void loop() {
  // Read ADC data from MCP3008 via SPI.
  Serial.print(mcp_read_channel(0));
  // Periodically print the data to the Serial Plotter.
  if (millis() % 100 == 0) {
    // Print min and max values.
    Serial.print('\t');
    Serial.print(0);
    Serial.print('\t');
    Serial.println(1023);
  }
}

// Custom function that uses the MCP3008 for a single-ended
// analog read of the specified channel.
int mcp_read_channel(uint8_t channel) {
  if (channel > 7) {
    return -1;  // Return error code –1 if channel is invalid.
  }
  // Start the SPI transaction.
  // Bitrate is 1Mhz, bitorder is MSB first, SPI mode is 0.
  SPI.beginTransaction(SPISettings(1e6, MSBFIRST, SPI_MODE0));
  // Pull CS LOW to select the MCP3008.
  digitalWrite(cs_pin, LOW);
  // Transmit start bit.
  SPI.transfer(0x01);
  // Transfer settings bit and channel bits, padded with zeros.
  // Capture the returned byte in the msb variable.
  uint8_t msb = SPI.transfer(mcp_set_sgl << 7 | channel << 4);
  // Use a bitmask to extract the two payload bits.
  // These are the two most significant bits of the ADC value.
  msb = msb & 0b11;
  // Transfer a dummy byte and capture the returned byte.
  // This is the LSB of the 10 Bit ADC value.
  uint8_t lsb = SPI.transfer(0);
  // Push CS HIGH to release the MCP3008.
  digitalWrite(cs_pin, HIGH);
  // End the SPI transaction.
  SPI.endTransaction();
  // Combine the MSB and the LSB into the 16 bit return value.
  return msb << 8 | lsb;
}
#include <LiquidCrystal.h>

const short rs_pin = 2;
const short rw_pin = 3;
const short en_pin = 4;

const short d4_pin = 8;
const short d5_pin = 9;
const short d6_pin = 10;
const short d7_pin = 11;

// Instantiate LCD object in 4-bit mode.
LiquidCrystal myLCD(rs_pin, rw_pin, en_pin, d4_pin, d5_pin, d6_pin, d7_pin);

void setup()
{
  myLCD.begin(16, 2);
  myLCD.print("Arduino Robots!");
}

void loop() {
  // Set cursor to column 0, row 1.
  myLCD.setCursor(0, 1);
  myLCD.print(millis());
  delay(50);
}

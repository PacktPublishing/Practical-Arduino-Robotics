#include "QuadEncoder.h"

// Instantiate encoder object on pins 2 and 3.
QuadEncoder myEnc(2, 3);

// Global wrappers for the interrupt handlers.
void myEncChangeIsrA() {
    myEnc.changeIsrA();
}
void myEncChangeIsrB() {
    myEnc.changeIsrB();
}

void setup() {
    Serial.begin(115200);
    // Attach interrupt handlers to interrupt sources.
    attachInterrupt(digitalPinToInterrupt(myEnc.getPinA()), myEncChangeIsrA,
                    CHANGE);
    attachInterrupt(digitalPinToInterrupt(myEnc.getPinB()), myEncChangeIsrB,
                    CHANGE);
}

void loop() {
    // Run update as often as possible.
    // Do not use delay() in the loop() function!
    myEnc.update();
    // Periodically print position and veolocity.
    if (millis() % 50 == 0) {
        Serial.print(myEnc.getPosition());
        Serial.print('\t');
        Serial.println(myEnc.getVelocity());
    }
}

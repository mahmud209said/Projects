#include <Arduino.h>

#define IR_PIN 4   // Change to the GPIO you connected the OUT pin to

void setup() {
    Serial.begin(9600);
    pinMode(IR_PIN, INPUT);

    Serial.println("IR Sensor Test Started");
}

void loop() {
    int sensorState = digitalRead(IR_PIN);

    if (sensorState == LOW) {
        Serial.println("                    Object detected");
    } else {
        Serial.println("                    No object detected");
    }

    delay(100);

    
}

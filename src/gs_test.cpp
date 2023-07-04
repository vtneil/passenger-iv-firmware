#include <Arduino.h>

//UART Serial2(6, 7);

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);
}

void loop() {
    while (Serial2.available()) Serial.write(Serial2.read());
}

/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-serial-monitor
 */

#include "HardwareSerial.h"

HardwareSerial SerialMSP(2);

void setup() {
  Serial.begin(9600);
  SerialMSP.begin(115200, SERIAL_8N1, 16, 17);
}

void loop() {
  if (Serial.available()) { // if there is data comming
    String direction = Serial.readStringUntil('\n'); // read string until newline character

    if (direction == "go left") {
      SerialMSP.write(0);
    } else if (direction == "go right") {
      SerialMSP.write(1);
    } else if (direction == "go up") {
      SerialMSP.write(2);
    } else if (direction == "go down") {
      SerialMSP.write(3);
    } else if (direction == "centered") {
      SerialMSP.write(4);
    } else if (direction == "no target") {
      SerialMSP.write(5);
    }

    Serial.println(String("Received ") += direction);
  }
}
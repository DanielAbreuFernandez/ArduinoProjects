#include <Wire.h>
#include <Adafruit_DRV2917.h> // or DRV8871 if that's the chip

Adafruit_DRV2917 motor = Adafruit_DRV2917();

void setup() {
  Serial.begin(9600);
  if (!motor.begin()) {
    Serial.println("Failed to find DRV2927 chip");
    while (1);
  }
  Serial.println("Motor ready!");
}

void loop() {
  motor.setSpeed(255); // full speed
  delay(1000);
  motor.setSpeed(0);   // stop
  delay(1000);
}

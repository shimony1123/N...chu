#include <Arduino.h>

int led;
void setup(){
  Serial.begin(9600);
}

void loop() {
  Serial.println("Hello sekai!!");
  delay(1000);
}
#include <Arduino.h>

void setup() {
      pinMode(A2, OUTPUT);
      pinMode(A3, OUTPUT);
}
void loop() {
      digitalWrite(A2, LOW);
      digitalWrite(A3, HIGH);
      delay(100);
      digitalWrite(A2, HIGH);
      digitalWrite(A3, LOW);
      delay(100);
}
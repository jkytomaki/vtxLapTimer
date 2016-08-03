#include <Arduino.h>

void setPinHigh(int pin){
  digitalWrite(D8, HIGH);

  digitalWrite(D0, bitRead(pin, 0));
  digitalWrite(D1, bitRead(pin, 1));
  digitalWrite(D2, bitRead(pin, 2));
  
}

void setPinLow(int pin){
  digitalWrite(D8, LOW);

  digitalWrite(D0, bitRead(pin, 0));
  digitalWrite(D1, bitRead(pin, 1));
  digitalWrite(D2, bitRead(pin, 2));
  
}

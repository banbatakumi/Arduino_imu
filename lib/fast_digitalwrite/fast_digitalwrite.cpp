#include "fast_digitalwrite.h"

#include <Arduino.h>

void High(unsigned char pin) {
      switch (pin) {
            case 0:
                  PORTD |= _BV(0);
                  break;
            case 1:
                  PORTD |= _BV(1);
                  break;
            case 2:
                  PORTD |= _BV(2);
                  break;
            case 3:
                  PORTD |= _BV(3);
                  break;
            case 4:
                  PORTD |= _BV(4);
                  break;
            case 5:
                  PORTD |= _BV(5);
                  break;
            case 6:
                  PORTD |= _BV(6);
                  break;
            case 7:
                  PORTD |= _BV(7);
                  break;
            case 8:
                  PORTB |= _BV(0);
                  break;
            case 9:
                  PORTB |= _BV(1);
                  break;
            case 10:
                  PORTB |= _BV(2);
                  break;
            case 11:
                  PORTB |= _BV(3);
                  break;
            case 12:
                  PORTB |= _BV(4);
                  break;
            case 13:
                  PORTB |= _BV(5);
                  break;
            case 14:
                  PORTC |= _BV(0);
                  break;
            case 15:
                  PORTC |= _BV(1);
                  break;
            case 16:
                  PORTC |= _BV(2);
                  break;
            case 17:
                  PORTC |= _BV(3);
                  break;
            case 18:
                  PORTC |= _BV(4);
                  break;
            case 19:
                  PORTC |= _BV(5);
                  break;
            default:
                  return;
                  break;
      }
}

void Low(unsigned char pin) {
      switch (pin) {
            case 0:
                  PORTD &= ~_BV(0);
                  break;
            case 1:
                  PORTD &= ~_BV(1);
                  break;
            case 2:
                  PORTD &= ~_BV(2);
                  break;
            case 3:
                  PORTD &= ~_BV(3);
                  break;
            case 4:
                  PORTD &= ~_BV(4);
                  break;
            case 5:
                  PORTD &= ~_BV(5);
                  break;
            case 6:
                  PORTD &= ~_BV(6);
                  break;
            case 7:
                  PORTD &= ~_BV(7);
                  break;
            case 8:
                  PORTB &= ~_BV(0);
                  break;
            case 9:
                  PORTB &= ~_BV(1);
                  break;
            case 10:
                  PORTB &= ~_BV(2);
                  break;
            case 11:
                  PORTB &= ~_BV(3);
                  break;
            case 12:
                  PORTB &= ~_BV(4);
                  break;
            case 13:
                  PORTB &= ~_BV(5);
                  break;
            case 14:
                  PORTC &= ~_BV(0);
                  break;
            case 15:
                  PORTC &= ~_BV(1);
                  break;
            case 16:
                  PORTC &= ~_BV(2);
                  break;
            case 17:
                  PORTC &= ~_BV(3);
                  break;
            case 18:
                  PORTC &= ~_BV(4);
                  break;
            case 19:
                  PORTC &= ~_BV(5);
                  break;
            default:
                  return;
                  break;
      }
}
#ifndef INIT_TIMER0_H
#define INIT_TIMER0_H

#include "Arduino.h"

void init_timer0() {
  #if defined(WGM01) // if Timer0 has PWM
    TCCR0A = (1 << WGM01) | (1 << WGM00);
  #endif
  #if defined(TCCR0B) //The x61 has a wacky Timer0!
    TCCR0B = (timer0Prescaler << CS00);
  #elif defined(TCCR0A)  // Tiny x8 has no PWM from timer0
    TCCR0A = (timer0Prescaler << CS00);
  #else // tiny26 has no TCCR0A at all, only TCCR0
    TCCR0 = (timer0Prescaler << CS00);
  #endif
}

#endif

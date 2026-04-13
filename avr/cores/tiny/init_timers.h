#ifndef INIT_TIMERS_H
#define INIT_TIMERS_H

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

void init_timer1() {
  #if defined(__AVR_ATtiny841__) || defined(__AVR_ATtiny441__)
    TCCR1A   = (1 << WGM10) | (1 << COM1A1)| (1 << COM1B1); // enable OC1A, OC1B
    TCCR2A   = (1 << WGM20) | (1 << COM2A1)| (1 << COM2B1); // enable OC2A, OC2B
    #if !defined(TIMER1_USE_FAST_PWM)
      TCCR1B = (ToneTimer_Prescale_Index << CS10); // set the clock - do this last, always!
      TCCR2B = (ToneTimer_Prescale_Index << CS10); // set the clock - cause it starts the timer!
    #else
      TCCR1B = (1 << WGM12) | (ToneTimer_Prescale_Index << CS10); // set the clock - do this last, always!
      TCCR2B = (1 << WGM22) | (ToneTimer_Prescale_Index << CS10); // set the clock - cause it starts the timer!
    #endif
  #elif defined(__AVR_ATtiny828__)
    TCCR1A   = (1 << WGM10) | (1 << COM1A1)| (1 << COM1B1); // enable OC1A, OC1B
    #if !defined(TIMER0_USE_FAST_PWM)
      TCCR1B =(ToneTimer_Prescale_Index << CS10); // set the clock
    #else
      TCCR1B =(1 << WGM12) | (ToneTimer_Prescale_Index << CS10); // set the clock
    #endif
  #elif defined(__AVR_ATtiny43__)
    #if !defined(TIMER0_USE_FAST_PWM)
      TCCR1A = 1; //WGM 10=1, WGM11=1 // Phase correct
    #else
      TCCR1A = 3; //WGM 10=1, WGM11=1 // Fast
    #endif
    TCCR1B = (ToneTimer_Prescale_Index << CS10);
  #elif defined(TCCR1) // ATtiny25/45/85
    // Use the Tone Timer for fast PWM as phase correct not supported by this timer
    GTCCR = (1 << PWM1B);
    TCCR1 = (1 << CTC1) | (1 << PWM1A) | (ToneTimer_Prescale_Index << CS10);
    /* Fast mode the only option here! */
  #elif defined(TCCR1E) // ATtiny261/461/861
    // Phase correct PWM
    TCCR1A = (1 << PWM1A) | (1 << PWM1B);
    TCCR1C = (1 << PWM1D);
    #if !defined(TIMER1_USE_FAST_PWM)
      TCCR1D = (1 << WGM10);
    #else
      TCCR1D = 0
    #endif
    TCCR1B = (ToneTimer_Prescale_Index << CS10);
  #elif  defined(__AVR_ATtinyX7__)
    /* Like the 841/441/828 we turn on the output compare, and analogWrite() only twiddles the enable bits */
    TCCR1A = (1 << COM1A1)  |(1 << COM1B1) | (1 << WGM10);
    #if !defined(TIMER1_USE_FAST_PWM)
      TCCR1B = (ToneTimer_Prescale_Index << CS10);
    #else
      TCCR1B = (1 << WGM12) | (ToneTimer_Prescale_Index << CS10);
    #endif
    TCCR1D = 0;
  #elif defined(__AVR_ATtiny26__)
    TCCR1A = (1 << PWM1A) | (1 << PWM1B);
    TCCR1B = ToneTimer_Prescale_Index;
    OCR1C = 0xFE; // Top count
  #else // ATtiny24/44/84, ATtiny48/88, ATtiny2313/4313
      TCCR1A = (1 << WGM10);
    #if !defined(TIMER1_USE_FAST_PWM)
      TCCR1B = (ToneTimer_Prescale_Index << CS10); //set the clock
    #else
      TCCR1B = (1 << WGM12) | (ToneTimer_Prescale_Index << CS10); //set the clock
    #endif
  #endif
}

#endif

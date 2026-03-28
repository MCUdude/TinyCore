#include "wiring_private.h"
#include "pins_arduino.h"
#include "init_timer0.h"

// This function is ran before main(), but after setup_millis() and makes sure timer0 gets set up correctly
void timer0_setup(void) __attribute__ ((naked)) __attribute__ ((used)) __attribute__ ((section (".init7")));

void timer0_setup() {
  init_timer0();
}

void analogWrite(uint8_t pin, int val) {
  if(__builtin_constant_p(pin)) {
    // No stupid exception here - nobody analogWrite()'s to pins that don't exist!
    if (pin >= NUM_DIGITAL_PINS) badArg("analogWrite to constant pin number that is not a valid pin");
  }
  // let's wait until the end to set pinMode - why output an unknown value for a few dozen clock cycles while we sort out the pwm channel?
  if (val <= 0) {
    digitalWrite(pin, LOW);
  }
  else if (val >= 255)
  {
    digitalWrite(pin, HIGH);
  }
  else
  {
    uint8_t timer = digitalPinToTimer(pin);
    #if defined(TOCPMCOE)
      if (timer) {
        uint8_t bitmask = timer & 0xF0;
        if (timer & 0x08) {
          _SWAP(bitmask);
        }
        timer &= 0x07;
        switch (timer) {
          case TIMER0A:
            OCR0A = val;
            break;
          case TIMER1A:
            OCR1A = val;
            break;
          case TIMER1B:
            OCR1B = val;
            break;
          #if defined(TCCR2A)
            // only test for these cases on x41
            case TIMER2A:
              OCR2A = val;
              break;
            case TIMER2B:
              OCR2B = val;
              break;
            //end of x41-only
          #endif
          //case TIMER0B:
          default:
            // if it's not 0, and it's not one of the other timers, it's gotta be TIMER0B.
            OCR0B = val;
            break;
        }
        // In any event we can now switch OE for that pin.
        TOCPMCOE |= bitmask;
      } else // has to end with this, from if (timer)
    #else //Non-TOCPMCOE implementation
      // Timer0 has a output compare channel A (most parts)
      #if defined(TCCR0A) && defined(COM0A1)
        if (timer == TIMER0A) {
          // connect pwm to pin on timer 0, channel A
          TCCR0A |= (1 << COM0A1);
          OCR0A = val; // set pwm duty
        } else
      #endif
      // Timer0 has a output compare channel B (most parts)
      #if defined(TCCR0A) && defined(COM0B1)
        if (timer == TIMER0B) {
          // connect pwm to pin on timer 0, channel B
          TCCR0A |= (1 << COM0B1);
          OCR0B = val; // set pwm duty
        } else
      #endif

      // TCCR1D is present only on tinyx61 and tinyx7 and there's no TCCR1A on Tiny85
      // Hence this line is approximately "If Timer1 has PWM, and isn't some wacky thing"
      #if defined(TCCR1A) && defined(COM1A1) && !defined(TCCR1D)
        //So this handles "normal" timers
        if (timer == TIMER1A) {
          // connect pwm to pin on timer 1, channel A
          TCCR1A |= (1 << COM1A1);
          OCR1A = val; // set pwm duty
        } else
      #endif
      #if defined(TCCR1A) && defined(COM1B1) && !defined(TCCR1D)
        if (timer == TIMER1B) {
          // connect pwm to pin on timer 1, channel B
          TCCR1A |= (1 << COM1B1);
          OCR1B = val; // set pwm duty
        } else
      #endif
      // Handle the Timer1 flexible PWM on the x7
      #if !defined(TCCR1E) && defined(TCCR1D)
        if (timer & 0xF0) { // x7 timer will be either 0x01 (Timer0A) or will be 0x_3 or 0x_4 with the
          // high nybble containing bitmask for the enable pins.
          // Timer 1 - we enable the COM1xn bits in init() so all we need to do is set OCR1x
          // timer & 0xF0 gives the bitmask, and if it is to be swapped, swap it,

          uint8_t bitmask = 0xF0 & timer;
          if ((bitmask & (bitmask -1)) != 0) {
            return;
          }
          if (timer & 0x04) {
            OCR1B = val;
          } else {
            OCR1A = val;
            _SWAP(bitmask); // Timer1A controls the low half of bits in the TCCR1D register.
          }
          TCCR1D |= bitmask;
        } else
      #endif
      // ATtiny x61
      #if defined(TCCR1E) //Tiny861
        if (timer == TIMER1A) {
          // connect pwm to pin on timer 1, channel A
          TCCR1C |= (1 << COM1A1S);
          OCR1A = val; // set pwm duty
        } else if (timer == TIMER1B) {
          // connect pwm to pin on timer 1, channel B
          TCCR1C |= (1 << COM1B1S);
          OCR1B = val; // set pwm duty
        } else if (timer == TIMER1D) {
          // connect pwm to pin on timer 1, channel D
          TCCR1C |= (1 << COM1D1); /* #614 - not sure why I was doing it wrong for all PWM here.  */
          OCR1D = val; // set pwm duty
        } else
      #endif
      // ATtiny x5
      #if defined(TCCR1) && defined(COM1A1)
        if (timer == TIMER1A) {
          // connect pwm to pin on timer 1, channel A
          TCCR1 |= (1 << COM1A1);
          OCR1A = val; // set pwm duty
        } else
      #endif
      // ATtiny x5
      #if defined(TCCR1) && defined(COM1B1)
        if (timer == TIMER1B) {
          // connect pwm to pin on timer 1, channel B
          GTCCR |= (1 << COM1B1);
          OCR1B = val; // set pwm duty
        } else
      #endif
    #endif // end non-TOCPMCOE implementation
    {
      if (val < 128) {
        digitalWrite(pin, LOW);
      } else {
        digitalWrite(pin, HIGH);
      }
    }
  }
  pinMode(pin,OUTPUT);
}

/*
  wiring.c - Partial implementation of the Wiring API for the ATmega8.
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.c 970 2010-05-25 20:16:15Z dmellis $

  Modified 28-08-2009 for attiny84 R.Wiersma
  Modified 14-10-2009 for attiny45 Saposoft
  Modified 20-11-2010 - B.Cook - Rewritten to use the various Veneers.
*/

#include "wiring_private.h"
#include <avr/boot.h>





#if F_CPU > 12000000L
  // above 12mhz, prescale by 128, the highest prescaler available
  #define ADC_ARDUINO_PRESCALER   B111
#elif F_CPU >= 6000000L
  // 12 MHz / 64 ~= 188 KHz
  // 8 MHz / 64 = 125 KHz
  #define ADC_ARDUINO_PRESCALER   B110
#elif F_CPU >= 3000000L
  // 4 MHz / 32 = 125 KHz
  #define ADC_ARDUINO_PRESCALER   B101
#elif F_CPU >= 1500000L
  // 2 MHz / 16 = 125 KHz
  #define ADC_ARDUINO_PRESCALER   B100
#elif F_CPU >= 750000L
  // 1 MHz / 8 = 125 KHz
  #define ADC_ARDUINO_PRESCALER   B011
#elif F_CPU < 400000L
  // 128 kHz / 2 = 64 KHz -> This is the closest you can get, the prescaler is 2
  #define ADC_ARDUINO_PRESCALER   B000
#else //speed between 400khz and 750khz
  #define ADC_ARDUINO_PRESCALER   B010 //prescaler of 4
#endif



#if INITIALIZE_SECONDARY_TIMERS
static void initToneTimerInternal(void);
#endif

static void __empty() {
    // Empty
  }
void yield(void) __attribute__ ((weak, alias("__empty")));


void delay(unsigned long ms) //non-millis-timer-dependent delay()
{
  #ifdef MICROS_ENABLED

  #else
  while(ms--){
    yield();
    delayMicroseconds(999);
  }
  #endif
}

/* Delay for the given number of microseconds.  Assumes a 1, 8, 12, 16, 20 or 24 MHz clock. */
void delayMicroseconds(unsigned int us)
{
  #define _MORENOP_ "" // redefine to include NOPs depending on frequency

  // call = 4 cycles + 2 to 4 cycles to init us(2 for constant delay, 4 for variable)

  // calling avrlib's delay_us() function with low values (e.g. 1 or
  // 2 microseconds) gives delays longer than desired.
  //delay_us(us);
  #if F_CPU >= 24000000L
    // for the 24 MHz clock for the adventurous ones, trying to overclock

    // zero delay fix
    if (!us) return; //  = 3 cycles, (4 when true)

    // the following loop takes a 1/6 of a microsecond (4 cycles)
    // per iteration, so execute it six times for each microsecond of
    // delay requested.
    us *= 6; // x6 us, = 7 cycles

    // account for the time taken in the preceding commands.
    // we just burned 22 (24) cycles above, remove 5, (5*4=20)
    // us is at least 6 so we can subtract 5
    us -= 5; //=2 cycles

  #elif F_CPU >= 20000000L
    // for the 20 MHz clock on rare Arduino boards

    // for a one-microsecond delay, simply return.  the overhead
    // of the function call takes 18 (20) cycles, which is 1us
    __asm__ __volatile__ (
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop"); //just waiting 4 cycles
    if (us <= 1) return; //  = 3 cycles, (4 when true)

    // the following loop takes a 1/5 of a microsecond (4 cycles)
    // per iteration, so execute it five times for each microsecond of
    // delay requested.
    us = (us << 2) + us; // x5 us, = 7 cycles

    // account for the time taken in the preceding commands.
    // we just burned 26 (28) cycles above, remove 7, (7*4=28)
    // us is at least 10 so we can subtract 7
    us -= 7; // 2 cycles

  #elif F_CPU >= 18432000L
    // for a one-microsecond delay, simply return.  the overhead
    // of the function call takes 18 (20) cycles, which is approx. 1us
    __asm__ __volatile__ (
      "nop" "\n\t"
      "nop" "\n\t"
      "nop" "\n\t"
      "nop"); //just waiting 4 cycles

    if (us <= 1) return; //  = 3 cycles, (4 when true)

    // the following loop takes nearly 1/5 (0.217%) of a microsecond (4 cycles)
    // per iteration, so execute it five times for each microsecond of
    // delay requested.
    us = (us << 2) + us; // x5 us, = 7 cycles

    // user wants to wait 7us or more -- here we can use approximation
    if (us > 34) { // 3 cycles
      // Since the loop is not accurately 1/5 of a microsecond we need
      // to multiply us by (18.432 / 20), very close to 60398 / 2.**16.

      // Approximate (60398UL * us) >> 16 by using 60384 instead.
      // This leaves a relative error of 232ppm, or 1 in 4321.
      unsigned int r = us - (us >> 5);  // 30 cycles
      us = r + (r >> 6) - (us >> 4);    // 55 cycles
      // careful: us is generally less than before, so don't underrun below

      // account for the time taken in the preceding and following commands.
      // we are burning 114 (116) cycles, remove 29 iterations: 29*4=116.

      /* TODO: is this calculation correct.  Right now, we do
                function call           6 (+ 2) cycles
                wait at top             4
                comparison false        3
                multiply by 5           7
                comparison false        3
                compute r               30
                update us               55
                subtraction             2
                return                  4
                total                   --> 114 (116) cycles
       */

      // us dropped to no less than 32, so we can subtract 29
      us -= 29; // 2 cycles
    } else {
      // account for the time taken in the preceding commands.
      // we just burned 30 (32) cycles above, remove 8, (8*4=32)
      // us is at least 10, so we can subtract 8
      us -= 8; // 2 cycles
    }

  #elif F_CPU >= 18000000L
    // for the 18 MHz clock, if somebody is working with USB
    // or otherwise relating to 12 or 24 MHz clocks

    // for a 1 microsecond delay, simply return.  the overhead
    // of the function call takes 14 (16) cycles, which is .8 us
    if (us <= 1) return; // = 3 cycles, (4 when true)

    // make the loop below last 6 cycles
  #undef  _MORENOP_
  #define _MORENOP_ " nop \n\t  nop \n\t"

    // the following loop takes 1/3 of a microsecond (6 cycles) per iteration,
    // so execute it three times for each microsecond of delay requested.
    us = (us << 1) + us; // x3 us, = 5 cycles

    // account for the time taken in the preceding commands.
    // we burned 20 (22) cycles above, plus 2 more below, remove 4 (4*6=24),
    // us is at least 6 so we may subtract 4
    us -= 4; // = 2 cycles

  #elif F_CPU >= 16500000L
    // for the special 16.5 MHz clock

    // for a one-microsecond delay, simply return.  the overhead
    // of the function call takes 14 (16) cycles, which is about 1us
    if (us <= 1) return; //  = 3 cycles, (4 when true)

    // the following loop takes 1/4 of a microsecond (4 cycles) times 32./33.
    // per iteration, thus rescale us by 4. * 33. / 32. = 4.125 to compensate
    us = (us << 2) + (us >> 3); // x4.125 with 23 cycles

    // account for the time taken in the preceding commands.
    // we burned 38 (40) cycles above, plus 2 below, remove 10 (4*10=40)
    // us is at least 8, so we subtract only 7 to keep it positive
    // the error is below one microsecond and not worth extra code
    us -= 7; // = 2 cycles

  #elif F_CPU >= 16000000L
    // for the 16 MHz clock on most Arduino boards

    // for a one-microsecond delay, simply return.  the overhead
    // of the function call takes 14 (16) cycles, which is 1us
    if (us <= 1) return; //  = 3 cycles, (4 when true)

    // the following loop takes 1/4 of a microsecond (4 cycles)
    // per iteration, so execute it four times for each microsecond of
    // delay requested.
    us <<= 2; // x4 us, = 4 cycles

    // account for the time taken in the preceding commands.
    // we just burned 19 (21) cycles above, remove 5, (5*4=20)
    // us is at least 8 so we can subtract 5
    us -= 5; // = 2 cycles,

  #elif F_CPU >= 14745600L
    // for a one-microsecond delay, simply return.  the overhead
    // of the function call takes 14 (16) cycles, which is approx. 1us

    if (us <= 1) return; //  = 3 cycles, (4 when true)

    // the following loop takes nearly 1/4 (0.271%) of a microsecond (4 cycles)
    // per iteration, so execute it four times for each microsecond of
    // delay requested.
    us <<= 2; // x4 us, = 4 cycles

    // user wants to wait 8us or more -- here we can use approximation
    if (us > 31) { // 3 cycles
      // Since the loop is not accurately 1/4 of a microsecond we need
      // to multiply us by (14.7456 / 16), very close to 60398 / 2.**16.

      // Approximate (60398UL * us) >> 16 by using 60384 instead.
      // This leaves a relative error of 232ppm, or 1 in 4321.
      unsigned int r = us - (us >> 5);  // 30 cycles
      us = r + (r >> 6) - (us >> 4);    // 55 cycles
      // careful: us is generally less than before, so don't underrun below

      // account for the time taken in the preceding and following commands.
      // we are burning 107 (109) cycles, remove 27 iterations: 27*4=108.

      // us dropped to no less than 29, so we can subtract 27
      us -= 27; // 2 cycles
    } else {
      // account for the time taken in the preceding commands.
      // we just burned 23 (25) cycles above, remove 6, (6*4=24)
      // us is at least 8, so we can subtract 6
      us -= 6; // 2 cycles
    }

  #elif F_CPU >= 12000000L
    // for the 12 MHz clock if somebody is working with USB

    // for a 1 microsecond delay, simply return.  the overhead
    // of the function call takes 14 (16) cycles, which is 1.3us
    if (us <= 1) return; //  = 3 cycles, (4 when true)

    // the following loop takes 1/3 of a microsecond (4 cycles)
    // per iteration, so execute it three times for each microsecond of
    // delay requested.
    us = (us << 1) + us; // x3 us, = 5 cycles

    // account for the time taken in the preceding commands.
    // we just burned 20 (22) cycles above, remove 5, (5*4=20)
    // us is at least 6 so we can subtract 5
    us -= 5; //2 cycles

  #elif F_CPU >= 8000000L
    // for the 8 MHz internal clock

    // for a 1 and 2 microsecond delay, simply return.  the overhead
    // of the function call takes 14 (16) cycles, which is 2us
    if (us <= 2) return; //  = 3 cycles, (4 when true)

    // the following loop takes 1/2 of a microsecond (4 cycles)
    // per iteration, so execute it twice for each microsecond of
    // delay requested.
    us <<= 1; //x2 us, = 2 cycles

    // account for the time taken in the preceding commands.
    // we just burned 17 (19) cycles above, remove 4, (4*4=16)
    // us is at least 6 so we can subtract 4
    us -= 4; // = 2 cycles

  #elif F_CPU >= 6000000L
    // for that unusual 6mhz clock...

    // for a 1 to 3 microsecond delay, simply return.  the overhead
    // of the function call takes 14 (16) cycles, which is 2.5us
    if (us <= 3) return; //  = 3 cycles, (4 when true)

    // make the loop below last 6 cycles
  #undef  _MORENOP_
  #define _MORENOP_ " nop \n\t  nop \n\t"

    // the following loop takes 1 microsecond (6 cycles) per iteration
    // we burned 15 (17) cycles above, plus 2 below, remove 3 (3 * 6 = 18)
    // us is at least 4 so we can subtract 3
    us -= 3; // = 2 cycles

  #elif F_CPU >= 4000000L
    // for that unusual 4mhz clock...

    // for a 1 to 4 microsecond delay, simply return.  the overhead
    // of the function call takes 14 (16) cycles, which is 4us
    if (us <= 4) return; //  = 3 cycles, (4 when true)

    // the following loop takes 1 microsecond (4 cycles)
    // per iteration, so nothing to do here! \o/
    // ... in terms of rescaling.  We burned 15 (17) above plus 2 below,
    // so remove 5 (5 * 4 = 20), but we may at most remove 4 to keep us > 0.
    us -= 4; // = 2 cycles

  #elif F_CPU >= 2000000L
    // for that unusual 2mhz clock...

    // for a 1 to 9 microsecond delay, simply return.  the overhead
    // of the function call takes 14 (16) cycles, which is 8us
    if (us <= 9) return; //  = 3 cycles, (4 when true)
    // must be at least 10 if we want to do /= 2 -= 4

    // divide by 2 to account for 2us runtime per loop iteration
    us >>= 1; // = 2 cycles;

    // the following loop takes 2 microseconds (4 cycles) per iteration
    // we burned 17 (19) above plus 2 below,
    // so remove 5 (5 * 4 = 20), but we may at most remove 4 to keep us > 0.
    us -= 4; // = 2 cycles

  #else
    // for the 1 MHz internal clock (default settings for common AVR microcontrollers)
    // the overhead of the function calls is 14 (16) cycles
    if (us <= 16) return; //= 3 cycles, (4 when true)
    if (us <= 25) return; //= 3 cycles, (4 when true), (must be at least 25 if we want to subtract 22)

    // compensate for the time taken by the preceding and next commands (about 22 cycles)
    us -= 22; // = 2 cycles
    // the following loop takes 4 microseconds (4 cycles)
    // per iteration, so execute it us/4 times
    // us is at least 4, divided by 4 gives us 1 (no zero delay bug)
    us >>= 2; // us div 4, = 4 cycles
  #endif

  // busy wait
  __asm__ __volatile__ (
    "1: sbiw %0,1" "\n\t" // 2 cycles
        _MORENOP_         // more cycles according to definition
    "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
  );
  // return = 4 cycles
}

// This clears up the timer settings, and then calls the tone timer initialization function (unless it's been disabled - but in this case, whatever called this isn't working anyway!
void initToneTimer(void)
{
  // Ensure the timer is in the same state as power-up
  #if defined(__AVR_ATtiny43__)
    TIMSK1 = 0;
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 0;
    OCR1B = 0;
    TIFR1 = 0x07;
  #elif (TIMER_TO_USE_FOR_TONE == 0)
    // Just zero the registers out, instead of trying to name all the bits, as there are combinations of hardware and settings where that doesn't work
    TCCR0B = 0; //  (0<<FOC0A) | (0<<FOC0B) | (0<<WGM02) | (0<<CS02) | (0<<CS01) | (0<<CS00);
    TCCR0A = 0; // (0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (0<<WGM01) | (0<<WGM00);
    // Reset the count to zero
    TCNT0 = 0;
    // Set the output compare registers to zero
    OCR0A = 0;
    OCR0B = 0;
    #if defined(TIMSK)
      // Disable all Timer0 interrupts
      // Clear the Timer0 interrupt flags
      #if defined(TICIE0) // x61-series has an additional input capture interrupt vector...
        TIMSK &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0)|(1<<TICIE0));
        TIFR = ((1<<OCF0B) | (1<<OCF0A) | (1<<TOV0)|(1<<ICF0));
      #else
        TIMSK &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0));
        TIFR = ((1<<OCF0B) | (1<<OCF0A) | (1<<TOV0));
      #endif
    #elif defined(TIMSK0)
      // Disable all Timer0 interrupts
      TIMSK0 = 0; //can do this because all of TIMSK0 is timer 0 interrupt masks
      // Clear the Timer0 interrupt flags
      TIFR0 = ((1<<OCF0B) | (1<<OCF0A) | (1<<TOV0)); //no ICF0 interrupt on any supported part with TIMSK0
    #endif
  #elif (TIMER_TO_USE_FOR_TONE == 1) && defined(TCCR1)
    // Turn off Clear on Compare Match, turn off PWM A, disconnect the timer from the output pin, stop the clock
    TCCR1 = (0<<CTC1) | (0<<PWM1A) | (0<<COM1A1) | (0<<COM1A0) | (0<<CS13) | (0<<CS12) | (0<<CS11) | (0<<CS10);
    // 0 out TCCR1
    // Turn off PWM A, disconnect the timer from the output pin, no Force Output Compare Match, no Prescaler Reset
    GTCCR &= ~((1<<PWM1B) | (1<<COM1B1) | (1<<COM1B0) | (1<<FOC1B) | (1<<FOC1A) | (1<<PSR1));
    // Reset the count to zero
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A = 0;
    OCR1B = 0;
    OCR1C = 0;
    // Disable all Timer1 interrupts
    TIMSK &= ~((1<<OCIE1A) | (1<<OCIE1B) | (1<<TOIE1));
    // Clear the Timer1 interrupt flags
    TIFR = ((1<<OCF1A) | (1<<OCF1B) | (1<<TOV1));
  #elif (TIMER_TO_USE_FOR_TONE == 1) && defined(TCCR1E)
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1C = 0;
    TCCR1D = 0;
    TCCR1E = 0;
    // Reset the count to zero
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A = 0;
    OCR1B = 0;
    OCR1C = 0;
    OCR1D = 0;
    // Disable all Timer1 interrupts
    TIMSK &= ~((1<<TOIE1) | (1<<OCIE1A) | (1<<OCIE1B) | (1<<OCIE1D));
    // Clear the Timer1 interrupt flags
    TIFR = ((1<<TOV1) | (1<<OCF1A) | (1<<OCF1B) | (1<<OCF1D));
  #elif (TIMER_TO_USE_FOR_TONE == 1)
    // Normal, well-behaved 16-bit Timer 1.
    // Turn off Input Capture Noise Canceler, Input Capture Edge Select on Falling, stop the clock
    TCCR1B = (0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
    // TCCR1B=0; But above is compile time known, so optimized out, and will fail if
    // Disconnect the timer from the output pins, Set Waveform Generation Mode to Normal
    TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
    // TCCR1A = 0, same logic as above
    // Reset the count to zero
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A = 0;
    OCR1B = 0;
    // Disable all Timer1 interrupts
    #if defined(TIMSK)
    TIMSK &= ~((1<<TOIE1) | (1<<OCIE1A) | (1<<OCIE1B) | (1<<ICIE1));
    // Clear the Timer1 interrupt flags
    TIFR = ((1<<TOV1) | (1<<OCF1A) | (1<<OCF1B) | (1<<ICF1));
    #elif defined(TIMSK1)
    // Disable all Timer1 interrupts
    TIMSK1 = 0; //~((1<<TOIE1) | (1<<OCIE1A) | (1<<OCIE1B) | (1<<ICIE1));
    // Clear the Timer1 interrupt flags
    TIFR1 = ((1<<TOV1) | (1<<OCF1A) | (1<<OCF1B) | (1<<ICF1));
    #endif
  #endif

  #if INITIALIZE_SECONDARY_TIMERS
  // Prepare the timer for PWM
    initToneTimerInternal();
  #endif
}

// initToneTimerInternal() - initialize the timer used for tone for PWM

#if INITIALIZE_SECONDARY_TIMERS
  static void initToneTimerInternal(void)
  {
    // Timer is processor clock divided by ToneTimer_Prescale_Index
    #if (TIMER_TO_USE_FOR_TONE == 0)
      // Use the Tone Timer for phase correct PWM
      TCCR0A = (1<<WGM00) | (0<<WGM01);
      TCCR0B = (ToneTimer_Prescale_Index << CS00) | (0<<WGM02);
    #elif defined(__AVR_ATtiny43__)
      TCCR1A = 3; //WGM 10=1, WGM11=1
      TCCR1B = 3; //prescaler of 64
    #elif (TIMER_TO_USE_FOR_TONE == 1) && defined(TCCR1) // ATtiny x5
      // Use the Tone Timer for fast PWM as phase correct not supported by this timer
      GTCCR = (1<<PWM1B);
      OCR1C = 0xFF; //Use 255 as the top to match with the others as this module doesn't have a 8bit PWM mode.
      TCCR1 = (1<<CTC1) | (1<<PWM1A) | (ToneTimer_Prescale_Index << CS10);
    #elif (TIMER_TO_USE_FOR_TONE == 1) && defined(TCCR1E) // ATtiny x61
      // Use the Tone Timer for phase correct PWM
      TCCR1A = (1<<PWM1A) | (1<<PWM1B);
      TCCR1C = (1<<PWM1D);
      TCCR1D = (1<<WGM10) | (0<<WGM11);
      TCCR1B = (ToneTimer_Prescale_Index << CS10);
    #elif (TIMER_TO_USE_FOR_TONE == 1 ) && defined(__AVR_ATtinyX7__)
      TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM10);
      TCCR1B = (ToneTimer_Prescale_Index << CS10);
    #elif (TIMER_TO_USE_FOR_TONE == 1) // x4, x8, x313,
      // Use the Tone Timer for phase correct PWM
      TCCR1A = (1<<WGM10);
      TCCR1B = (0<<WGM12) | (0<<WGM13) | (ToneTimer_Prescale_Index << CS10); //set the clock
    #endif
  }
#endif


uint8_t read_factory_calibration(void)
{
  uint8_t value = boot_signature_byte_get(1);
  return value;
}

void init(void)
{
  /*
  If clocked from the PLL (CLOCK_SOURCE==6) then there are three special cases all involving
  the 16.5 MHz clock option used to support VUSB on PLL-clocked parts.
  If F_CPU is set to 16.5, and the Micronucleus bootloader is in use (indicated by BOOTTUNED165
  being defined), the bootloader has already set OSCCAL to run at 16.5; if that is set while
  F_CPU is set to 16, we reload that from factory.
  If it is set to 16.5 but BOOTTUNED165 is not set, we're not using the micronucleus bootloader
  and we need to increase OSCCAL; this is a guess, but works better than nothing.
  */

  #if (CLOCK_SOURCE==6) //handle weird frequencies when using the PLL clock
    #if (defined(BOOTTUNED165)) // If it's a micronucleus board, it will either run at 16.5 after
      // adjusting the internal oscillator for that speed (in which case we are done) or we want
      // it to be set to run at 16, in which case we need to reload the factory cal.
      #if (F_CPU!=16500000L) //if not 16.5, it's 16, or that divided by power of two
        OSCCAL = read_factory_calibration(); //we do this if it was tuned by micronucleus, but we don't want USB.
        #if (F_CPU!=16000000) // 16MHz is speed of unprescaled PLL clock - if we don't want that...
          #ifdef CCP
            CCP=0xD8; //enable change of protected register
          #else
            CLKPR=1<<CLKPCE; //enable change of protected register
          #endif
          #if (F_CPU ==8000000L)
            CLKPR=1; //prescale by 2 for 8MHz
          #elif (F_CPU==4000000L)
            CLKPR=2; //prescale by 4 for 4MHz
          #elif (F_CPU==2000000L)
            CLKPR=3; //prescale by 8 for 2MHz
          #elif (F_CPU==1000000L)
            CLKPR=4; //prescale by 16 for 1MHz
          #elif (F_CPU ==500000L)
            CLKPR=5; //prescale by 32 for 0.5MHz
          #elif (F_CPU ==250000L) // these extremely slow speeds are of questionable value.
            CLKPR=6; //prescale by 64 for 0.25MHz
          #elif (F_CPU ==125000L) //
            CLKPR=7; //prescale by 128 for 125kHz
          #elif (F_CPU ==62500L)
            CLKPR=8; //prescale by 256 for 62.50kHz
          #else
            #error "Frequency requested from PLL that cannot be generated by prescaling"
            #error "Custom tuning is not supported in the current version of ATTinyCore"
          #endif // end check for each prescale freq
        #endif //end if not 16
      #endif // end if not 16.5
    #elif (F_CPU == 16500000L) // not pretuned to 16.5 (no micronucleus), but 16.5 requested, presumably for VUSB.
      if (OSCCAL == read_factory_calibration()) {
        // adjust the calibration up from 16.0mhz to 16.5mhz
        if (OSCCAL >= 128) {
          // maybe 8 is better? oh well - only about 0.3% out anyway
          OSCCAL += 7;
        } else {
          OSCCAL += 5;
        }
      }
    #else // We're using PLL, and it's not a VUSB special case.
      #if (F_CPU!=16000000) // 16MHz is speed of unprescaled PLL clock.
        #ifdef CCP
          CCP=0xD8; //enable change of protected register
        #else
          CLKPR=1<<CLKPCE; //enable change of protected register
        #endif
        // One really wonders why someone would use the PLL as clock source if they weren't using VUSB or running at 16MHz, it's got to burn more power...
        #if (F_CPU ==8000000L)
          CLKPR=1; //prescale by 2 for 8MHz
        #elif (F_CPU==4000000L)
          CLKPR=2; //prescale by 4 for 4MHz
        #elif (F_CPU==2000000L)
          CLKPR=3; //prescale by 8 for 2MHz
        #elif (F_CPU==1000000L)
          CLKPR=4; //prescale by 16 for 1MHz
        #elif (F_CPU ==500000L)
          CLKPR=5; //prescale by 32 for 0.5MHz
        #elif (F_CPU ==250000L) // these extremely slow speeds are of questionable value.
          CLKPR=6; //prescale by 64 for 0.25MHz
        #elif (F_CPU ==125000L) //
          CLKPR=7; //prescale by 128 for 125kHz
        #elif (F_CPU ==62500L)
          CLKPR=8; //prescale by 256 for 62.50kHz
        #else
          #error "Frequency requested from PLL that cannot be generated by prescaling"
          #error "Custom tuning is not supported in the current version of ATTinyCore"
        #endif //end handling of individual frequencies
      #endif //end if not 16 MHz (default speed)
    #endif // end check for VUSB-related special cases
  #elif (CLOCK_SOURCE==0 && F_CPU!=8000000L && F_CPU!=1000000L)
    // normal oscillator, we want a setting that fuses won't give us,
    // so need to set prescale.
    #ifdef CCP
      CCP=0xD8; //enable change of protected register
    #else
      CLKPR=1<<CLKPCE; //enable change of protected register
    #endif
    #if (F_CPU ==4000000L)
      CLKPR=1; //prescale by 2 for 4MHz
    #elif (F_CPU ==2000000L)
      CLKPR=2; //prescale by 4 for 2MHz
      // 1 MHz handled by fuse
    #elif (F_CPU ==500000L) // these extremely slow speeds are of questionable value.
      CLKPR=4; //prescale by 16 for 0.5MHz
    #elif (F_CPU ==250000L)
      CLKPR=5; //prescale by 32 for 0.25MHz
    #elif (F_CPU ==125000L) //
      CLKPR=6; //prescale by 64 for 125kHz
    #elif (F_CPU ==62500L)
      CLKPR=7; //prescale by 128 for 62.50kHz
    #elif (F_CPU ==31250L)
      CLKPR=8; //prescale by 256 for 31.25kHz
    #else
      #error "Frequency requested from internal oscillator that cannot be generated by prescaling"
      #error "Custom tuning is not supported in the current version of ATTinyCore"
    #endif
  #elif(CLOCK_SOURCE==17 || CLOCK_SOURCE==18) // external 16MHz CLOCK or Crystal, but maybe they want to go slower to save power...
    #if (F_CPU!=16000000) // 16MHz is speed of external clock on these
      #ifdef CCP
        CCP=0xD8; //enable change of protected register
      #else
        CLKPR=1<<CLKPCE; //enable change of protected register
      #endif
      #if (F_CPU ==8000000L)
        CLKPR=1; //prescale by 2 for 8MHz
      #elif (F_CPU==4000000L)
        CLKPR=2; //prescale by 4 for 4MHz
      #elif (F_CPU==2000000L)
        CLKPR=3; //prescale by 8 for 2MHz
      #elif (F_CPU==1000000L)
        CLKPR=4; //prescale by 16 for 1MHz
      #elif (F_CPU ==500000L)
        CLKPR=5; //prescale by 32 for 0.5MHz
      #elif (F_CPU ==250000L) // these extremely slow speeds are of questionable value.
        CLKPR=6; //prescale by 64 for 0.25MHz
      #elif (F_CPU ==125000L) //
        CLKPR=7; //prescale by 128 for 125kHz
      #elif (F_CPU ==62500L)
        CLKPR=8; //prescale by 256 for 62.50kHz
      #else
        #error "Frequency requested from 16MHz external clock that cannot be generated by prescaling"
      #endif //end handling of individual frequencies
    #endif //end if not 16 MHz (default speed)
  #endif //end handling for the two types of internal oscillator derived clock source and 16MHz ext clock of MH-ET tiny88


  #if defined(PLLTIMER1) && (!defined(PLLCSR))
    #error "Chip does not have PLL (only x5, x61 series do), yet you somehow selected PLL as timer1 source. If you have not modified the core, please report this to core maintainer."
  #endif
  #ifdef PLLTIMER1 // option on x5 and x61
    if (!PLLCSR) {
      PLLCSR = (1<<PLLE);
        while (!(PLLCSR&1)) {
          ; //wait for lock
        }
      PLLCSR = (1<<PCKE)|(1<<PLLE);
    }
  #endif
  #if defined(LOWPLLTIMER1) && ((CLOCK_SOURCE==6) || (!defined(PLLCSR)))
    #error "LOW SPEED PLL Timer1 clock source is NOT SUPPORTED when PLL is used as system clock source; the bit to enable it cannot be set, per datasheet (section Timer/Counter1->Register Descriotion->PLLCSR, x5 / x61 only) or chip does not have PLL"
  #endif
  #ifdef LOWPLLTIMER1 // option on x5 and x61
    if (!PLLCSR) {
      PLLCSR = (1<<LSM) | (1<<PLLE);
      while (!(PLLCSR&1)) {
        ; //wait for lock
      }
      // faster than |= since we know the value we want (OUT vs )
      PLLCSR = (1<<PCKE)|(1<<LSM)|(1<<PLLE);
    }
  #endif
  // Initialize the timer used for Tone
  #if INITIALIZE_SECONDARY_TIMERS
    initToneTimerInternal();
  #endif

  // Initialize the ADC
  #if defined( INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER ) && INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER
    #if defined(ADCSRA)
      // set a2d prescale factor
      // ADCSRA = (ADCSRA & ~((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0))) | (ADC_ARDUINO_PRESCALER << ADPS0) | (1<<ADEN);
      // dude, this is being called on startup. We know that ADCSRA is 0! Why add a RMW cycle?!
      ADCSRA = (ADC_ARDUINO_PRESCALER << ADPS0) | (1<<ADEN);
      // enable a2d conversions
      // sbi(ADCSRA, ADEN); //we already set this!!!
    #endif
  #endif
}

/* wiring.c - Timing and initialization
 * Part of ATTinyCore - github.com/SpenceKonde/ATTinyCore
 *   (c) 2005-2006 David A. Mellis as part of Arduino
 *   Modified  28-08-2009 for attiny84 R.Wiersma
 *   Modified  14-10-2009 for attiny45 Saposoft
 *   Modified 17-05-2010 - B.Cook Rewritten to use the various Veneers.
 *   Modified extensively 2016-2021 Spence Konde for ATTinyCore
 *   Modified 2026 by MCUdude for TinyCore
 *   Free Software - LGPL 2.1, please see LICENCE.md for details
 *---------------------------------------------------------------------------*/


#include "wiring_private.h"
#ifndef __AVR_ATtiny26__
#include <avr/boot.h>
#endif
#if USING_BOOTLOADER
  #include <avr/pgmspace.h>
#else
  #include <avr/eeprom.h>
#endif

#ifdef OSCCAL0
  #define OSC OSCCAL0
#else
  #define OSC OSCCAL
#endif

#if INITIALIZE_SECONDARY_TIMERS
static void initToneTimerInternal(void);
#endif
static void __empty() {
    // Empty
  }
void yield(void) __attribute__ ((weak, alias("__empty")));

void delay(unsigned int count)
{
  do {
    yield();
    _delay_loop_2((uint16_t)(F_CPU / 1000 / 4));
  } while (--count);
}


void init_clock() {
  #if (CLOCK_SOURCE == 6) // Internal 16 MHz PLL
      #if (F_CPU == 16000000L || F_CPU == 8000000L || F_CPU == 4000000L || F_CPU == 2000000L || F_CPU == 1000000L || F_CPU == 500000L)
        // 16MHz is speed of unprescaled PLL clock.
        #if (F_CPU != 16000000)
          #ifdef CCP
            CCP = 0xD8; // Wnable change of protected register
          #else
            CLKPR = (1 << CLKPCE); // Wnable change of protected register
          #endif
          #if (F_CPU == 8000000L)
            CLKPR = 1; // Prescale by 2 for 8MHz
          #elif (F_CPU == 4000000L)
            CLKPR = 2; // Prescale by 4 for 4MHz
          #elif (F_CPU ==  2000000L)  // could also be done with fuses, but this is such a weird situation,
            CLKPR = 3; // Prescale by 8 for 2MHz
          #elif (F_CPU == 1000000L)
            CLKPR = 4; // Prescale by 16 for 1MHz
          #elif (F_CPU == 500000L)
            CLKPR = 5; // Prescale by 32 for 0.5MHz
          #else
            CLKPR = 8; // Prescale by 256 for 62.50kHz
          #endif  // End of checking F_CPUs
        #endif // End not 16 MHz
      #endif // End if not 16 MHz derived
  /* End of PLL prescale */
  #elif (CLOCK_SOURCE == 0) // System clock is internal
    /* Start internal osc prescale and tuning */
    #if (F_CPU == 8000000L || F_CPU == 4000000L || F_CPU == 2000000L || F_CPU == 1000000L || F_CPU == 500000L)
      #if defined(LOWERCAL)
        OSC -= LOWERCAL;
      #endif
      // Apply prescaling to get desired frequency if not set by fuses
      #if (F_CPU != 8000000L && F_CPU != 1000000L)
        // Normal oscillator, and we want a setting that fuses won't give us, so need to set prescale.
        #ifdef CCP
          CCP = 0xD8; // Enable change of protected register
        #else
          CLKPR = 1 << CLKPCE; // Enable change of protected register
        #endif
        #if (F_CPU == 4000000L)
          CLKPR = 1; // Prescale by 2 for 4MHz
        #elif (F_CPU == 2000000L)
          CLKPR = 2; // Prescale by 4 for 2MHz
        #elif (F_CPU == 500000L)
          CLKPR = 4; // Prescale by 16 for 0.5MHz
        #else
          CLKPR = 8; // Prescale by 256 for 31.25kHz
        #endif
      #endif // End if not 1/8 MHz
    #else
      CLKPR= 1 << CLKPCE; // Enable change of protected register
    #endif
    /* End of internal osc prescale and tuning */
  #endif // End handling for the two types of internal oscillator derived clock source
}


void init() {
  // this needs to be called before setup() or some functions won't work there
  sei();

  init_clock(); // initialize the main system clock
  #ifdef SET_REMAP
    REMAP = SET_REMAP;
  #endif
  #ifdef SET_REMAPUSI
    USIPP = 1;
  #endif
  // Initialize the timer used for Tone
  #if INITIALIZE_SECONDARY_TIMERS
    initToneTimerInternal();
  #endif
  // Initialize the ADC
  #if defined(INITIALIZE_ADC) && INITIALIZE_ADC
    #if defined(ADCSRA)
      // set a2d prescale factor
      // ADCSRA = (ADCSRA & ~((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))) | (ADC_ARDUINO_PRESCALER << ADPS0) | (1 << ADEN);
      // dude, this is being called on startup. We know that ADCSRA is 0! Why add a RMW cycle?!
      ADCSRA = (ADC_ARDUINO_PRESCALER << ADPS0) | (1 << ADEN);
      // enable a2d conversions
      // sbi(ADCSRA, ADEN); //we already set this!!!
    #endif
  #endif
}


// This clears up the timer settings, and then calls the tone timer initialization function (unless it's been disabled - but in this case, whatever called this isn't working anyway!
// Note that this is used **only** when directly called - and serves only to reset the timers to stock state. The initToneTimerInternal() does the other half, and is called by this
// 2023 - commented out a few register writes in here because we then rewrite the same registers a few moments later.
void initToneTimer(void) {
  // Ensure the timer is in the same state as power-up, except for any registers we write over with straight assignment in initToneTimerInternal() other than
  // TCCRnB and TIMSK/TIMSKn, which are zero'ed to turn off interrupts and stop the timer.
  #if defined(__AVR_ATtiny43__)   // Always uses TC0 for millis and TC1 for Tone(), both are 8-bit
    TIMSK1 = 0;
    TCCR1B = 0;
    //TCCR1A = 0;
    TCNT1  = 0;
    OCR1A  = 0;
    OCR1B  = 0;
    TIFR1  = 0x07;
  #elif defined(__AVR_ATtiny26__) // Always uses low speed timer.
    TIMSK  &= 2;
    //TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A  = 0;
    OCR1B  = 0;
    TIFR   = 0x66;
  #elif (TIMER_TO_USE_FOR_TONE == 0)
    #if !defined(DISABLE_MILLIS) && defined(TIMER_TO_USE_FOR_TONE) && (!defined(TIMER_TO_USE_FOR_MILLIS) || TIMER_TO_USE_FOR_MILLIS == 0 )
      /* Policy: If we know some combination of options will result in behavior that is not plausaibly intended behavior, and we are in a position
       * to recognize this at compile time, we should do so and error out; this helps compensate for poor debuggability of the tinyAVR parts without
       * uncommon programming hardware and official Microchip software. */
      #error "Core defect - Please report this issue along with the settings that generated it"
    #endif
    // Just zero the registers out, instead of trying to name all the bits, as there are combinations of hardware and settings where that doesn't work
    TCCR0B = 0; //  (0 << FOC0A) | (0 << FOC0B) | (0 << WGM02) | (0 << CS02) | (0 << CS01) | (0 << CS00);
    //TCCR0A = 0; // (0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (0 << WGM01) | (0 << WGM00);
    // Reset the count to zero
    TCNT0 = 0;
    // Set the output compare registers to zero
    OCR0A = 0;
    OCR0B = 0;
    #if defined(TIMSK)
      // Disable all Timer0 interrupts
      // Clear the Timer0 interrupt flags
      #if defined(TICIE0) // x61-series has an additional input capture interrupt vector...
        TIMSK &= ~((1 << OCIE0B) | (1 << OCIE0A) | (1 << TOIE0) | (1 << TICIE0));
        TIFR = ((1 << OCF0B) | (1 << OCF0A) | (1 << TOV0) | (1 << ICF0));
      #else
        TIMSK &= ~((1 << OCIE0B) | (1 << OCIE0A) | (1 << TOIE0));
        TIFR = ((1 << OCF0B) | (1 << OCF0A) | (1 << TOV0));
      #endif
    #elif defined(TIMSK0)
      // Disable all Timer0 interrupts
      TIMSK0 = 0; //can do this because all of TIMSK0 is timer 0 interrupt masks
      // Clear the Timer0 interrupt flags
      TIFR0 = ((1 << OCF0B) | (1 << OCF0A) | (1 << TOV0)); //no ICF0 interrupt on any supported part with TIMSK0
    #endif
  #elif (TIMER_TO_USE_FOR_TONE == 1) && defined(TCCR1) // t85
    // Disable all Timer1 interrupts
    TIMSK &= ~((1 << OCIE1A) | (1 << OCIE1B) | (1 << TOIE1));
    // Turn off Clear on Compare Match, turn off PWM A, disconnect the timer from the output pin, stop the clock
    TCCR1 = 0;
    // 0 out TCCR1
    // Turn off PWM A, disconnect the timer from the output pin, no Force Output Compare Match, no Prescaler Reset
    // GTCCR = GTCCR & (~((1 << PWM1B) | (1 << COM1B1) | (1 << COM1B0) | (1 << FOC1B) | (1 << FOC1A) | (1 << PSR1)));
    //  set in initToneTimerInternal - and hardly any of these values are ever used.

    // Reset the count to zero
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A = 0;
    OCR1B = 0;
    OCR1C = 0xFF;
    // Clear the Timer1 interrupt flags
    TIFR = ((1 << OCF1A) | (1 << OCF1B) | (1 << TOV1));
  #elif (TIMER_TO_USE_FOR_TONE == 1) && defined(TCCR1E) // x61
    // turn off all interrupts
    TIMSK &= ~((1 << TOIE1) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << OCIE1D));
    // TCCR1A = 0; set in initToneTimerInternal
    TCCR1B = 0;
    //TCCR1C = 0;  set in initToneTimerInternal
    //TCCR1D = 0;  set in initToneTimerInternal
    TCCR1E = 0;
    // Reset the count to zero
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A  = 0;
    OCR1B  = 0;
    OCR1C  = 0xFF;
    OCR1D  = 0;
    // Clear the Timer1 interrupt flags
    TIFR   =  ((1 <<  TOV1) | (1 <<  OCF1A) | (1 <<  OCF1B) | (1 <<  OCF1D));

  #elif (TIMER_TO_USE_FOR_TONE == 1)
    // Normal, well-behaved 16-bit Timer 1.
    // Turn off Input Capture Noise Canceler, Input Capture Edge Select on Falling, stop the clock
    TCCR1B = 0;
    // Disconnect the timer from the output pins, Set Waveform Generation Mode to Normal
    TCCR1A = 0;
    TCNT1 = 0;
    // Set the output compare registers to zero
    OCR1A = 0;
    OCR1B = 0;
    // Disable all Timer1 interrupts
    #if defined(TIMSK)
      TIMSK &= ~((1 << TOIE1) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << ICIE1));
      // Clear the Timer1 interrupt flags
      TIFR = ((1 << TOV1) | (1 << OCF1A) | (1 << OCF1B) | (1 << ICF1));
    #elif defined(TIMSK1)
      // Disable all Timer1 interrupts
      TIMSK1 = 0; //~((1 << TOIE1) | (1 << OCIE1A) | (1 << OCIE1B) | (1 << ICIE1));
      // Clear the Timer1 interrupt flags
      TIFR1 = ((1 << TOV1) | (1 << OCF1A) | (1 << OCF1B) | (1 << ICF1));
    #endif
  #endif
  #if INITIALIZE_SECONDARY_TIMERS
  // Prepare the timer for PWM
    initToneTimerInternal();
  #endif
}

// initToneTimerInternal() - initialize the timer used for tone for PWM

#if INITIALIZE_SECONDARY_TIMERS
  static void initToneTimerInternal(void) {
    #if defined(TIMER_TO_USE_FOR_TONE)
      #if (TIMER_TO_USE_FOR_TONE == 0)
        #warning "ATTinyCore only supports using Timer1 for tone - this is untested code!"
        // Use the Tone Timer for phase correct PWM
        TCCR0A = (1 << WGM00);
        TCCR0B = (ToneTimer_Prescale_Index << CS00);
      #endif
    #else
      #warning "Core defect: TIMER_TO_USE_FOR_TONE undefined! Please report along with what part and settings you used"
      #define TIMER_TO_USE_FOR_TONE (1)
    #endif
    /* Now the case we expect to work, TC1 */
    #if (TIMER_TO_USE_FOR_TONE == 1)
      #if defined(__AVR_ATtinyX41__)

        TCCR1A   = (1 << WGM10) | (1 << COM1A1)| (1 << COM1B1); // enable OC1A, OC1B
        TCCR2A   = (1 << WGM20) | (1 << COM2A1)| (1 << COM2B1); // enable OC2A, OC2B
        TOCPMSA0 = 0b00010000; // PA4: OC0A, PA3: OC1B, PA2: N/A,  PA1: N/A
        TOCPMSA1 = 0b10100100; // PB2: OC2A, PA7: OC2B, PA6: OC1A, PA5: OC0B
        #if !defined(TIMER1_USE_FAST_PWM)
          TCCR1B   = (ToneTimer_Prescale_Index << CS10); // set the clock - do this last, always!
          TCCR2B   = (ToneTimer_Prescale_Index << CS10); // set the clock - cause it starts the timer!
        #else
          TCCR1B   = (1 << WGM12) | (ToneTimer_Prescale_Index << CS10); // set the clock - do this last, always!
          TCCR2B   = (1 << WGM22) | (ToneTimer_Prescale_Index << CS10); // set the clock - cause it starts the timer!
        #endif
        // TOCPMCOE = 0; // keep these disabled!
      #elif defined(__AVR_ATtiny828__)
        TCCR1A   = (1 << WGM10) | (1 << COM1A1)| (1 << COM1B1); // enable OC1A, OC1B
        TOCPMSA0 = (0b11100100);  // PC3: OC1B, PC2: OC1A, PC1: OC0B, PC0 OC0A.
        TOCPMSA1 = (0b11001001);  // PC7: OC1B, PC6: OC0A, PC5: OC1A, PC4,OC0B
        // TOCPMCOE = 0; // keep these disabled!
        #if !defined(TIMER0_USE_FAST_PWM)
          TCCR1B   =(ToneTimer_Prescale_Index << CS10); // set the clock
        #else
          TCCR1B   =(1 << WGM12) | (ToneTimer_Prescale_Index << CS10); // set the clock
        #endif
      #elif defined(__AVR_ATtiny43__)
        #if !defined(TIMER0_USE_FAST_PWM)
          TCCR1A = 1; //WGM 10=1, WGM11=1 // Phase correct
        #else
          TCCR1A = 3; //WGM 10=1, WGM11=1 // Fast
        #endif
        TCCR1B = (ToneTimer_Prescale_Index << CS10);
      #elif defined(TCCR1) // ATtiny x5
        // Use the Tone Timer for fast PWM as phase correct not supported by this timer
        GTCCR = (1 << PWM1B);
        TCCR1 = (1 << CTC1) | (1 << PWM1A) | (ToneTimer_Prescale_Index << CS10);
        /* Fast mode the only option here! */
      #elif defined(TCCR1E) // ATtiny x61
        // Use the Tone Timer for phase correct PWM
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
      #else // x4, x8, x313,
        // Use the Tone Timer for PWM
          TCCR1A = (1 << WGM10);
        #if !defined(TIMER1_USE_FAST_PWM)
          TCCR1B = (ToneTimer_Prescale_Index << CS10); //set the clock
        #else
          TCCR1B = (1 << WGM12) | (ToneTimer_Prescale_Index << CS10); //set the clock
        #endif
      #endif
    #else
      #error "The selected tone timer does not exist. "
    #endif
  }
#endif

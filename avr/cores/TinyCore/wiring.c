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

static void __empty() {
  }

void yield(void) __attribute__ ((weak, alias("__empty")));

void delay(unsigned int count) {
  do {
    yield();
    _delay_loop_2((uint16_t)(F_CPU / 1000 / 4));
  } while (--count);
}


void init_clock() {
  #if (CLOCK_SOURCE == 6) // Internal 16 MHz PLL
      #if (F_CPU == 16000000L || F_CPU == 8000000L || F_CPU == 4000000L || F_CPU == 2000000L || F_CPU == 1000000L || F_CPU == 500000L)
        // 16MHz is speed of unprescaled PLL clock.
        #if (F_CPU != 16000000L)
          #if defined(CCP)
            CCP = 0xD8; // Wnable change of protected register
          #elif defined(CLKPR)
            CLKPR = (1 << CLKPCE); // Wnable change of protected register
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
          #endif
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
        #if defined(CCP)
          CCP = 0xD8; // Enable change of protected register
        #elif defined(CLKPR)
          CLKPR = (1 << CLKPCE); // Enable change of protected register
          #if (F_CPU == 4000000L)
            CLKPR = 1; // Prescale by 2 for 4MHz
          #elif (F_CPU == 2000000L)
            CLKPR = 2; // Prescale by 4 for 2MHz
          #elif (F_CPU == 500000L)
            CLKPR = 4; // Prescale by 16 for 0.5MHz
          #else
            CLKPR = 8; // Prescale by 256 for 31.25kHz
          #endif
        #endif
      #endif // End if not 1/8 MHz
    #endif
    /* End of internal osc prescale and tuning */
  #endif // End handling for the two types of internal oscillator derived clock source
}


void init() {
  sei();

  init_clock(); // Initialize the main system clock
}

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
  Modified 2015 for Attiny841/1634/828 and for uart clock support S. Konde
*/

#ifndef _NOP
  #define _NOP() do { __asm__ volatile ("nop"); } while (0)
#endif

#include "Arduino.h"
#include "core_build_options.h"
#include "core_adc.h"
#include "core_timers.h"
#include "wiring_private.h"
#include "ToneTimer.h"
#include <avr/boot.h>



static void __empty() {
  // Empty
}
void yield(void) __attribute__ ((weak, alias("__empty")));

void delay(unsigned int count)
{
  do {
    yield();
    _delay_loop_2(16 * F_CPU/1000/4);
  } while (--count);
}

static void initToneTimerInternal(void)
{
  // Stop the clock while we make changes
  ToneTimer_ClockSelect( ToneTimer_(Stopped) );

  // Set the timer to phase-correct PWM
  #if defined( TONETIMER_SUPPORTS_PHASE_CORRECT_PWM ) && TONETIMER_SUPPORTS_PHASE_CORRECT_PWM
    ToneTimer_SetWaveformGenerationMode( ToneTimer_(Phase_Correct_PWM_FF) );
  #else
    ToneTimer_SetWaveformGenerationMode( ToneTimer_(Fast_PWM_FF) );
  #endif

  // Timer is processor clock divided by ToneTimer_Prescale_Index (64)
  ToneTimer_ClockSelect( ToneTimer_Prescale_Index );
}

#if defined (__AVR_ATtinyX41__)
static void initTimer841(void)
{
  #if(TIMER_TO_USE_FOR_TONE==1)
  Timer2_ClockSelect(0);
  Timer2_SetWaveformGenerationMode(1);
  Timer2_ClockSelect(3);
  #else
  Timer1_ClockSelect(0);
  Timer1_SetWaveformGenerationMode(1);
  Timer1_ClockSelect(3);

  #endif

  TOCPMSA0=0b00010000;
  TOCPMSA1=0b10100100;
  TOCPMCOE=0b11111100;

}
#endif

void initToneTimer(void)
{
  // Ensure the timer is in the same state as power-up
  ToneTimer_SetToPowerup();

  #if defined( INITIALIZE_SECONDARY_TIMERS ) && INITIALIZE_SECONDARY_TIMERS
    // Prepare the timer for PWM
    initToneTimerInternal();
  #endif
}
#if ((F_CPU==16000000 || defined(LOWERCAL)) && CLOCK_SOURCE==0 )
  static uint8_t origOSC=0;

  uint8_t read_factory_calibration(void)
  {
    uint8_t SIGRD = 5; //Yes, this variable is needed. boot.h is looking for SIGRD but the io.h calls it RSIG... (unlike where this is needed in the other half of this core, at least the io.h file mentions it... ). Since it's actually a macro, not a function call, this works...
    uint8_t value = boot_signature_byte_get(1);
    return value;
  }
  void oscSlow(uint8_t newcal) {
    OSCCAL0=newcal;
    _NOP(); //this is all micronucleus does, and it seems to work fine...
  }

#endif


void init(void)
{
  #if (CLOCK_SOURCE==0 && defined(LOWERCAL))
    origOSC=read_factory_calibration();
    oscSlow(origOSC-LOWERCAL);
  #endif
  // this needs to be called before setup() or some functions won't work there
  #if (F_CPU==4000000L && CLOCK_SOURCE==0)
  //cli();
  #ifdef CCP
  CCP=0xD8; //enable change of protected register
  #else
  CLKPR=1<<CLKPCE; //enable change of protected register
  #endif
  CLKPR=1; //prescale by 2 for 4MHz
  #endif
  sei();

  // Initialize the timer used for Tone
  #if defined( INITIALIZE_SECONDARY_TIMERS ) && INITIALIZE_SECONDARY_TIMERS
    initToneTimerInternal();
    #if defined(__AVR_ATtinyX41__)
      initTimer841();
    #endif
    #if defined(__AVR_ATtiny828__)
      TOCPMSA0=0b00010000;
      TOCPMSA1=0b10100100;
      TOCPMCOE=0b11111100;
    #endif
  #endif



  // Initialize the ADC
  #if defined( INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER ) && INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER
    ADC_PrescalerSelect( ADC_ARDUINO_PRESCALER );
    ADC_Enable();
  #endif
}

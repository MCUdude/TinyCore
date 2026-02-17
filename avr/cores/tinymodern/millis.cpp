
#include <avr/interrupt.h>
#include "Arduino.h"
#include "core_timers.h"

#define MILLIS_ENABLED
#define MICROS_ENABLED

// The trick here is to move everything millis/micros related to aseparate file so it can be
// optimized away if millis() or micros() is nu used in the user application.
// And if millis() or micros() is present in the user application,
// init_millis() is actually ran _before_ main()!
void init_millis(void) __attribute__ ((naked)) __attribute__ ((used)) __attribute__ ((section (".init6")));

void init_millis(void)
{
  // Use the Millis Timer for fast PWM
  MillisTimer_SetWaveformGenerationMode( MillisTimer_(Fast_PWM_FF) );

  // Millis timer is always processor clock divided by MillisTimer_Prescale_Value (64)
  MillisTimer_ClockSelect( MillisTimer_Prescale_Index );

  // Enable the overflow interrupt (this is the basic system tic-toc for millis)
  MillisTimer_EnableOverflowInterrupt();
}

volatile unsigned long millis_timer_overflow_count = 0;
volatile unsigned long millis_timer_millis = 0;
static unsigned char millis_timer_fract = 0;

ISR(MILLISTIMER_OVF_vect)
{
  // copy these to local variables so they can be stored in registers
  // (volatile variables must be read from memory on every access)
  unsigned long m = millis_timer_millis;
  unsigned char f = millis_timer_fract;

/* rmv: The code below generates considerably less code (empty Sketch is 326 versus 304)...

  m += MILLIS_INC;
  f += FRACT_INC;
  if (f >= FRACT_MAX) {
    f -= FRACT_MAX;
    m += 1;
  }
...rmv */

  f += FRACT_INC;

  if (f >= FRACT_MAX)
  {
    f -= FRACT_MAX;
    m = m + MILLIS_INC + 1;
  }
  else
  {
    m += MILLIS_INC;
  }

  millis_timer_fract = f;
  millis_timer_millis = m;
  millis_timer_overflow_count++;
}

unsigned long millis()
{
  unsigned long m;
  uint8_t oldSREG = SREG;

  // disable interrupts while we read millis_timer_millis or we might get an
  // inconsistent value (e.g. in the middle of a write to millis_timer_millis)
  cli();
  m = millis_timer_millis;
  SREG = oldSREG;
  return m;
}
unsigned long micros()
{
  unsigned long m;
  uint8_t oldSREG = SREG, t;

  cli();
  m = millis_timer_overflow_count;
#if defined(TCNT0) && (TIMER_TO_USE_FOR_MILLIS == 0) && !defined(TCW0)
  t = TCNT0;
#elif defined(TCNT0L) && (TIMER_TO_USE_FOR_MILLIS == 0)
  t = TCNT0L;
#elif defined(TCNT1) && (TIMER_TO_USE_FOR_MILLIS == 1)
  t = TCNT1;
#elif defined(TCNT1L) && (TIMER_TO_USE_FOR_MILLIS == 1)
  t = TCNT1L;
#else
  #error Millis()/Micros() timer not defined
#endif

#if defined(TIFR0) && (TIMER_TO_USE_FOR_MILLIS == 0)
  if ((TIFR0 & _BV(TOV0)) && (t < 255))
    m++;
#elif defined(TIFR) && (TIMER_TO_USE_FOR_MILLIS == 0)
  if ((TIFR & _BV(TOV0)) && (t < 255))
    m++;
#elif defined(TIFR1) && (TIMER_TO_USE_FOR_MILLIS == 1)
  if ((TIFR1 & _BV(TOV1)) && (t < 255))
    m++;
#elif defined(TIFR) && (TIMER_TO_USE_FOR_MILLIS == 1)
  if ((TIFR & _BV(TOV1)) && (t < 255))
    m++;
#endif

  SREG = oldSREG;


#if F_CPU < 1000000L
  return ((m << 8) + t) * MillisTimer_Prescale_Value * (1000000L/F_CPU);
#else
#if (MillisTimer_Prescale_Value % clockCyclesPerMicrosecond() == 0 ) // Can we just do it the naive way? If so great!
  return ((m << 8) + t) * (MillisTimer_Prescale_Value / clockCyclesPerMicrosecond());
  // Otherwise we do clock-specific calculations
#elif (MillisTimer_Prescale_Value == 64 && F_CPU == 12800000L)  //64/12.8=5, but the compiler wouldn't realize it because of integer math - this is a supported speed for Micronucleus.
  return ((m << 8) + t) * 5;
  // Otherwise we do clock-specific calculations
#elif (MillisTimer_Prescale_Value == 64 && F_CPU == 24000000L) // 2.6875 vs real value 2.67
  m = (m << 8) + t;
  return (m<<1) + (m >> 1) + (m >> 3) + (m >> 4); // multiply by 2.6875
#elif (MillisTimer_Prescale_Value == 64 && clockCyclesPerMicrosecond() == 20) // 3.187 vs real value 3.2
  m=(m << 8) + t;
  return m+(m<<1)+(m>>2)-(m>>4);
#elif (MillisTimer_Prescale_Value == 64 && F_CPU == 18432000L) // 3.5 vs real value 3.47
  m=(m << 8) + t;
  return m+(m<<1)+(m>>1);
#elif (MillisTimer_Prescale_Value == 64 && F_CPU == 14745600L) //4.375  vs real value 4.34
  m=(m << 8) + t;
  return (m<<2)+(m>>1)-(m>>3);
#elif (MillisTimer_Prescale_Value == 64 && clockCyclesPerMicrosecond() == 14) //4.5 - actual 4.57 for 14.0mhz, 4.47 for the 14.3 crystals scrappable from everything
  m=(m << 8) + t;
  return (m<<2)+(m>>1);
#elif (MillisTimer_Prescale_Value == 64 && clockCyclesPerMicrosecond() == 12) // 5.3125 vs real value 5.333
  m=(m << 8) + t;
  return m+(m<<2)+(m>>2)+(m>>4);
#elif (MillisTimer_Prescale_Value == 64 && clockCyclesPerMicrosecond() == 11) // 5.75 vs real value 5.818 (11mhz) 5.78 (11.059)
  m=(m << 8) + t;
  return m+(m<<2)+(m>>1)+(m>>2);
#elif (MillisTimer_Prescale_Value == 64 && F_CPU==7372800L) // 8.625, vs real value 8.68
  m=(m << 8) + t;
  return (m<<3)+(m>>2)+(m>>3);
#elif (MillisTimer_Prescale_Value == 64 && F_CPU==6000000L) // 10.625, vs real value 10.67
  m=(m << 8) + t;
  return (m<<3)+(m<<1)+(m>>2)+(m>>3);
#elif (MillisTimer_Prescale_Value == 64 && clockCyclesPerMicrosecond() == 9) // For 9mhz, this is a little off, but for 9.21, it's very close!
  return ((m << 8) + t) * (MillisTimer_Prescale_Value / clockCyclesPerMicrosecond());
#else
  //return ((m << 8) + t) * (MillisTimer_Prescale_Value / clockCyclesPerMicrosecond());
  //return ((m << 8) + t) * MillisTimer_Prescale_Value / clockCyclesPerMicrosecond();
  //Integer division precludes the above technique.
  //so we have to get a bit more creative.
  //We can't just remove the parens, because then it will overflow (MillisTimer_Prescale_Value) times more often than unsigned longs should, so overflows would break everything.
  //So what we do here is:
  //the high part gets divided by cCPuS then multiplied by the prescaler. Then take the low 8 bits plus the high part modulo-cCPuS to correct for the division, then multiply that by the prescaler value first before dividing by cCPuS, and finally add the two together.
  //return ((m << 8 )/clockCyclesPerMicrosecond()* MillisTimer_Prescale_Value) + ((t+(((m<<8)%clockCyclesPerMicrosecond())) * MillisTimer_Prescale_Value / clockCyclesPerMicrosecond()));
  return ((m << 8 )/clockCyclesPerMicrosecond()* MillisTimer_Prescale_Value) + (t * MillisTimer_Prescale_Value / clockCyclesPerMicrosecond());
#endif
#endif
}

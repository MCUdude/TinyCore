#include <avr/interrupt.h>
#include "wiring_private.h"


#define MILLIS_ENABLED
#define MICROS_ENABLED

// The trick here is to move everything millis/micros related to aseparate file so it can be
// optimized away if millis() or micros() is nu used in the user application.
// And if millis() or micros() is present in the user application,
// init_millis() is actually ran _before_ main()!
void init_millis(void) __attribute__ ((naked)) __attribute__ ((used)) __attribute__ ((section (".init6")));

void init_millis(void)
{
/*
  // In case the bootloader left our millis timer in a bad way
  there had been some very dubious code here that reinitialized registers.
  ATTinyCore does not include any bootloaders that mess with timers without ensuring that they are reset.
  so this code has been removed.
*/

  // Use the Millis Timer for fast PWM (unless it doesn't have an output).
  #if (TIMER_TO_USE_FOR_MILLIS == 0)
    #if defined(WGM01) // if Timer0 has PWM
      TCCR0A = (1<<WGM01) | (1<<WGM00);
    #endif
    #if defined(TCCR0B) //The x61 has a wacky Timer0!
      TCCR0B = (MillisTimer_Prescale_Index << CS00);
    #else  //I think this means t's an x8
      TCCR0A = (MillisTimer_Prescale_Index << CS00);
    #endif
  #elif (TIMER_TO_USE_FOR_MILLIS == 1) && defined(TCCR1) //ATtiny x5
    TCCR1 = (1<<CTC1) | (1<<PWM1A) | (MillisTimer_Prescale_Index << CS10);
    GTCCR = (1<<PWM1B);
    OCR1C = 0xFF; //Use 255 as the top to match with the others as this module doesn't have a 8bit PWM mode.
  #elif (TIMER_TO_USE_FOR_MILLIS == 1) && defined(TCCR1E) //ATtiny x61
    TCCR1C = 1<<PWM1D;
    TCCR1B = (MillisTimer_Prescale_Index << CS10);
    TCCR1A = (1<<PWM1A) | (1<<PWM1B);
    //cbi(TCCR1E, WGM10); //fast pwm mode
    //cbi(TCCR1E, WGM11);
    OCR1C = 0xFF; //Use 255 as the top to match with the others as this module doesn't have a 8bit PWM mode.
  #elif (TIMER_TO_USE_FOR_MILLIS == 1)
    TCCR1A = 1<<WGM10;
    TCCR1B = (1<<WGM12) | (MillisTimer_Prescale_Index << CS10);
  #endif

  // this needs to be called before setup() or some functions won't work there
  sei();

    // Enable the overflow interrupt (this is the basic system tic-toc for millis)
    #if defined(TIMSK) && defined(TOIE0) && (TIMER_TO_USE_FOR_MILLIS == 0)
      sbi(TIMSK, TOIE0);
    #elif defined(TIMSK0) && defined(TOIE0) && (TIMER_TO_USE_FOR_MILLIS == 0)
      sbi(TIMSK0, TOIE0);
    #elif defined(TIMSK) && defined(TOIE1) && (TIMER_TO_USE_FOR_MILLIS == 1)
      sbi(TIMSK, TOIE1);
    #elif defined(TIMSK1) && defined(TOIE1) && (TIMER_TO_USE_FOR_MILLIS == 1)
      sbi(TIMSK1, TOIE1);
    #else
      #error Millis() Timer overflow interrupt not set correctly
    #endif
}

#ifndef CORRECT_EXACT_MICROS
  volatile unsigned long millis_timer_overflow_count = 0;
#endif
  volatile unsigned long millis_timer_millis = 0;
  volatile unsigned char millis_timer_fract = 0;
  #if (TIMER_TO_USE_FOR_MILLIS == 0)
    #if defined(TIMER0_OVF_vect)
      ISR(TIMER0_OVF_vect)
    #elif defined(TIM0_OVF_vect)
      ISR(TIM0_OVF_vect)
    #else
      #error "cannot find Millis() timer overflow vector"
    #endif
  #elif (TIMER_TO_USE_FOR_MILLIS == 1)
    #if defined(TIMER1_OVF_vect)
      ISR(TIMER1_OVF_vect)
    #elif defined(TIM1_OVF_vect)
      ISR(TIM1_OVF_vect)
    #else
      #error "cannot find Millis() timer overflow vector"
    #endif
  #else
    #error "Millis() timer not defined!"
  #endif
  {
    // copy these to local variables so they can be stored in registers
    // (volatile variables must be read from memory on every access)
    unsigned long m = millis_timer_millis;
    unsigned char f = millis_timer_fract;
#ifdef CORRECT_EXACT_MILLIS
    static unsigned char correct_exact = 0;     // rollover intended
    if (++correct_exact < CORRECT_EXACT_MANY) {
      ++f;
    }
#endif
    f += FRACT_INC;

    if (f >= FRACT_MAX)
    {
      f -= FRACT_MAX;
      m += MILLIS_INC + 1;
    }
    else
    {
      m += MILLIS_INC;
    }

    millis_timer_fract = f;
    millis_timer_millis = m;
#ifndef CORRECT_EXACT_MICROS
    millis_timer_overflow_count++;
#endif
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
#ifdef CORRECT_EXACT_MICROS
    unsigned int r; // needed for some frequencies, optimized away otherwise
    unsigned char f; // temporary storage for millis fraction counter
    unsigned char q = 0; // record whether an overflow is flagged
#endif
    unsigned long m;
    uint8_t t, oldSREG = SREG;

    cli();
#ifdef CORRECT_EXACT_MICROS
    m = millis_timer_millis;
    f = millis_timer_fract;
#else
    m = millis_timer_overflow_count;
#endif
  #if defined(TCNT0) && (TIMER_TO_USE_FOR_MILLIS == 0) && !defined(TCW0)
    t = TCNT0;
  #elif defined(TCNT0L) && (TIMER_TO_USE_FOR_MILLIS == 0)
    t = TCNT0L;
  #elif defined(TCNT1) && (TIMER_TO_USE_FOR_MILLIS == 1)
    t = TCNT1;
  #elif defined(TCNT1L) && (TIMER_TO_USE_FOR_MILLIS == 1)
    t = TCNT1L;
  #else
    #error "Millis()/Micros() timer not defined"
  #endif

  #if defined(TIFR0) && (TIMER_TO_USE_FOR_MILLIS == 0)
    if ((TIFR0 & _BV(TOV0)) && (t < 255))
    #ifndef CORRECT_EXACT_MICROS
      m++;
    #else
      q = 1;
    #endif
  #elif defined(TIFR) && (TIMER_TO_USE_FOR_MILLIS == 0)
    if ((TIFR & _BV(TOV0)) && (t < 255))
    #ifndef CORRECT_EXACT_MICROS
      m++;
    #else
      q = 1;
    #endif
  #elif defined(TIFR1) && (TIMER_TO_USE_FOR_MILLIS == 1)
    if ((TIFR1 & _BV(TOV1)) && (t < 255))
    #ifndef CORRECT_EXACT_MICROS
      m++;
    #else
      q = 1;
    #endif
  #elif defined(TIFR) && (TIMER_TO_USE_FOR_MILLIS == 1)
    if ((TIFR & _BV(TOV1)) && (t < 255))
    #ifndef CORRECT_EXACT_MICROS
      m++;
    #else
      q = 1;
    #endif
  #endif

    SREG = oldSREG;

  #ifdef CORRECT_EXACT_MICROS
    /* We convert milliseconds, fractional part and timer value
       into a microsecond value.  Relies on CORRECT_EXACT_MILLIS.
       Basically we multiply by 1000 and add the scaled timer.

       The leading part by m and f is long-term accurate.
       For the timer we just need to be close from below.
       Must never be too high, or micros jumps backwards. */
    m = (((m << 7) - (m << 1) - m + f) << 3) + ((
    #if   F_CPU == 24000000L || F_CPU == 12000000L || F_CPU == 6000000L // 1360, 680
        (r = ((unsigned int) t << 7) + ((unsigned int) t << 5), r + (r >> 4))
    #elif F_CPU == 22118400L || F_CPU == 11059200L // 1472, 736
        ((unsigned int) t << 8) - ((unsigned int) t << 6) - ((unsigned int) t << 3)
    #elif F_CPU == 20000000L || F_CPU == 10000000L // 816, 408
        (r = ((unsigned int) t << 8) - ((unsigned int) t << 6), r + (r >> 4))
    #elif F_CPU == 18432000L || F_CPU == 9216000L // 888, 444, etc.
        ((unsigned int) t << 8) - ((unsigned int) t << 5) - ((unsigned int) t << 1)
    #elif F_CPU == 18000000L // hand-tuned correction: 910
        (r = ((unsigned int) t << 8) - ((unsigned int) t << 5), r + (r >> 6))
    #elif F_CPU == 16500000L // hand-tuned correction: 992
        ((unsigned int) t << 8) - ((unsigned int) t << 3)
    #elif F_CPU == 14745600L || F_CPU == 7372800L || F_CPU == 3686400L // 1104, 552
        ((unsigned int) t << 7) + ((unsigned int) t << 3) + ((unsigned int) t << 1)
    #else // general catch-all
        (((((((((((((CORRECT_BIT7S
                     CORRECT_BIT6) << 1)
                     CORRECT_BIT5) << 1)
                     CORRECT_BIT4) << 1)
                     CORRECT_BIT3) << 1)
                     CORRECT_BIT2) << 1)
                     CORRECT_BIT1) << 1)
                     CORRECT_BIT0)
    #endif
      ) >> (8 - CORRECT_BITS));
    return q ? m + MICROSECONDS_PER_MILLIS_OVERFLOW : m;
  #else
  #if F_CPU < 1000000L
    return ((m << 8) + t) * MillisTimer_Prescale_Value * (1000000L/F_CPU);
  #else
    #if (MillisTimer_Prescale_Value % clockCyclesPerMicrosecond() == 0 && (F_CPU % 1000000 == 0 )) // Can we just do it the naive way? If so great!
      return ((m << 8) + t) * (MillisTimer_Prescale_Value / clockCyclesPerMicrosecond());
      // Otherwise we do clock-specific calculations
    #elif (MillisTimer_Prescale_Value == 64 && F_CPU == 12800000L) //64/12.8=5, but the compiler wouldn't realize it because of integer math - this is a supported speed for Micronucleus.
      return ((m << 8) + t) * 5;
    #elif (MillisTimer_Prescale_Value == 64 && F_CPU == 16500000L) //(16500000) - (16500000 >> 5) = approx 16000000
      m = (((m << 8) + t) << 2 ); // multiply by 4 - this gives us the value it would be if it were 16 MHz
      return (m - (m>>5));        // but it's not - we want 32/33nds of that. We can't divide an unsigned long by 33 in a time sewnsitive function. So we do 31/32nds, and that's goddamned close.
    #elif (MillisTimer_Prescale_Value == 64 && F_CPU == 24000000L) // 2.6875 vs real value 2.67
      m = (m << 8) + t;
      return (m<<1) + (m >> 1) + (m >> 3) + (m >> 4); // multiply by 2.6875
    #elif (MillisTimer_Prescale_Value == 64 && clockCyclesPerMicrosecond() == 20) // 3.187 vs real value 3.2
      m=(m << 8) + t;
      return m+(m<<1)+(m>>2)-(m>>4);
    #elif (MillisTimer_Prescale_Value == 64 && F_CPU == 18432000L) // 3.5 vs real value 3.47
      m=(m << 8) + t;
      return m+(m<<1)+(m>>1);
    #elif (MillisTimer_Prescale_Value == 64 && F_CPU==14745600L) //4.375  vs real value 4.34
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
    #elif (MillisTimer_Prescale_Value == 32 && F_CPU==7372800L) // 4.3125, vs real value 4.34 - x7 now runs timer0 twice as fast at speeds under 8 MHz
      m=(m << 8) + t;
      return (m<<2)+(m>>3)+(m>>4);
    #elif (MillisTimer_Prescale_Value == 32 && F_CPU==6000000L) // 5.3125, vs real value 5.33 - x7 now runs timer0 twice as fast at speeds under 8 MHz
      m=(m << 8) + t;
      return (m<<2)+(m)+(m>>3)+(m>>4);
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
  #endif // !CORRECT_EXACT_MICROS
  }

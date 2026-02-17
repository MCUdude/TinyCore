#ifndef Arduino_h
#define Arduino_h

#define ATTINY_CORE 1

#include <WProgram.h>

#define millistimer_(t)                           TIMER_PASTE_A( timer, TIMER_TO_USE_FOR_MILLIS, t )
#define MillisTimer_(f)                           TIMER_PASTE_A( Timer, TIMER_TO_USE_FOR_MILLIS, f )
#define MILLISTIMER_(c)                           TIMER_PASTE_A( TIMER, TIMER_TO_USE_FOR_MILLIS, c )

#define MillisTimer_SetToPowerup                  MillisTimer_(SetToPowerup)
#define MillisTimer_SetWaveformGenerationMode     MillisTimer_(SetWaveformGenerationMode)
#define MillisTimer_GetCount                      MillisTimer_(GetCount)
#define MillisTimer_IsOverflowSet                 MillisTimer_(IsOverflowSet)
#define MillisTimer_ClockSelect                   MillisTimer_(ClockSelect)
#define MillisTimer_EnableOverflowInterrupt       MillisTimer_(EnableOverflowInterrupt)
#define MILLISTIMER_OVF_vect                      MILLISTIMER_(OVF_vect)


#if F_CPU >= 3000000L
  #define MillisTimer_Prescale_Index  MillisTimer_(Prescale_Value_64)
  #define MillisTimer_Prescale_Value  (64)
  #define ToneTimer_Prescale_Index    ToneTimer_(Prescale_Value_64)
  #define ToneTimer_Prescale_Value    (64)
#else
  #define MillisTimer_Prescale_Index  MillisTimer_(Prescale_Value_8)
  #define MillisTimer_Prescale_Value  (8)
  #define ToneTimer_Prescale_Index    ToneTimer_(Prescale_Value_8)
  #define ToneTimer_Prescale_Value    (8)
#endif

// the prescaler is set so that the millis timer ticks every MillisTimer_Prescale_Value (64) clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_MILLIS_OVERFLOW (clockCyclesToMicroseconds(MillisTimer_Prescale_Value * 256))

// the whole number of milliseconds per millis timer overflow
#define MILLIS_INC (MICROSECONDS_PER_MILLIS_OVERFLOW / 1000)

// the fractional number of milliseconds per millis timer overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_MILLIS_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

#endif

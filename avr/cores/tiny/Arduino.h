#ifndef Arduino_h
#define Arduino_h

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "binary.h"

#ifdef __cplusplus
extern "C"{
#endif

#define ATTINY_CORE 1
#ifndef _NOPNOP
  #define _NOPNOP() do { __asm__ volatile ("rjmp .+0"); } while (0)
#endif

void yield(void);

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2


#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define SERIAL  0x0
#define DISPLAY 0x1

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

#define NOT_AN_INTERRUPT -1

// undefine stdlib's abs if encountered
#ifdef abs
#undef abs
#endif

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define interrupts() sei()
#define noInterrupts() cli()

#if F_CPU < 1000000L
//Prevent a divide by 0 is
#warning "Clocks per microsecond < 1. To prevent divide by 0, it is rounded up to 1."
//static inline unsigned long clockCyclesPerMicrosecond() __attribute__ ((always_inline));
//static inline unsigned long clockCyclesPerMicrosecond()
//{//
//Inline function will be optimised out.
//  return 1;
//}
  //WTF were they thinking?!
#define clockCyclesPerMicrosecond() 1UL
#else
#define clockCyclesPerMicrosecond() ( F_CPU / 1000000UL )
#endif

//#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
//#define microsecondsToClockCycles(a) ( ((a) * (F_CPU / 1000L)) / 1000L )

#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))


typedef unsigned int word;

#define bit(b) (1UL << (b))

typedef uint8_t boolean;
typedef uint8_t byte;

__attribute__((section(".init9"))) int main();
void initToneTimer(void);
void init(void);

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
int analogRead(uint8_t);
void analogReference(uint8_t mode);
void analogWrite(uint8_t, int);

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout);

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

void attachInterrupt(uint8_t, void (*)(void), int mode);
void detachInterrupt(uint8_t);

void setup(void);
void loop(void);

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.

#define analogInPinToBit(P) (P)

extern const uint16_t PROGMEM port_to_mode_PGM[];
extern const uint16_t PROGMEM port_to_input_PGM[];
extern const uint16_t PROGMEM port_to_output_PGM[];

extern const uint8_t PROGMEM digital_pin_to_port_PGM[];
extern const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[];
extern const uint8_t PROGMEM digital_pin_to_timer_PGM[];

// Get the bit location within the hardware port of the given virtual pin.
// This comes from the pins_*.c file for the active board configuration.
//
// These perform slightly better as macros compared to inline functions
//
#define const_array_or_pgm_(FUNC,ARR,IDX) ({size_t idx_ = (IDX); __builtin_constant_p((ARR)[idx_]) ? (ARR)[idx_] : FUNC((ARR)+idx_); })
#define digitalPinToPort(P) ( const_array_or_pgm_(pgm_read_byte, digital_pin_to_port_PGM, (P) ) )
#define digitalPinToBitMask(P) ( const_array_or_pgm_(pgm_read_byte, digital_pin_to_bit_mask_PGM, (P) ) )
#define digitalPinToTimer(P) ( const_array_or_pgm_(pgm_read_byte, digital_pin_to_timer_PGM, (P) ) )
#define analogInPinToBit(P) (P)
#define portOutputRegister(P) ( (volatile uint8_t *)( const_array_or_pgm_(pgm_read_word, port_to_output_PGM, (P))) )
#define portInputRegister(P) ( (volatile uint8_t *)( const_array_or_pgm_(pgm_read_word, port_to_input_PGM, (P))) )
#define portModeRegister(P) ( (volatile uint8_t *)( const_array_or_pgm_(pgm_read_word, port_to_mode_PGM, (P))) )

#define NOT_A_PIN 0
#define NOT_A_PORT 0

#define PA 1
#define PB 2
#define PC 3
#define PD 4

#define NOT_ON_TIMER 0
#define TIMER0A 1
#define TIMER0B 2
#define TIMER1A 3
#define TIMER1B 4
#define TIMER1D 5
#define TIM1AU (0x10)
#define TIM1AV (0x11)
#define TIM1AW (0x12)
#define TIM1AX (0x13)
#define TIM1BU (0x14)
#define TIM1BV (0x15)
#define TIM1BW (0x16)
#define TIM1BX (0x17)


#include "pins_arduino.h"

#ifndef USE_SOFTWARE_SERIAL
  //Default to hardware serial.
  #define USE_SOFTWARE_SERIAL 0
#endif

/*=============================================================================
  Allow the ADC to be optional for low-power applications
=============================================================================*/

#ifndef TIMER_TO_USE_FOR_MILLIS
  #define TIMER_TO_USE_FOR_MILLIS                     0
#endif
/*
  Tone goes on whichever timer was not used for millis.
*/
#if TIMER_TO_USE_FOR_MILLIS == 1
  #define TIMER_TO_USE_FOR_TONE                     0
#else
  #define TIMER_TO_USE_FOR_TONE                     1
#endif

#if NUM_ANALOG_INPUTS > 0
  #define HAVE_ADC                  1
  #ifndef INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER
    #define INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER   1
  #endif
#else
  #define HAVE_ADC                0
  #if defined(INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER)
    #undef INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER
  #endif
  #define INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER  0
#endif

#if !HAVE_ADC
  #undef INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER
  #define INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER  0
#else
  #ifndef INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER
    #define INITIALIZE_ANALOG_TO_DIGITAL_CONVERTER   1
  #endif
#endif

/*=============================================================================
  Allow the "secondary timers" to be optional for low-power applications
=============================================================================*/

#ifndef INITIALIZE_SECONDARY_TIMERS
  #define INITIALIZE_SECONDARY_TIMERS               1
#endif


#ifdef __cplusplus
} // extern "C"
#endif

#ifdef __cplusplus
#include "WCharacter.h"
#include "WString.h"
#include "HardwareSerial.h"
#include "TinySoftwareSerial.h"

uint16_t makeWord(uint16_t w);
uint16_t makeWord(byte h, byte l);

#define word(...) makeWord(__VA_ARGS__)

unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
#ifndef DISABLEMILLIS
unsigned long pulseInLong(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);
#endif

void tone(uint8_t _pin, unsigned long frequency, unsigned long duration = 0);
void noTone(uint8_t _pin = 255);

// WMath prototypes
long random(long);
long random(long, long);
void randomSeed(unsigned int);
long map(long, long, long, long, long);

#endif

/*=============================================================================
  Aliases for the interrupt service routine vector numbers so the code
  doesn't have to be riddled with #ifdefs.
=============================================================================*/

#ifndef SIGRD
  #ifndef RSIG
    #define SIGRD 5
    #define RSIG 5
  #else
    #define SIGRD RSIG
  #endif
#else
  #ifndef RSIG
  #define RSIG SIGRD
    #define RSIG SIGRD
  #endif
#endif


#if defined( TIM0_CAPT_vect ) && ! defined( TIMER0_CAPT_vect )
#define TIMER0_CAPT_vect TIM0_CAPT_vect
#endif

#if defined( TIM0_COMPA_vect ) && ! defined( TIMER0_COMPA_vect )
#define TIMER0_COMPA_vect TIM0_COMPA_vect
#endif

#if defined( TIM0_COMPB_vect ) && ! defined( TIMER0_COMPB_vect )
#define TIMER0_COMPB_vect TIM0_COMPB_vect
#endif

#if defined( TIM0_OVF_vect ) && ! defined( TIMER0_OVF_vect )
#define TIMER0_OVF_vect TIM0_OVF_vect
#endif

#if defined( TIM1_CAPT_vect ) && ! defined( TIMER1_CAPT_vect )
#define TIMER1_CAPT_vect TIM1_CAPT_vect
#endif

#if defined( TIM1_COMPA_vect ) && ! defined( TIMER1_COMPA_vect )
#define TIMER1_COMPA_vect TIM1_COMPA_vect
#endif

#if defined( TIM1_COMPB_vect ) && ! defined( TIMER1_COMPB_vect )
#define TIMER1_COMPB_vect TIM1_COMPB_vect
#endif

#if defined( TIM1_OVF_vect ) && ! defined( TIMER1_OVF_vect )
#define TIMER1_OVF_vect TIM1_OVF_vect
#endif

#if defined( TIM2_CAPT_vect ) && ! defined( TIMER2_CAPT_vect )
#define TIMER2_CAPT_vect TIM2_CAPT_vect
#endif

#if defined( TIM2_COMPA_vect ) && ! defined( TIMER2_COMPA_vect )
#define TIMER2_COMPA_vect TIM2_COMPA_vect
#endif

#if defined( TIM2_COMPB_vect ) && ! defined( TIMER2_COMPB_vect )
#define TIMER2_COMPB_vect TIM2_COMPB_vect
#endif

#if defined( TIM2_OVF_vect ) && ! defined( TIMER2_OVF_vect )
#define TIMER2_OVF_vect TIM2_OVF_vect
#endif

/*=============================================================================
  millis(), micros() and timer related macros
=============================================================================*/

#if F_CPU >= 3000000L

  #if defined(__AVR_ATtiny167__) || defined(__AVR_ATtiny87__)
    #if F_CPU < 8000000L // 4 and 6 MHz get PWM within the target range of 500-1kHz now on the one pin that is driven by timer0.
      #define timer0Prescaler (0b011)
      #define timer0_Prescale_Value  (32)
    #else
      #define timer0Prescaler (0b100)
      #define timer0_Prescale_Value  (64)
    #endif
  #else
    #define timer0Prescaler (0b011)
    #define timer0_Prescale_Value  (64)
  #endif
  #if (defined(TCCR1) || defined(TCCR1E)) // x5 and x61
    #if F_CPU < 8000000L // 4 and 6 MHz get PWM within the target range of 500-1kHz now on 2 pins of t85, and all PWM pins of the x61, since it's weirdo timer0 has "output" compare that just fires an ISR...
      #define timer1Prescaler (0b0110)
      #define timer1_Prescale_Value  (32)
    #else
      #define timer1Prescaler (0b0111)
      #define timer1_Prescale_Value  (64)
    #endif
  #else
    #define timer1Prescaler (0b011)
    #define timer1_Prescale_Value  (64)
  #endif
#else // 1 or 2 MHz system clock
  #define timer0Prescaler (0b010)
  #if (defined(TCCR1) || defined(TCCR1E))
    #define timer1Prescaler (0b0100)
  #else
    #define timer1Prescaler (0b010)
  #endif
  #define timer0_Prescale_Value    (8)
  #define timer1_Prescale_Value    (8)
#endif

#if (TIMER_TO_USE_FOR_MILLIS == 0)
  #define MillisTimer_Prescale_Value  (timer0_Prescale_Value)
  #define ToneTimer_Prescale_Value    (timer1_Prescale_Value)
  #define MillisTimer_Prescale_Index  (timer0Prescaler)
  #define ToneTimer_Prescale_Index    (timer1Prescaler)
#else
  #warning "WARNING: Use of Timer1 for millis has been configured - this option is untested and unsupported!"
  #define MillisTimer_Prescale_Value  (timer1_Prescale_Value)
  #define ToneTimer_Prescale_Value    (timer0_Prescale_Value)
  #define MillisTimer_Prescale_Index  (timer1Prescaler)
  #define ToneTimer_Prescale_Index    (timer0Prescaler)
#endif

// the prescaler is set so that the millis timer ticks every MillisTimer_Prescale_Value (64) clock cycles, and the
// the overflow handler is called every 256 ticks.
#if 0 // generally valid scaling formula follows below in the #else branch
#if (F_CPU==12800000)
//#define MICROSECONDS_PER_MILLIS_OVERFLOW (clockCyclesToMicroseconds(MillisTimer_Prescale_Value * 256))
//#define MICROSECONDS_PER_MILLIS_OVERFLOW ((64 * 256)/12.8) = 256*(64/12.8) = 256*5 = 1280
#define MICROSECONDS_PER_MILLIS_OVERFLOW (1280)
#elif (F_CPU==16500000)
#define MICROSECONDS_PER_MILLIS_OVERFLOW (992)
#else
#define MICROSECONDS_PER_MILLIS_OVERFLOW (clockCyclesToMicroseconds(MillisTimer_Prescale_Value * 256))
#endif
#else
/* The key is never to compute (F_CPU / 1000000L), which may lose precision.
   The formula below is correct for all F_CPU times that evenly divide by 10,
   at least for prescaler values up and including 64 as used in this file. */
#if MillisTimer_Prescale_Value <= 64
#define MICROSECONDS_PER_MILLIS_OVERFLOW \
  (MillisTimer_Prescale_Value * 256UL * 1000UL * 100UL / ((F_CPU + 5UL) / 10UL))
#else
/* It may be sufficient to swap the 100L and 10L in the above formula, but
   please double-check EXACT_NUMERATOR and EXACT_DENOMINATOR below as well
   and make sure it does not roll over. */
#define MICROSECONDS_PER_MILLIS_OVERFLOW 0
#error "Please adjust MICROSECONDS_PER_MILLIS_OVERFLOW formula"
#endif
#endif

/* Correct millis to zero long term drift
   --------------------------------------

   When MICROSECONDS_PER_MILLIS_OVERFLOW >> 3 is exact, we do nothing.
   In this case, millis() has zero long-term drift, that is,
   it precisely follows the oscillator used for timing.

   When it has a fractional part that leads to an error when ignored,
   we apply a correction.  This correction yields a drift of 30 ppm or less:
   1e6 / (512 * (minimum_MICROSECONDS_PER_MILLIS_OVERFLOW >> 3)) <= 30.

   The mathematics of the correction are coded in the preprocessor and
   produce compile-time constants that do not affect size or run time.
 */

/* We cancel a factor of 10 in the ratio MICROSECONDS_PER_MILLIS_OVERFLOW
   and divide the numerator by 8.  The calculation fits into a long int
   and produces the same right shift by 3 as the original code.
 */
#define EXACT_NUMERATOR (MillisTimer_Prescale_Value * 256UL * 12500UL)
#define EXACT_DENOMINATOR ((F_CPU + 5UL) / 10UL)

/* The remainder is an integer in the range [0, EXACT_DENOMINATOR). */
#define EXACT_REMAINDER \
  (EXACT_NUMERATOR - (EXACT_NUMERATOR / EXACT_DENOMINATOR) * EXACT_DENOMINATOR)

/* If the remainder is zero, MICROSECONDS_PER_MILLIS_OVERFLOW is exact.

   Otherwise we compute the fractional part and approximate it by the closest
   rational number n / 256.  Effectively, we increase millis accuracy by 512x.

   We compute n by scaling down the remainder to the range [0, 256].
   The two extreme cases 0 and 256 require only trivial correction.
   All others are handled by an unsigned char counter in millis().
 */
#define CORRECT_FRACT_PLUSONE // possibly needed for high/cheap corner case
#if EXACT_REMAINDER > 0
#define CORRECT_EXACT_MILLIS // enable zero drift correction in millis()
#define CORRECT_EXACT_MICROS // enable zero drift correction in micros()
#define CORRECT_EXACT_MANY \
  ((2U * 256U * EXACT_REMAINDER + EXACT_DENOMINATOR) / (2U * EXACT_DENOMINATOR))
#if CORRECT_EXACT_MANY < 0 || CORRECT_EXACT_MANY > 256
#error "Miscalculation in millis() exactness correction"
#endif
#if CORRECT_EXACT_MANY == 0 // low/cheap corner case
#undef CORRECT_EXACT_MILLIS // go back to nothing for millis only
#elif CORRECT_EXACT_MANY == 256 // high/cheap corner case
#undef CORRECT_EXACT_MILLIS // go back to nothing for millis only
#undef CORRECT_FRACT_PLUSONE // but use this macro...
#define CORRECT_FRACT_PLUSONE + 1 // ...to add 1 more to fract every time
#endif // cheap corner cases
#endif // EXACT_REMAINDER > 0
/* End of preparations for exact millis() with oddball frequencies */

/* More preparations to optimize calculation of exact micros().
   The idea is to reduce microseconds per overflow to unsigned char.
   Then we find the leading one-bits to add, avoiding multiplication.

   This way of calculating micros is currently enabled whenever
   *both* the millis() exactness correction is enabled
   *and* MICROSECONDS_PER_MILLIS_OVERFLOW < 65536.
   Otherwise we fall back to the existing micros().
 */
#ifdef CORRECT_EXACT_MICROS
#if MICROSECONDS_PER_MILLIS_OVERFLOW >= (1 << 16)
#undef CORRECT_EXACT_MICROS // disable correction for such long intervals
#elif MICROSECONDS_PER_MILLIS_OVERFLOW >= (1 << 15)
#define CORRECT_BITS 8
#elif MICROSECONDS_PER_MILLIS_OVERFLOW >= (1 << 14)
#define CORRECT_BITS 7
#elif MICROSECONDS_PER_MILLIS_OVERFLOW >= (1 << 13)
#define CORRECT_BITS 6
#elif MICROSECONDS_PER_MILLIS_OVERFLOW >= (1 << 12)
#define CORRECT_BITS 5
#elif MICROSECONDS_PER_MILLIS_OVERFLOW >= (1 << 11)
#define CORRECT_BITS 4
#elif MICROSECONDS_PER_MILLIS_OVERFLOW >= (1 << 10)
#define CORRECT_BITS 3
#elif MICROSECONDS_PER_MILLIS_OVERFLOW >= (1 << 9)
#define CORRECT_BITS 2
#elif MICROSECONDS_PER_MILLIS_OVERFLOW >= (1 << 8)
#define CORRECT_BITS 1
#else
#define CORRECT_BITS 0
#endif
#ifdef CORRECT_BITS // microsecs per overflow in the expected range of values
#define CORRECT_BIT7S (0)
#define CORRECT_BIT6
#define CORRECT_BIT5
#define CORRECT_BIT4
#define CORRECT_BIT3
#define CORRECT_BIT2
#define CORRECT_BIT1
#define CORRECT_BIT0
#define CORRECT_UINT ((unsigned int) t)
#define CORRECT_BYTE (MICROSECONDS_PER_MILLIS_OVERFLOW >> CORRECT_BITS)
#if CORRECT_BYTE >= (1 << 8)
#error "Miscalculation in micros() exactness correction"
#endif
#if (CORRECT_BYTE & (1 << 7))
#undef  CORRECT_BIT7S
#define CORRECT_BIT7S (CORRECT_UINT << 1)
#endif
#if (CORRECT_BYTE & (1 << 6))
#undef  CORRECT_BIT6
#define CORRECT_BIT6 + CORRECT_UINT
#endif
#if (CORRECT_BYTE & (1 << 5))
#undef  CORRECT_BIT5
#define CORRECT_BIT5 + CORRECT_UINT
#endif
#if (CORRECT_BYTE & (1 << 4))
#undef  CORRECT_BIT4
#define CORRECT_BIT4 + CORRECT_UINT
#endif
#if (CORRECT_BYTE & (1 << 3))
#undef  CORRECT_BIT3
#define CORRECT_BIT3 + CORRECT_UINT
#endif
#if (CORRECT_BYTE & (1 << 2))
#undef  CORRECT_BIT2
#define CORRECT_BIT2 + CORRECT_UINT
#endif
#if (CORRECT_BYTE & (1 << 1))
#undef  CORRECT_BIT1
#define CORRECT_BIT1 + CORRECT_UINT
#endif
#if (CORRECT_BYTE & (1 << 0))
#undef  CORRECT_BIT0
#define CORRECT_BIT0 + CORRECT_UINT
#endif
#endif // CORRECT_BITS
#endif // CORRECT_EXACT_MICROS

// the whole number of milliseconds per millis timer overflow
#define MILLIS_INC (MICROSECONDS_PER_MILLIS_OVERFLOW / 1000U)

// the fractional number of milliseconds per millis timer overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC (((MICROSECONDS_PER_MILLIS_OVERFLOW % 1000U) >> 3) \
                   CORRECT_FRACT_PLUSONE)
#define FRACT_MAX (1000U >> 3)



#endif

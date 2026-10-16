#ifndef TinyCore_h
#define TinyCore_h

#include <Arduino.h>

#if defined(__AVR_ATtinyX5__)
  typedef enum TIMERONE_CLOCK_enum
  {
      TIMER1_FCPU    = (0x00),
      TIMER1_64M_PLL = (0x06),
      TIMER1_32M_PLL = (0x86)
  } TIMERONE_CLOCK_t;
  typedef enum TIMERONE_PRESC_enum
  {
    TIMER1_STOPPED  = (0x00),
    TIMER1_DIV1     = (0x01),
    TIMER1_DIV2     = (0x02),
    TIMER1_DIV4     = (0x03),
    TIMER1_DIV8     = (0x04),
    TIMER1_DIV16    = (0x05),
    TIMER1_DIV32    = (0x06),
    TIMER1_DIV64    = (0x07),
    TIMER1_DIV128   = (0x08),
    TIMER1_DIV256   = (0x09),
    TIMER1_DIV512   = (0x0A),
    TIMER1_DIV1024  = (0x0B),
    TIMER1_DIV2048  = (0x0C),
    TIMER1_DIV4096  = (0x0D),
    TIMER1_DIV8192  = (0x0E),
    TIMER1_DIV16384 = (0x0F)
  } TIMER1_PRESCALE_t;

  static inline void setTimer1Clock(TIMERONE_CLOCK_t clk) {
    uint8_t t = PLLCSR & 0x87;
    if ((t & 0x86) == clk) {    // already set right
      return;
    }
    uint8_t oldtccr = TCCR1;
    TCCR1 = oldtccr & 0xF0;
    if ((clk & 0x04) && ((t & 0x03) != 3)) {
      // PLL requested and not currently running
      if ((t & 0x02) == 0) {
        PLLCSR = clk & 0x02; // enable the PLL
      }
      while ((PLLCSR & 0x01) == 0); // Wait for lock
    }
    PLLCSR = clk; // now we can set the PLLCSR.
    TCCR1 = oldtccr; // and turn back on the timer.
  }
  static inline void setTimer1Prescaler(TIMER1_PRESCALE_t presc) {
    uint8_t tccr = TCCR1;
    tccr &= 0xF0;
    tccr |= presc;
    TCCR1 = tccr;
  }
  static inline void setTimer1Top(uint8_t newtop) {
    OCR1C = newtop;
  }
  static inline uint8_t setTimer1Frequency(uint32_t target) {
    while (true) {
      uint8_t pll = PLLCSR;
      pll &= 0x84;
      unsigned long speed;
      if (pll == 0x84) {
        #if F_CPU  == 16500000
          speed     = 33000000;
        #else
          speed     = 32000000;
        #endif
      } else if (pll) {
        #if F_CPU  == 16500000
          speed     = 66000000;
        #else
          speed     = 64000000;
        #endif
      } else {
        speed     = F_CPU;
      }
      uint32_t topval = speed/target;
      if (topval < 64 && pll != 0x04) {
        setTimer1Clock(TIMER1_64M_PLL);
      } else if (topval < 128 && pll == 0) {
        setTimer1Clock(TIMER1_32M_PLL);
      } else {
        uint8_t newpresc = 1;
        while (topval > 255 && newpresc < 15) {
          newpresc++;
          topval    >>= 1;
        }
        uint8_t top;
        if (topval > 255) {
          top = 255;
        } else {
          top = topval;
        }
        OCR1C         = top;
        uint8_t tccr  = TCCR1;
        tccr         &= 0xF0;
        tccr         |= newpresc;
        TCCR1         = tccr;
        return top;
      }
    }
  }
#endif

#if defined(__AVR_ATtinyX61__)
    typedef enum TIMERONE_CLOCK_enum
  {
      TIMER1_FCPU    = (0x00),
      TIMER1_64M_PLL = (0x06),
      TIMER1_32M_PLL = (0x86)
  } TIMERONE_CLOCK_t;
  typedef enum TIMERONE_PRESC_enum
  {
    TIMER1_STOPPED  = (0x00),
    TIMER1_DIV1     = (0x01),
    TIMER1_DIV2     = (0x02),
    TIMER1_DIV4     = (0x03),
    TIMER1_DIV8     = (0x04),
    TIMER1_DIV16    = (0x05),
    TIMER1_DIV32    = (0x06),
    TIMER1_DIV64    = (0x07),
    TIMER1_DIV128   = (0x08),
    TIMER1_DIV256   = (0x09),
    TIMER1_DIV512   = (0x0A),
    TIMER1_DIV1024  = (0x0B),
    TIMER1_DIV2048  = (0x0C),
    TIMER1_DIV4096  = (0x0D),
    TIMER1_DIV8192  = (0x0E),
    TIMER1_DIV16384 = (0x0F)
  } TIMER1_PRESCALE_t;

  static inline void setTimer1Clock(TIMERONE_CLOCK_t clk) {
    uint8_t t = PLLCSR & 0x87;
    if ((t & 0x86) == clk) {    // already set right
      return;
    }
    uint8_t oldtccr = TCCR1B;
    TCCR1B = oldtccr & 0xF0;
    if ((clk & 0x04) && ((t & 0x03) != 3)) {
      // PLL requested and not currently running
      if ((t & 0x02) == 0) {
        PLLCSR = clk & 0x02; // enable the PLL
      }
      while ((PLLCSR & 0x01) == 0); // Wait for lock
    }
    PLLCSR = clk; // now we can set the PLLCSR.
    TCCR1B = oldtccr; // and turn back on the timer.
  }
  static inline void setTimer1Prescaler(TIMER1_PRESCALE_t presc) {
    uint8_t tccr = TCCR1B;
    tccr &= 0xF0;
    tccr |= presc;
    TCCR1B = tccr;
  }
  static inline void setTimer1Top(uint8_t newtop) {
    uint8_t oldsreg = SREG;
    cli();
    TC1H  = 0;
    OCR1C = newtop;
    SREG  = oldsreg;
  }

  static inline void setTimer1Top_10bit(uint16_t newtop) {
    TC1H = (newtop >> 8);
    OCR1C = newtop & 0xFF;
  }
  static inline uint8_t setTimer1Frequency(uint32_t target) {
    while (true) {
      unsigned long speed;
      uint8_t pll   = PLLCSR;
      pll          &= 0x84;
      if (pll == 0x84) {
        #if F_CPU  == 16500000
          speed     = 33000000;
        #else
          speed     = 32000000;
        #endif
      } else if (pll) {
        #if F_CPU  == 16500000
          speed     = 66000000;
        #else
          speed     = 64000000;
        #endif
      } else {
        speed       = F_CPU;
      }
      uint32_t topval = speed/target;
      if (topval < 64 && pll != 0x04) {
        setTimer1Clock(TIMER1_64M_PLL);
      } else if (topval < 128 && pll == 0) {
        setTimer1Clock(TIMER1_32M_PLL);
      } else {
        uint8_t newpresc = 1;
        while (topval > 255 && newpresc < 15) {
          newpresc++;
          topval    >>= 1;
        }
        uint8_t top;
        if (topval > 256) {
          top = 255;
        } else {
          top = (uint8_t)topval - 1;
        }
        OCR1C         = top;
        uint8_t tccr  = TCCR1B;
        tccr         &= 0xF0;
        tccr         |= newpresc;
        TCCR1B        = tccr;
        return top;
      }
    }
  }
  static inline uint16_t setTimer1Frequency_10bit(uint32_t target) {
    while (true) {
      unsigned long speed;
      uint8_t pll   = PLLCSR;
      pll          &= 0x84;
      if (pll == 0x84) {
        #if F_CPU  == 16500000
          speed     = 33000000;
        #else
          speed     = 32000000;
        #endif
      } else if (pll) {
        #if F_CPU  == 16500000
          speed     = 66000000;
        #else
          speed     = 64000000;
        #endif
      } else {
        speed       = F_CPU;
      }
      uint32_t topval = speed/target;
      if (topval < 10 && pll != 0x04) {
        setTimer1Clock(TIMER1_64M_PLL);
      } else if (topval < 128 && pll == 0) {
        setTimer1Clock(TIMER1_32M_PLL);
      } else {
        uint8_t newpresc = 1;
        while (topval > 1024 && newpresc < 15) {
          newpresc++;
          topval    >>= 1;
        }
        uint16_t top;
        if (topval >= 1024) {
          top = 1023;
        } else {
          top = (uint16_t)topval - 1;
          setTimer1Top_10bit(top);
        }
        uint8_t tccr  = TCCR1B;
        tccr         &= 0xF0;
        tccr         |= newpresc;
        TCCR1B        = tccr;
        return top;
      }
    }
  }
  static inline uint8_t pwmchanneltopin(uint8_t ch) {
    #if defined (PINMAPPING_NEW)
      if (ch < 3) {
        return (uint8_t)9 + ch + ch;
      }
    #else
      if (ch <3) {
        return (uint8_t)8 - ch - ch;
      }
    #endif
    return NOT_A_PIN;
  }
  static inline void setTimer1DutyCycle_10bit(uint8_t channel, uint16_t duty) {
    if (channel > 2 || duty > 1023) {
      return; // invalid channel
    }
    if (duty == 0) {
      digitalWrite(pwmchanneltopin(channel), LOW);
    } else if (duty == 1023) {
      digitalWrite(pwmchanneltopin(channel), HIGH);
    } else {
      uint8_t oldsreg = SREG;
      cli();
      TC1H    = duty >> 8;
      if (channel == 0) {
        TCCR1C |= (1 << COM1A1S);
        OCR1A = (uint8_t) duty;
      } else if (channel == 1) {
        TCCR1C |= (1 << COM1B1S);
        OCR1B = (uint8_t) duty;
      } else {
        TCCR1C |= (1 << COM1D1);
        OCR1D = (uint8_t) duty;
      }
      SREG    = oldsreg;
    }
  }
#endif

/* a grab-bag of miscellaneous functionality */



/* A grab-bag of miscellaneous functionality.
 *
 * These features exist on some parts and not on others. Where a part does not
 * have one, the function is not defined at all and a macro of the same name
 * expands to badCall(), so that a call is diagnosed at compile time while
 * nothing is emitted for a part whose sketch never calls it. A stub function
 * would have external linkage and be emitted either way, and its badCall()
 * would then fire on its own.
 *
 * They return 0 on success and 1 if the port or pin does not have the feature.
 */

#if defined(__AVR_ATtiny828__)
  static inline uint8_t enableHighSinkPort(uint8_t port, bool mode) {
    if (port == PC) {
      PHDE = mode ? 0x04 : 0;
      return 0;
    }
    return 1;
  }
#else
  #define enableHighSinkPort(...) (badCall("Only the 828 has a whole port that can be set high-sink, though the 841/441 can set PA5 and PA7 high sink with enableHighSinkPin()"),1)
#endif

#if defined(__AVR_ATtinyX41__)
  static inline uint8_t enableHighSinkPin(uint8_t pin, bool mode) {
    if (pin == PIN_PA7 || pin == PIN_PA5) {
      uint8_t bit = (pin == PIN_PA7) ? 2 : 1;
      if (mode) {
        PHDE |= bit;
      } else {
        PHDE &= ~bit;
      }
      return 0;
    }
    return 1;
  }
#else
  #define enableHighSinkPin(...) (badCall("Only the 841 and 441 support this. The 828 can set PORTC high-sink with enableHighSinkPort()"),1)
#endif

/* The t87 and t167 can connect PA3 to an internal current source. This is much
 * more precise than the internal pullups, and does not vary over voltage changes
 * like they do. It was proposed for LIN addressing and a variety of other
 * uncommon tasks. The datasheet discourages leaving it on for an extended
 * period of time.
 */
#if defined(__AVR_ATtinyX7__)
  static inline void enableISRC(bool mode) {
    if (mode) {
      AMISCR |= 1 << ISRCEN;
    } else {
      AMISCR &= ~(1 << ISRCEN);
    }
  }
#else
  #define enableISRC(...) badCall("The internal current source is only available on the t87 and t167")
#endif

#if defined(PUD)
  static inline void disableAllPullups(bool mode) {
    if (mode) {
      MCUCR |= 1 << PUD;
    } else {
      MCUCR &= ~(1 << PUD);
    }
  }
#else
  #define disableAllPullups(...) badCall("The global pullup disable feature is not available on this part")
#endif

/* Port-wise pullup disabling. If either the global or the port PUD bit is set,
 * the pullups on that port are disabled.
 */
#if defined(PORTCR) && defined(PUDB)   // every part has a PORTB
  static inline uint8_t disablePortPullups(uint8_t port, bool mode) {
    uint8_t bit;
    switch (port) {
      #if defined(PORTA) && defined(PUDA)
        case PA: bit = 1 << PUDA; break;
      #endif
      #if defined(PORTB) && defined(PUDB)
        case PB: bit = 1 << PUDB; break;
      #endif
      #if defined(PORTC) && defined(PUDC)
        case PC: bit = 1 << PUDC; break;
      #endif
      #if defined(PORTD) && defined(PUDD)
        case PD: bit = 1 << PUDD; break;
      #endif
      default: return 1;
    }
    if (mode) {
      PORTCR |= bit;
    } else {
      PORTCR &= ~bit;
    }
    return 0;
  }
#else
  #define disablePortPullups(...) (badCall("This part does not support portwise pullup disabling"),1)
#endif

/* Break-Before-Make switching inserts one system clock of tristate on a pin
 * when the DDR register is changed.
 */
#if defined(PORTCR)
  static inline uint8_t enableBBM(uint8_t port, bool mode) {
    uint8_t bit;
    switch (port) {
      #if defined(PORTA) && defined(BBMA)
        case PA: bit = 1 << BBMA; break;
      #endif
      #if defined(PORTB) && defined(BBMB)
        case PB: bit = 1 << BBMB; break;
      #endif
      #if defined(PORTC) && defined(BBMC)
        case PC: bit = 1 << BBMC; break;
      #endif
      #if defined(PORTD) && defined(BBMD)
        case PD: bit = 1 << BBMD; break;
      #endif
      default: return 1;
    }
    if (mode) {
      PORTCR |= bit;
    } else {
      PORTCR &= ~bit;
    }
    return 0;
  }
#else
  #define enableBBM(...) (badCall("This part does not support break-before-make switching"),1)
#endif

#endif

/*
  TinySoftwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 23 November 2006 by David A. Mellis
  Modified 28 September 2010 by Mark Sproul
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "Arduino.h"
#include "wiring_private.h"

#if USE_SOFTWARE_SERIAL
#include "TinySoftwareSerial.h"
extern "C"{

  #ifndef TX_ONLY
    #define RXMASK (1 << SOFTSERIAL_RXBIT)
    soft_ring_buffer rx_buffer = {{ 0 }, 0, 0};
    ISR(SOFTSERIAL_vect) {
      uint8_t ch = 0;
    __asm__ __volatile__ (
		"   rcall uartDelay\n"          // Get to 0.25 of start bit (our baud is too fast, so give room to correct)
		"1: rcall uartDelay\n"              // Wait 0.25 bit period
		"   rcall uartDelay\n"              // Wait 0.25 bit period
		"   rcall uartDelay\n"              // Wait 0.25 bit period
		"   rcall uartDelay\n"              // Wait 0.25 bit period
		"   clc\n"
		"   in r23,%[pin]\n"
		"   andi r23, %[mask]\n"
		"   breq 2f\n"
		"   sec\n"
		"2: ror   %0\n"                    
		"   dec   %[count]\n"
		"   breq  3f\n"
		"   rjmp  1b\n"
		"3: rcall uartDelay\n"              // Wait 0.25 bit period
		"   rcall uartDelay\n"              // Wait 0.25 bit period
		:
		  "=r" (ch)
		:
		  "0" ((uint8_t)0),
		  [count] "r" ((uint8_t)8),
		  [pin] "I" (_SFR_IO_ADDR(ANALOG_COMP_PIN)),
		  [mask] "M" (RXMASK)
		:
		  "r23",
		  "r24",
		  "r25"
    );
      /* clear the flag once we have finished receiving the byte. Per datasheet:
          "the Program Counter is vectored to the actual Interrupt Vector in order to execute the interrupt handling routine, and hardware clears the corresponding Interrupt Flag."
        This implies that the automatic clearing happens **at the start** of the interrupt. But the interrupt condition (rising edge of ACO) will occur again, likely several times
        over the course of receiving a byte, so we had damned well better clear this bit manually!
      */
      #if !defined(__AVR_ATtinyx8__)
        ACSR |= (1 << ACI); // SBI - except on x8 where this isn't in low I/O
      #else
        ACSR = (1 << ACBG) | (1 << ACIS1) | (1 << ACIS0) | (1 << ACI) | (1 << ACIE); // ldi, out
      #endif
      uint8_t i = (uint8_t)(rx_buffer.head + 1) & (SERIAL_BUFFER_SIZE-1); //lds, andi

      // if we should be storing the received character into the location
      // just before the tail (meaning that the head would advance to the
      // current location of the tail), we're about to overflow the buffer
      // and so we don't write the character or advance the head.
      if (i != rx_buffer.tail) { // lds, cp, breq
        rx_buffer.buffer[rx_buffer.head] = ch; // ldi, ldi, add, adc, st
        rx_buffer.head = i; // sts
      }
      // total 2 + (2+1) + (2 + 1 + 1) + (1 + 1 + 1 + 1 + 2) = 2 + 3 + 4 + 6 = 15 clocks from sampling last bit
      // epilogue = prologue only with pop's instead of pushes, same time as prologue on classic AVRs = 11 clocks
      // reti 4 clocks
      // total 15 + 11 + 4 = 30 clocks from end of asm to end of ISR. which is 447 + 99 * _delayCount clocks from it's start
      // 477 + 99 * _delayCount total.
      // the falling edge of the next byte's start bit must not occur during this time, or we will get garbage for the next byte
      // (actual byte duration in clocks) < 477 * 99 * delayCount, or (actual bit time in clocks ) < 34 (30 + the 4 clocks from sample time to end of ISR) worst case assuming we're barely catching end of last bit.
    }

  #endif

  void uartDelay() {
    __asm__ __volatile__ (
      "mov r25,%[count]\n"
      "1:dec r25\n"
        "brne 1b\n"
        "ret\n"
      ::[count] "r" ((uint8_t)Serial._delayCount)
    );
  }
}
#if defined(TX_ONLY)
  TinySoftwareSerial::TinySoftwareSerial() {
    _txmask   = _BV(SOFTSERIAL_TXBIT);
    _delayCount = 0;
  }
#else
  TinySoftwareSerial::TinySoftwareSerial(soft_ring_buffer *rx_buffer) {
    _rx_buffer = rx_buffer;
    _txmask   = _BV(SOFTSERIAL_TXBIT);
    _delayCount = 0;
  }
#endif

void TinySoftwareSerial::setTxBit(uint8_t txbit) {
  _txmask   = _BV(txbit);
}

void TinySoftwareSerial::begin(long baud) {
  long tempDelay = (((F_CPU/baud) - 39) / 12);
  if ((tempDelay > 255) || (tempDelay <= 0)) {
    return; //Cannot start - baud rate out of range.
  }
  _delayCount = (uint8_t)tempDelay;
  #ifndef TX_ONLY
    //Straight assignment, we need to configure all bits
    // ACBR connects the 1.1v bandgap reference the positive side of the analog comparator. ACO is high when AINp > AINn.
    // since AINn is our RX line, not AINp, ACO goes high when the RX line falls below 1.1v (ie, ACO is inverted relative to
    // the RX input). Hence we want ACIS = 11 to interrupt on the rising edge of ACO so we get the falling edge of RX.
    ACSR = (1 << ACBG) | (1 << ACIS1) | (1 << ACIS0) | (1 << ACI);
    // These should compile to cbi and sbi - everything is compile time known.
    SOFTSERIAL_DDR    &=  ~(1 << SOFTSERIAL_RXBIT);  // set RX to an input
    SOFTSERIAL_PORT   |=   (1 << SOFTSERIAL_RXBIT);  // enable pullup on RX pin - to prevent accidental interrupt triggers.
    #if !defined(__AVR_ATtinyx8__) // on most classic tinyies ACSR is in the low IO space, so we can use sbi, cbi instructions.
      ACSR            |=   (1 <<  ACI);              // clear the flag - above configuration may cause it to be set.
      ACSR            |=   (1 << ACIE);              // turn on the comparator interrupt
    #else
      ACSR = (1 << ACBG) | (1 << ACIS1) | (1 << ACIS0) | (1 << ACI) | (1 << ACIE); // do this with an LDI and OUT on the x8 series.
    #endif
    #ifdef ACSRB
      /* This is only the case on an ATtiny x61, out of the parts that support this builtin Software serial. The 1634 and 828 have it
       * but they aren't supported because they have hardware serial. The 841 has multiple AC's, which would give the same sort of choices, except
       * that t has hardware serial ports too. so we don't provide the builtin softSeral.
       * On the 861, this is located in the low I/O space. We assume ACSRB is is POR state, hence 0, so it's better to use |=, which compiles to SBI instead of = x which compiles to ldi r__, x out ACSRB,x
       */
      #if defined(SOFTSERIAL_RXAIN0)
        ACSRB |= 2; // Use PA6 (AIN0)
      #elif defined(SOFTSERIAL_RXAIN2)
        ACSRB |= 1; // Use PA5 (AIN2)
      #endif
      // Otherwise we leave it at 0;
    #endif
  #endif
  uint8_t oldsreg      =   SREG;    //  These are NOT going to get compiled to cbi/sbi as _txmask is not compile time known.
  cli();                            //  so we need to protect this.
  SOFTSERIAL_DDR      |=   _txmask; //  set TX to an output.
  SOFTSERIAL_PORT     |=   _txmask; //  set TX pin high
  SREG                 =   oldsreg; //  restore SREG.
}

void TinySoftwareSerial::end() {
  #ifndef TX_ONLY
    ACSR = (1 << ACD) | (1 << ACI); // turn off the analog comparator, clearing the flag while we're at it.
    _rx_buffer->head = _rx_buffer->tail;
  #endif
  _delayCount = 0;
}

int TinySoftwareSerial::available(void) {
  #ifndef TX_ONLY
    if (_delayCount) {
      return (uint8_t)(SERIAL_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) & (SERIAL_BUFFER_SIZE-1);
    }
  #endif
  return 0;
}


int TinySoftwareSerial::peek(void) {
  #ifndef TX_ONLY
    if (_rx_buffer->head == _rx_buffer->tail) {
      return -1;
    } else {
      return _rx_buffer->buffer[_rx_buffer->tail];
    }
  #else
    return -1;
  #endif
}

int TinySoftwareSerial::read(void) {
  #ifndef TX_ONLY
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer->head == _rx_buffer->tail || _begun != 0) {
      return -1;
    } else {
      uint8_t c = _rx_buffer->buffer[_rx_buffer->tail];
      _rx_buffer->tail = (uint8_t)(_rx_buffer->tail + 1) & (SERIAL_BUFFER_SIZE-1);
      return c;
    }
  #else
    return -1;
  #endif
}
bool TinySoftwareSerial::listen() {
  #ifndef TX_ONLY
    if (!_delayCount) {
      return false;
    }
    if (ACSR & (1 << ACD)) {
      _rx_buffer->head = 0;
      _rx_buffer->tail = 0;
      ACSR = (1 << ACBG) | (1 << ACIS1) | (1 << ACIS0) | (1 << ACI); // must not have ACIE set while changing ACD
      #if defined(__AVR_ATtiny_x8__) // not in the low I/O space so use LDI, OUT
        ACSR = (1 << ACBG) | (1 << ACIS1) | (1 << ACIS0) | (1 << ACI) | (1 << ACIE);
      #else // we can use sbi. on 88/48 this would result in a 3 instruction IN, ORI, OUT sequence.
        ACSR |= 1 << ACIE;
      #endif
      }
    return true;
  #else
    return false;
  #endif
}

// Stop listening. Returns true if we were actually listening.
bool TinySoftwareSerial::stopListening() {
  #ifndef TX_ONLY
    if (ACSR & (1 << ACD)) {
      ACSR = (1 << ACD) | (1 << ACBG) | (1 << ACIS1) | (1 << ACIS0) | (1 << ACI);
      return true;
    }
  #endif
  return false;
}

size_t TinySoftwareSerial::write(uint8_t ch) {
  uint8_t oldSREG = SREG;
  cli();
  __asm__ __volatile__ (
    "   com %[ch]             \n" // ones complement, carry set
    "   sec                   \n"
    "1: brcc 2f               \n"
    "   in r23,%[uartPort]    \n"
    "   or r23,%[uartMask]    \n" // build the '1' pattern first
    "   eor r23,%[uartMask]   \n" // flip TX bit low - zero bit pattern, no _txunmask needed
    "   out %[uartPort],r23   \n"
    "   rjmp 3f               \n"
    "2: in r23,%[uartPort]    \n"
    "   or r23,%[uartMask]    \n"
    "   out %[uartPort],r23   \n"
    "   nop                   \n"
    "3: rcall uartDelay       \n"
    "   rcall uartDelay       \n"
    "   rcall uartDelay       \n"
    "   rcall uartDelay       \n"
    "   lsr %[ch]             \n"
    "   dec %[count]          \n"
    "   brne 1b               \n"
    :
    :
      [ch]       "r" (ch),
      [count]    "r" ((uint8_t)10),
      [uartPort] "I" (_SFR_IO_ADDR(ANALOG_COMP_PORT)),
      [uartMask] "r" (_txmask)
    : "r23", "r24", "r25"
  );
  SREG = oldSREG;
  return 1;
}

void TinySoftwareSerial::printHex(const uint8_t b) {
    char x = (b >> 4) | '0';
    if (x > '9')
      x += 7;
    write(x);
    x = (b & 0x0F) | '0';
    if (x > '9')
      x += 7;
    write(x);
  }

  void TinySoftwareSerial::printHex(const uint16_t w, bool swaporder) {
    uint8_t *ptr = (uint8_t *) &w;
    if (swaporder) {
      printHex(*(ptr++));
      printHex(*(ptr));
    } else {
      printHex(*(ptr + 1));
      printHex(*(ptr));
    }
  }

  void TinySoftwareSerial::printHex(const uint32_t l, bool swaporder) {
    uint8_t *ptr = (uint8_t *) &l;
    if (swaporder) {
      printHex(*(ptr++));
      printHex(*(ptr++));
      printHex(*(ptr++));
      printHex(*(ptr));
    } else {
      ptr+=3;
      printHex(*(ptr--));
      printHex(*(ptr--));
      printHex(*(ptr--));
      printHex(*(ptr));
    }
  }

  uint8_t * TinySoftwareSerial::printHex(uint8_t* p, uint8_t len, char sep) {
    for (byte i = 0; i < len; i++) {
      if (sep && i) write(sep);
      printHex(*p++);
    }
    println();
    return p;
  }

  uint16_t * TinySoftwareSerial::printHex(uint16_t* p, uint8_t len, char sep, bool swaporder) {
    for (byte i = 0; i < len; i++) {
      if (sep && i) write(sep);
      printHex(*p++, swaporder);
    }
    println();
    return p;
  }
void TinySoftwareSerial::flush() {
  ;
}

TinySoftwareSerial::operator bool() {
  return !!_delayCount;
}

#if defined(TX_ONLY)
  TinySoftwareSerial Serial;
#else
  TinySoftwareSerial Serial(&rx_buffer);
#endif

#endif // whole file

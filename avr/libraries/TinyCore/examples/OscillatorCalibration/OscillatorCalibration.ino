/*
  TinyCore internal oscillator tuner

  Based on picoUart by Ralph Doncaster (2020)
  https://github.com/nerdralph/picoUART
    
  Modified by MCUdude to work with TinyCore
  https://github.com/MCUdude/TinyCore
  ------------------------------------------------------------------------------

  Tunes the internal oscillator using a software serial implementation
  and a USB to serial adapter, and store the calibrated OSCCAL value to
  EEPROM address E2END (last byte in EEPROM). The default RX and TX pins are the
  ones found in the device spesific documentation.
  https://github.com/MCUdude/TinyCore#supported-microcontrollers

  The microcontroller has to be running off its internal 8 MHz or 16 MHz PLL
  oscillator, if the part has a PLL.
  Start off by opening the serial monitor and select the 115200 baud.
  Make sure you're not sending any line ending characters (CR, LF).
  Repedeatly press 'x' [send] to tune the internal oscillator. After a few
  messages you'll eventually see readable text in the serial monitor, and a
  new, stable OSCCAL value. Continue doing this until you'll get a message
  saying that the value have been stored to EEPROM address E2END. After this the
  program will halt.

  RECOMMENDED SETTINGS FOR THIS SKETCH
  ------------------------------------------------------------------------------

  Tools > Board          : Any TinyCore compatible target
  Tools > BOD            : Any BOD level
  Tools > Clock          : 8 MHz internal oscillator

  If you get garbage output:
   1. Make sure you have selected 115200 baud
   2. Make sure you ahve selected "No line ending" in the Arduino serial monitor
   2. Check if you have anything else connected to TX/RX like an LED
   3. You haven't sent enough 'x' characters yet
*/

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <picoUART.h>
#include <pu_print.h>

// Add 0.5 for integer rounding
const uint8_t CYCLES_PER_BIT = (uint8_t)(PUBIT_CYCLES + 0.5);

// converts 4-bit nibble to ascii hex
uint8_t nibbletohex(uint8_t value) {
  value &= 0x0F;
  if ( value > 9 )
    value += 'A' - ':';
  return value + '0';
}

void printHex(uint8_t value) {
  putx(nibbletohex(value/16));
  putx(nibbletohex(value));
}

// RX interrupt
ISR(PU_PCINT_vect) {
  static uint8_t cal_counter;
  
  // Start timer when pin transitions low
  if ((PURXPIN & _BV(PURXBIT)) == 0)
    TCCR0B = _BV(CS00);
  else {
    uint8_t current = TCNT0;    
    // End of interval, reset counter
    TCCR0B = 0;
    TCNT0 = 0;

    // 'x' begins with 3 zeros + start bit = 4 * bit-time
    // Match speed to soft uart bit time
    // Use mod256 math to handle potential uint8_t overflow
    uint16_t expected = (uint16_t)(PUBIT_CYCLES * 4 + 0.5);
    int8_t delta = (expected & 0xFF) - current;
    prints_P(PSTR("OSCCAL: 0x"));
    printHex(OSCCAL);
    if (delta > 3) {
      OSCCAL++;
      prints_P(PSTR(" - slow\n"));
    }
    else if (delta < -3) {
      OSCCAL--;
      prints_P(PSTR(" - fast\n"));
    }
    else {
      prints_P(PSTR(" - good\n"));
      cal_counter++;
    }
  }

  // Store new OSCCAL to EEPROM when stable    
  if(cal_counter >= 5) {
    eeprom_write_byte((uint8_t*)E2END, OSCCAL);
    prints_P(PSTR("New OSCCAL stored to EEPROM addr. 0x"));
    #if E2END > 0xFF
      printHex(E2END>>8);
    #endif
    printHex(E2END & 0xFF);
    prints_P(PSTR(" (E2END)\n"));
    while(true);
  }
  
  // Clear interrupt flag in case another triggered
  PU_PCIFR = _BV(PU_PCIF);
}

void setup() {
  // Serial monitor open delay
  _delay_ms(100);
  
  // Pullup RX line
  PURXPORT |= _BV(PURXBIT);

  printHex(OSCCAL);
  prints_P(PSTR(" Hit x to test.\n"));

  wait_x:
  // Wait for tuning character to ensure not reading noise
  // before entering tuning mode
  uint8_t counter = 0;
  while (PURXPIN & _BV(PURXBIT));
  do {
    counter++;
  } while ((PURXPIN & _BV(PURXBIT)) == 0);
  
  // Low period should be 4 bit-times for 'x'
  // Counter loop is 4 cycles, so counter =~ cycles/bit
  // 1/4 = 25% timing margin
  uint8_t margin = CYCLES_PER_BIT / 4;
  uint8_t delta = __builtin_abs(counter - CYCLES_PER_BIT);
  if (delta > margin) {
    prints_P(PSTR("Noise skipped\n"));
    goto wait_x;
  }
  
  // Skip remaining bits in frame
  _delay_ms(1);

  prints_P(PSTR(" OSCCAL "));
  if (delta < 2) { 
    prints_P(PSTR("OK\n"));
    return;
  }
  prints_P(PSTR("Imperfect\n"));
  
  // Reset counter for first interrupt
  TCCR0B = 0;
  TCNT0 = 0;

  // Prepare RX interrupt
  PU_PCMSK = _BV(PURXBIT);
  PU_PCINT_CTRL = _BV(PU_PCIE);

  // Enable interrupts
  sei();
}

void loop() {
}

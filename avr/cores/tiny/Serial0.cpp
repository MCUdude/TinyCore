#include "Arduino.h"
#if (!USE_SOFTWARE_SERIAL && !DISABLE_UART && !DISABLE_UART0)
  #include "HardwareSerial.h"
  /* We rely on the normalized register names from HardwareSerial.h */
  #if defined(UBRR0H) || defined(LINBRRH)
    #if !defined(USART0_UDRE_vect) && !defined(LIN_TC_vect)
      #error "Don't know what the Data Register Empty vector is called for the first UART"
    #endif
    #if defined(USART0_UDRE_vect) && defined(TXBUFFER)
      ISR(USART0_UDRE_vect) {
         Serial._tx_reg_empty_irq();
      }
    #endif
  #endif
  #if defined(USART0_RX_vect)
    ISR(USART0_RX_vect) {
      unsigned char c  =  UDR0;
      Serial._store_rx_char(c);
    }
  #elif defined(LIN_TC_vect)
    // this is for attinyX7
    ISR(LIN_TC_vect) {
      if(LINSIR & _BV(LRXOK)) {
          unsigned char c  =  LINDAT;
          Serial._store_rx_char(c);
      }
      if(LINSIR & _BV(LTXOK)) {
      #ifdef TXBUFFER
        //PINA |= _BV(PINA5); //debug
        Serial._tx_reg_empty_irq();
      #else
        // nothing to do because we check in the write routine for empty output buffer
      #endif
      }
    }
  #endif
  #if defined(UBRR0H)
    HardwareSerial Serial(&UBRR0H, &UBRR0L, &UCSR0A, &UCSR0B, &UDR0);
  #elif defined(LINBRRH)
    HardwareSerial Serial;
  #endif
#endif

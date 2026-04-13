#include "Arduino.h"
#if (!DISABLE_UART1 && !DISABLE_UART)
  #include "HardwareSerial.h"
  #if defined(UBRR1H)
    HardwareSerial Serial1(&UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UDR1);
  #endif
  #if defined(USART1_RX_vect)
    ISR(USART1_RX_vect)
    {
      unsigned char c = UDR1;
      Serial._store_rx_char(c);
    }
  #elif defined(USART1_RXC_vect)
    ISR(USART1_RXC_vect )
    {
      unsigned char c = UDR1;
      Serial._store_rx_char(c);
    }
  #else
    //no UART1
  #endif
  #ifdef USART1_UDRE_vect
    ISR(USART1_UDRE_vect)
    {
      Serial._tx_udr_empty_irq();
    }
  #endif
#endif

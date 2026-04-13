#include "Arduino.h"
#if (!DISABLE_UART1 && !DISABLE_UART)
  #include "HardwareSerial.h"
  #if defined(UBRR1H)
    ring_buffer rx_buffer1  =  { { 0 }, 0, 0 };
    ring_buffer tx_buffer1  =  { { 0 }, 0, 0 };
    HardwareSerial Serial1(&rx_buffer1, &tx_buffer1, &UBRR1H, &UBRR1L, &UCSR1A, &UCSR1B, &UDR1);
  #endif
  #if defined(USART1_RX_vect)
    ISR(USART1_RX_vect)
    {
      unsigned char c = UDR1;
      store_char(c, &rx_buffer1);
    }
  #elif defined(USART1_RXC_vect)
    ISR(USART1_RXC_vect )
    {
      unsigned char c = UDR1;
      store_char(c, &rx_buffer1);
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

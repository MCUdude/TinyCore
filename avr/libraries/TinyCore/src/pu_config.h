// picoUART configuration

#ifndef PU_BAUD_RATE
  #define PU_BAUD_RATE 115200L // default baud rate
#endif

// disable interrupts during Tx and Rx
#define PU_DISABLE_IRQ 1

// I/O register macros
#define GBIT(r,b)       b
#define GPORT(r,b)      (PORT ## r)
#define GDDR(r,b)       (DDR ## r)
#define GPIN(r,b)       (PIN ## r)
#define get_bit(io)     GBIT(io)
#define get_port(io)    GPORT(io)
#define get_ddr(io)     GDDR(io)
#define get_pin(io)     GPIN(io)

#define PUTXBIT     get_bit(PU_TX)
#define PUTXPORT    get_port(PU_TX)
#define PUTXDDR     get_ddr(PU_TX)
#define PURXBIT     get_bit(PU_RX)
#define PURXPORT    get_port(PU_RX)
#define PURXPIN     get_pin(PU_RX)

// Compatibility macros
#ifndef OSCCAL
  #define OSCCAL OSCCAL0
#endif
#ifndef PCMSK
  #define PCMSK PCMSK0
#endif
#ifndef PCIF
  #define PCIF PCIF0
#endif

#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  #ifndef PU_TX
    #define PU_TX B,0
  #endif
  #ifndef PU_RX
    #define PU_RX B,1
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT0_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL GIMSK
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR GIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF
  #endif

#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny441__) || defined(__AVR_ATtiny841__)
  #ifndef PU_TX
    #define PU_TX A,1
  #endif
  #ifndef PU_RX
    #define PU_RX A,2
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT0_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL GIMSK
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK0
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR GIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE0 
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF0
  #endif

#elif defined(__AVR_ATtiny261__) || defined(__AVR_ATtiny461__) || defined(__AVR_ATtiny861__) || defined(__AVR_ATtiny26__)
  #ifndef PU_TX
    #define PU_TX A,6
  #endif
  #ifndef PU_RX
    #define PU_RX A,7
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT0_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL GIMSK
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK0
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR GIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE0 
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF0
  #endif
  #ifndef TCNT0
    #define TCNT0 TCNT0L // Compatibility hack for the OscillatorCalibration sketch
  #endif

#elif defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
  #ifndef PU_TX
    #define PU_TX A,1
  #endif
  #ifndef PU_RX
    #define PU_RX A,0
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT0_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL PCICR
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK0
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR PCIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE0
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF0
  #endif

#elif defined(__AVR_ATtiny48__) || defined(__AVR_ATtiny88__)
  #ifndef PU_TX
    #define PU_TX D,6
  #endif
  #ifndef PU_RX
    #define PU_RX D,7
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT2_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL PCICR
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK2
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR PCIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE2
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF2
  #endif
  #ifndef TCCR0B
    #define TCCR0B TCCR0A // Compatibility hack for the OscillatorCalibration sketch
  #endif

#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__) || defined(__AVR_ATtiny4313__)
  #ifndef PU_TX
    #define PU_TX D,1
  #endif
  #ifndef PU_RX
    #define PU_RX D,0
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT2_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL GIMSK
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK2
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR GIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE2
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF2
  #endif

#elif defined(__AVR_ATtiny1634__) || defined(__AVR_ATtiny1634R__)
  #ifndef PU_TX
    #define PU_TX B,0
  #endif
  #ifndef PU_RX
    #define PU_RX A,7
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT0_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL GIMSK
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK0
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR GIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE0 
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF0
  #endif

#elif defined(__AVR_ATtiny828__) || defined(__AVR_ATtiny828R__)
  #ifndef PU_TX
    #define PU_TX C,2
  #endif
  #ifndef PU_RX
    #define PU_RX C,3
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT2_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL PCICR
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK2
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR PCIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE2
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF2
  #endif

#elif defined(__AVR_ATtiny43__) || defined(__AVR_ATtiny43U__)
  #ifndef PU_TX
    #define PU_TX A,4
  #endif
  #ifndef PU_RX
    #define PU_RX A,4
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT0_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL GIMSK
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK0
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR GIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE0 
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF0
  #endif

#elif defined(__AVR_ATmega48__) || defined(__AVR_ATmega48P__)  || defined(__AVR_ATmega48PB__)  \
|| defined(__AVR_ATmega88__)    || defined(__AVR_ATmega88P__)  || defined(__AVR_ATmega88PB__)  \
|| defined(__AVR_ATmega168__)   || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega168PB__) \
|| defined(__AVR_ATmega328__)   || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__)
  #ifndef PU_TX
    #define PU_TX D,1
  #endif
  #ifndef PU_RX
    #define PU_RX D,0
  #endif
  #ifndef PU_PCINT_vect
    #define PU_PCINT_vect PCINT2_vect
  #endif
  #ifndef PU_PCINT_CTRL
    #define PU_PCINT_CTRL PCICR
  #endif
  #ifndef PU_PCMSK
    #define PU_PCMSK PCMSK2
  #endif
  #ifndef PU_PCIFR
    #define PU_PCIFR PCIFR
  #endif
  #ifndef PU_PCIE
    #define PU_PCIE PCIE2
  #endif
  #ifndef PU_PCIF
    #define PU_PCIF PCIF2
  #endif

#else
  #error "PicoUART does not support this target"
#endif

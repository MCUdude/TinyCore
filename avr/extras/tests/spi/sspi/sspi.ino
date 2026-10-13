// Slave SPI sketch: Receive 'hello slave' and send 'hi master'
// terminate when first null byte comes in and the start over
#include <SPI.h>
volatile byte indx;
volatile boolean done;

#ifdef __AVR_ATtiny828__
#include <avr/wdt.h>
// The ATtiny828 SPI slave works only if WDT is active
// wdt is disabled initially in order to avoid wdt loops!
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3"))) __attribute__((used));
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();
}
#endif

const int len = 12;
char recv[12] = "hello slave";
char send[12] = "hi master";
char buf[12];
bool success = false;

void setup (void) {
#ifdef __AVR_ATtiny828__
   wdt_enable(WDTO_8S);
#endif
   Serial.begin (9600);
   Serial.println(F("\n\r\n\rSPI Slave test sketch"));
   pinMode(MISO, OUTPUT); // have to send on master in so it set as output
   SPCR |= _BV(SPE); // turn on SPI in slave mode
   indx = 0; // buffer empty
   done = false;
   SPI.attachInterrupt(); // turn on interrupt
}

ISR (SPI_STC_vect) // SPI interrupt routine 
{ 
   byte c = SPDR; // read byte from SPI Data Register
   if (indx < sizeof buf) {
      buf[indx] = c; // save data in the next index in the array buff
      if (c == '\0') //check for the end of the word
        done = true;
      SPDR = send[indx++]; // reply string
   }
}

void loop (void) {
  if (done) {
    Serial.print(F("Received: '"));
    Serial.print(buf); 
    Serial.println(F("'"));
    if (strcmp(recv, buf) == 0) 
      Serial.println(F("SPI test successful"));
    else
      Serial.println(F("SPI test failed"));
    indx = 0; 
    done = false; //reset the process   
  }
}

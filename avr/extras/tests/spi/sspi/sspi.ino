// Slave SPI sketch: Receive 'hello slave' and send 'hi master'
// terminate when first null byte comes in and the start over
#include <SPI.h>
volatile byte indx;
volatile boolean done;

const int len = 12;
char recv[12] = "hello slave";
char send[12] = "hi master";
char buf[12];
bool success = false;

void setup (void) {
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
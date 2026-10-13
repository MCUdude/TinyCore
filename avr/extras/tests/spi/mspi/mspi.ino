// Master SPI sketch: Send 'hello slave' and receive 'hi master'
#include <SPI.h>

//#define TXBIT 4
//#define CORR_OSCCAL 0x68 // for 43U

const int len = 12;
char send[12] = "hello slave";
char succ[12] = "success";
char recv[12] = "hi master";
char buf[12];

void setup (void) {
#if FLASHEND > 2047
#ifdef CORR_OSCCAL
   OSCCAL = CORR_OSCCAL;
#endif
#ifdef TXBIT
   Serial.setTxBit(TXBIT);
#endif
   Serial.begin(9600); //set baud
   Serial.println(F("\n\r\n\rSPI Master test sketch"));
   Serial.flush();
#endif
   pinMode(SS, OUTPUT);
   digitalWrite(SS, HIGH); // disable Slave Select
   SPI.begin ();
   SPI.setClockDivider(SPI_CLOCK_DIV8);//divide the clock by 8
}

void loop (void) {
   int i;
   char c;
   digitalWrite(SS, LOW); // enable Slave Select
   // send test string
#if FLASHEND > 2027
   Serial.println(F("Sending: 'hello slave'"));
#endif
   for (i=0; i < len; i++) {
      c = SPI.transfer (send[i]);
      delayMicroseconds(10);
      if (i > 0) buf[i-1] = c; 
   }
#if FLASHEND > 2047
   Serial.println();
   Serial.print(F("Received: "));
   Serial.println(buf);
#endif
   digitalWrite(SS, HIGH); // disable Slave Select
#if FLASHEND > 2047
   if (strcmp(recv, buf) == 0)
     Serial.println(F("SPI test successful"));
   else
     Serial.println(F("SPI test failed"));
#else 
   if (strcmp(recv, buf) == 0)
      success();
#endif
   while (1);
}

void success()
{
   while (2);
}

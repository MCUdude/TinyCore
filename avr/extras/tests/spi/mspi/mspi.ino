// Master SPI sketch: Send 'hello slave' and receive 'hi master'
#include <SPI.h>

#define TXBIT 4

const int len = 12;
char send[12] = "hello slave";
char succ[12] = "success";
char recv[12] = "hi master";
char buf[12];

void setup (void) {
   //Serial.setTxBit(4);
   Serial.begin(9600); //set baud
   Serial.println(F("\n\r\n\rSPI Master test sketch"));
   digitalWrite(SS, HIGH); // disable Slave Select
   SPI.begin ();
   SPI.setClockDivider(SPI_CLOCK_DIV8);//divide the clock by 8
}

void loop (void) {
   int i;
   char c;
   digitalWrite(SS, LOW); // enable Slave Select
   // send test string
   Serial.println(F("Sending: 'hello slave'"));
   for (i=0; i < len; i++) {
      c = SPI.transfer (send[i]);
      delayMicroseconds(10);
      if (i > 0) buf[i-1] = c; 
   }
   Serial.println();
   Serial.print(F("Received: "));
   Serial.println(buf);
   digitalWrite(SS, HIGH); // disable Slave Select
   if (strcmp(recv, buf) == 0)
     Serial.println(F("SPI test successful"));
   else
     Serial.println(F("SPI test failed"));
   while (1);
}
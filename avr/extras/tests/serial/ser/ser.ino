// At 8 MHz internal osc., 57600 baud should work for software UART, and 250000 baud should work for hardware UART
#define BPS 9600
//#define CORR_OSCCAL 0x68

void setup() {
#ifdef CORR_OSCCAL  
  OSCCAL = CORR_OSCCAL;
#endif
  Serial.begin(BPS);
  Serial.println();
  Serial.println();
  Serial.println("Hello World!");
  Serial.println(F("01234567890"));
  Serial.println(F("ABCDEFGHIJKLMNOPQRSTUVWXYZ"));
  Serial.println(F("abcdefghijklmnopqrstuvwxyz"));
}

void loop() {
  Serial.print(F("Enter char: "));
  Serial.flush();
  while (Serial.available() == 0);  // Check if data is received
  byte incomingByte = Serial.read(); // Read one byte
  Serial.println((char)incomingByte);
#if FLASHEND > 2047
  Serial.printf("Got char %c and ASCII value %d\n\r", incomingByte, incomingByte);
#endif
}
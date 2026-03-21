
uint8_t orig_osccal;
void setup() {
  Serial.begin(9600);
  Serial.println("Hello World!");
  orig_osccal=OSCCAL;
}

void loop() {
  while (Serial.available() > 0) {   // Check if data is received
    uint8_t incomingByte = Serial.read(); // Read one byte
    Serial.printf("Got char %c and ASCII value %d, orig. OSCCAL: %x, act. OSCCAL: %x\n", incomingByte, incomingByte, orig_osccal, OSCCAL);

    OSCCAL--;
  }
  delay(1000);
}
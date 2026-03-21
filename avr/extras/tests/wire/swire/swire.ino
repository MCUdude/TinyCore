//Write slave: receive from master and mirror it back.

#define I2CADDR 8 
#define IASTR "8"

#include <Wire.h>

char buf[10];

void setup() {
  Wire.begin(I2CADDR);          // join I2C bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent);
  Serial.begin(9600);           // start serial for output
  Serial.println(F("\n\rWire slave test"));
}

void loop() {
  delay(10);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  int i = 0;
  Serial.print(F("Received: "));
  while (Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
    buf[i++] = c;
  }
  Serial.println();         // print the integer
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  buf[4] = '\0';
  Serial.print("Replying to master: '");
  Serial.print(buf);
  Serial.println("'");
  Wire.write(buf); // respond with message of 4 bytes
                   // as expected by master
  if (strcmp(buf,"END3") == 0) {
    Serial.println(F("Wire test successful"));
  }
}

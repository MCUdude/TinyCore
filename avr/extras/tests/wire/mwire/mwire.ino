// Write a value to the slave, then read it again
#define I2CADDR 8 
#define STRINGIFY(x) #x

#include <Wire.h>

bool succ = true;
char buf[5] = "MES0";

void setup() {
  Serial.begin(9600);
  Serial.println(F("\n\r\n\rWire Master test"));
  Wire.begin(); // join I2C bus (address optional for master)
  while (1) {
    Serial.println(F("Checking for I2C slave at addr " STRINGIFY(I2CADDR)));
    Wire.beginTransmission(I2CADDR);
    byte error = Wire.endTransmission();
    if (error == 0) break;
    delay(1500);
  }
  Serial.println(F("Slave device located"));
}

void loop() {
  int i;
  Serial.print(F("Sending ")); Serial.println(buf);
  Wire.beginTransmission(I2CADDR); // transmit to device #8
  Wire.write(buf);             // sends four bytes
  if (Wire.endTransmission() != 0) {  // stop transmitting
    succ = false;
    Serial.println(F("*** Sending failed"));
  }
  delay(100);
  Serial.println(F("Requesting 4 bytes"));
  Serial.print(F("Receiving: "));
  Wire.requestFrom(I2CADDR, 4);    // request 6 bytes from slave device #8
  i = 0;
  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
    if (c != buf[i]) {
      succ = false;
      Serial.println(F("*** Received byte wrong"));
    }
    if (i == 4) {
      succ = false;
      Serial.println(F("*** Too many bytes"));
    }
    i++;      
  }
  Serial.println();
  delay(500);
  if (strcmp(buf,"MES2") == 0)
    strcpy(buf,"END2");
  if (buf[3]++ == '3') {
    if (succ)
      Serial.println(F("Wire test successfull"));
    else
      Serial.println("*** Wire test failed");
    while(1);
  }
}

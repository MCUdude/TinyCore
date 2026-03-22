#define VCC 5000L // Vcc in millivolt
//#define TXBIT 3   // Alternate TX bit in port
//#define CORROSCCAL 0x6F // corrected OSCCAL -- if needed

// number of digital pins without the RESET pin, for ATtinyx8 only the one for the DIP footprint
#if defined(__AVR_ATtiny48__) || defined(__AVR_ATtiny88__)
const int iopins = 23;
#else
const int iopins = NUM_DIGITAL_PINS - 1; 
#endif

void setup()
{
#ifdef CORROSCCAL
  OSCCAL = CORROSCCAL;
  delay(500);
#endif
#ifdef TXBIT
  Serial.setTxBit(TXBIT);
#endif
  Serial.begin(9600);
  delay(1000);
  Serial.println(F("ADC Measurements"));
}

void loop()
{
  int pin = 0;
  long adc;

  for (pin=0; pin < iopins; pin++) {
    if (digitalPinToAnalogInput(pin) >= 0 && digitalPinToAnalogInput(pin) != NOT_A_PIN) { 
      adc = analogRead(pin);
      adc = analogRead(pin);
      Serial.print(pin);
      Serial.print(":");
      Serial.print((adc*VCC)/1023);
      Serial.print(" ");
      Serial.flush();
    }
  }
  Serial.println();
  delay(1000);
}
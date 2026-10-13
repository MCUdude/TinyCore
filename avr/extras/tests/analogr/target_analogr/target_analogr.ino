#define WAIT_MS 100
#define TIMEOUT_MS 90000
#define PULSE_MS 5
#define COMPIN 0
#define ANALOG_LOW 15
#define ANALOG_HIGH 1008

unsigned long start;
// number of digital pins without the RESET pin (usually the pin with the highest number),
// for ATtinyx8 only the one for the DIP footprint
// for ATtiny828 only those before the RESET pin
#if defined(__AVR_ATtiny48__) || defined(__AVR_ATtiny88__)
const int iopins = 23;
#elif defined(__AVR_ATtiny828__)
const int iopins = 26;
#else
const int iopins = NUM_DIGITAL_PINS - 1;
#endif

int phase;
int pin = 0;
volatile int r;

void setup()
{
  delay(2000);
  phase = 1;
  start = millis();
}

void loop()
{
  int i;

  if (millis() - start > TIMEOUT_MS)
    report_failure();
  switch (phase) {
  case 1: /* wait for HIGH on pin 0, and then wait 20 ms */
    if (digitalRead(COMPIN) == LOW)
      return;
    delay(20);
    start = millis();
    phase = 2;
    break;
  case 2: /* send number of i/O pins */
    send_pulses(iopins);
    delay(50);
    phase = 3;
    start = millis();
    break;
  case 3: /* send next ADC pin number or termination signal = iopins + 1 */
    delay(150);
    while (pin < iopins) {
      if (digitalPinToAnalogInput(pin) >= 0 && digitalPinToAnalogInput(pin) != NOT_A_PIN) break;
      pin++;
    }
    if (pin >= iopins) { // report
      send_pulses(iopins+1);
      for (int i = 0; i < iopins; i++) pinMode(i, INPUT);
      digitalWrite(COMPIN, LOW);
      pinMode(COMPIN, OUTPUT);
      while (1);
    } else {
       send_pulses(pin+1);
    }
    start = millis();
    phase = 4;
    break;
  case 4: /* check for LOW and HIGH values */
    delay(10);
    r = analogRead(pin);
    start = millis();
    while (analogRead(pin) > ANALOG_LOW && millis() - start < WAIT_MS);
    if (analogRead(pin) > ANALOG_LOW) report_failure();
    delay(10);
    while (analogRead(pin) < ANALOG_HIGH && millis() - start < WAIT_MS);
    if (analogRead(pin) < ANALOG_HIGH) report_failure();
    delay(200);
    pin++;
    phase = 3;
    start = millis();
    break;
  default:
    report_failure();
    break;
  }
}

void report_failure()
{
  delay(200);
  send_pulses(iopins+1);
  for (int i = 0; i < iopins; i++) pinMode(i, INPUT);
  while (1);
}

void send_pulses(int num)
{
  for (int i = 0; i < num; i++) {
    digitalWrite(COMPIN, LOW);
    pinMode(COMPIN, OUTPUT);
    delay(5);
    pinMode(COMPIN, INPUT);
    delay(5);
  }
  delay(20);
}

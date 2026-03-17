#define WAIT 100
#define TIMEOUT_MS 90000
#define PULSE_MS 5
#define INTERVAL_MS 20
#define COMPIN 0

unsigned long start;
// number of digital pins without the RESET pin, for ATtinyx8 only the one for the DIP footprint
#if defined(__AVR_ATtiny48__) || defined(__AVR_ATtiny88__)
const iopins = 23;
#else
const int iopins = NUM_DIGITAL_PINS - 1; 
#endif
int phase;
int pin = 0;

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
  case 3: /* send next PWM pin number or termination signal = iopins + 1 */
    delay(150);
    while (pin < iopins) {
      if digitalPinHasPWM(pin) break;
      pin++;
    }
    if (pin >= iopins) { // report success
      send_pulses(iopins+1);
      for (int i = 0; i < iopins; i++) pinMode(i, INPUT);
      while (1);
    } else {
       send_pulses(pin+1);
    }
    start = millis();
    phase = 4;
    break;
  case 4: /* execute analogwrite commands */
    pinMode(pin, OUTPUT);
    analogWrite(pin, 10);
    delay(INTERVAL_MS);
    analogWrite(pin, 50);
    delay(INTERVAL_MS);
    analogWrite(pin, 90);
    delay(INTERVAL_MS);
    analogWrite(pin, 100);
    delay(INTERVAL_MS);
    pinMode(pin, INPUT);
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
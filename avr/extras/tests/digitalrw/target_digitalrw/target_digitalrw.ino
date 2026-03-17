#define WAIT 100
#define TIMEOUT_MS 90000
#define PULSE_MS 5
#define COMPIN 0

unsigned long start;
// number of digital pins without the RESET pin, for ATtinyx8 only the one for the DIP footprint
#if defined(__AVR_ATtiny48__) || defined(__AVR_ATtiny88__)
const iopins = 23;
#else
const int iopins = NUM_DIGITAL_PINS - 1; 
#endif
int phase, subphase;
int reset_pin = -1;

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
  case 2:
    send_pulses();
    delay(20);
    phase = 3;
    start = millis();
    break;
  case 3:
    for (i = 0; i < iopins; i++) pinMode(i, INPUT_PULLUP);
    delay(100);
    phase = 4;
    start = millis();
    break;
  case 4:
    for (i = 0; i < iopins; i++)
      if (digitalRead(i) == LOW)
        report_failure();
    phase = 5;
    start = millis();
    break;
  case 5:
    for (i = 0; i < iopins; i++) {
      digitalWrite(i, HIGH);
      pinMode(i, OUTPUT);
    }
    phase = 6;
    start = millis();
    break;
  case 6:
    for (i = 0; i < iopins; i++) {
      digitalWrite(i, LOW);
      delay(100);
    }
    delay(100);
    phase = 7;
    start = millis();
    break;
  case 7:
    for (i = 0; i < iopins; i++) pinMode(i, INPUT);
    phase = 10;
    subphase = 0;
    start = millis();
    break;
  case 10: /* Now wait for switching to LOW one by one */
    for (i = 0; i <= subphase; i++) 
      if (digitalRead(i) == HIGH) return;
    for (i = subphase+1; i < iopins; i++)
      if (digitalRead(i) == LOW) return;
    subphase++;
    if (subphase >= iopins) {
      subphase = -1;
      phase = 12;
      start = millis();
    }
    break;
  case 12:
    pinMode(COMPIN, OUTPUT);
    digitalWrite(COMPIN, LOW);
    while (1);
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

void send_pulses()
{
  for (int i = 0; i < iopins; i++) {
    digitalWrite(COMPIN, LOW);
    pinMode(COMPIN, OUTPUT);
    delay(5);
    pinMode(COMPIN, INPUT);
    delay(5);
  }
}
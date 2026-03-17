#define TIMEOUT_MS 30000
#define BLINKFAST_MS 100
#define BLINKSLOW_MS 1000
#define WAITFORHIGH_MS 300
#define MAXPULSE_MS 15
#define HPIN(p) (p>=LED_BUILTIN-2 ? p+3 : p+2)
#define COMPIN 2

unsigned long start;
int phase, subphase;
int iocount;
int lowcount = 0;


void setup()
{
  Serial.begin(115200);
  phase = 1;
  subphase = -1;
  Serial.println(F("Phase 1: Setup communication pin"));
  iocount = 0;
  start = millis();
}

void loop()
{
  int i;
  if (millis() - start > TIMEOUT_MS) 
    report_failure();
  switch (phase) {
  case 1:
    pinMode(COMPIN, INPUT_PULLUP);
    start = millis();
    phase = 2;
    Serial.println(F("Phase 2: Get I/O pin number"));
    break;
  case 2: /* wait for negative pulses telling us number of GPIOs */
    if (digitalRead(COMPIN) == HIGH)
      return;
    iocount = count_pulses();
    pinMode(COMPIN, INPUT);
    Serial.print(F("Number of I/O pins: "));
    Serial.println(iocount);
    phase = 3;
    Serial.println(F("Phase 3: Wait for all pins to go high"));
    start = millis();
    break;
  case 3: /* wait for all GPIOs of the target to go high */
    for (i = 0; i < iocount; i++) {
      Serial.print(F("Check pin: ")); Serial.println(i);
      if (digitalRead(HPIN(i)) == LOW) {
        Serial.print(F("This target pin is still low: "));
        Serial.println(i);
        if (lowcount++ > 10) 
          report_failure();
        return;
      }
    }
    phase = 6;
    Serial.println(F("Phase 6: Waiting for all pins to go low one by one"));
    subphase = 0;
    start = millis();
    break;
  case 6: /* Now wait for switching to LOW one by one */
    for (i = 0; i <= subphase; i++) 
      if (digitalRead(HPIN(i)) == HIGH) {
        return;
      }
    Serial.print(F("Target pin now LOW: "));
    Serial.println(subphase);
    for (i = subphase+1; i < iocount; i++)
      if (digitalRead(HPIN(i)) == LOW) {
        Serial.print(F("Target LOW too early: "));
        Serial.println(i);
        report_failure();
      }
    subphase++;
    if (subphase >= iocount) {
      subphase = -1;
      phase = 8;
      Serial.println(F("Now switch all pins to INPUT_PULLUP"));
      start = millis();
    }
    break;
  case 8: /* Switch all pins to INPUT_PULLUP */
    for (i = 0; i < iocount; i++) pinMode(HPIN(i), INPUT_PULLUP);
    phase = 9;
    Serial.println(F("Phase 9: Wait for all pins to be HIGH inputs"));
    start = millis();
    break;
  case 9: /* wait for all pins to read HIGH */
    for (i = 0; i < iocount; i++)
      if (digitalRead(HPIN(i)) == LOW) {
        return;
      }
    phase = 10;
    Serial.println(F("Phase 10: We now switch all pins to LOW, one by one"));
    start = millis();
    break;
  case 10: /* switch all pins to LOW, one by one */
    for (i = 0; i < iocount; i++) {
      Serial.print(F("Switch target pin to LOW: "));
      Serial.println(i);
      pinMode(HPIN(i), OUTPUT);
      digitalWrite(HPIN(i), LOW);
      delay(50);
    }
    phase = 11;
    Serial.println(F("Phase 11: Switch all pins to INPUT"));
    start = millis();
    break;
  case 11:
    delay(10);
    for (i = 0; i < iocount; i++) pinMode(HPIN(i), INPUT);
    phase = 12;
    Serial.print(F("Phase 12: Wait for LOW signal from target on target pin "));
    Serial.println(COMPIN - 2);
    start = millis();
    break;
  case 12:
    pinMode(COMPIN, INPUT_PULLUP);
    while (millis() - start < WAITFORHIGH_MS)
      if (digitalRead(COMPIN) == LOW)
        report_success();
    report_failure();
    break;
  default:
    report_failure();
  }
}
    

void report_success()
{
  Serial.println(F("Digital read/write test successfully completed!"));
  bool on = false;
  for (int i = 0; i < iocount; i++) pinMode(HPIN(i), INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  start = millis();
  while (1) {
    if (millis() - start > BLINKSLOW_MS) {
      on = !on;
      digitalWrite(LED_BUILTIN, on);
      start = millis();
    }
  }
}

void report_failure()
{
  bool on = false;
  for (int i = 0; i < iocount; i++) pinMode(HPIN(i), INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.print(F("*** Failure in phase "));
  Serial.println(phase);
  if (subphase >= 0) {
    Serial.print(F("*** before having success on pin "));
    Serial.print(subphase);
  }
  while (1) {
    if (millis() - start > BLINKFAST_MS) {
      on = !on;
      digitalWrite(LED_BUILTIN, on);
      start = millis();
    }
  }
}

// count the number of negative 5ms pulses
// when we start, the first pulse has started
int count_pulses()
{
  int pulses = 1;
  bool timeout = false;
  unsigned long lastedge;
  
  while (1) {
    lastedge = millis();
    // wait for raising edge
    while ((digitalRead(COMPIN) == LOW) && (millis() - lastedge < MAXPULSE_MS));
    if (digitalRead(COMPIN) == LOW) 
      report_failure(); // if timeout, something went wrong
    lastedge = millis();
    // now wait for falling edge
    while ((digitalRead(COMPIN) == HIGH) && (millis() - lastedge < MAXPULSE_MS));
    if (digitalRead(COMPIN) == HIGH) // timeout means end of message
      break;
    pulses++; // new falling edge
  }
  return pulses;
}
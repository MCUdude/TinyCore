#define TIMEOUT_MS 30000
#define BLINKFAST_MS 100
#define BLINKSLOW_MS 1000
#define INTERVAL_MS 50
#define MAXPULSE_MS 15
#define HPIN(p) (p>=LED_BUILTIN-2 ? p+3 : p+2)
#define COMPIN 2


unsigned long start;
int phase, lastadc, adc;
int iocount;



void setup()
{
  Serial.begin(115200);
  phase = 1;
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
    Serial.println(F("Phase 3: Wait for ADC pin number"));
    start = millis();
    adc = -1;
    break;
  case 3: /* wait for adc pin number */
    pinMode(COMPIN, INPUT_PULLUP); 
    if (digitalRead(COMPIN) == HIGH)
      return;
    lastadc = adc;
    adc = count_pulses() - 1; // number is one to high to allow for pin 0
    pinMode(COMPIN, INPUT); 
    if (adc == iocount) { // we are ready
      pinMode(COMPIN, INPUT_PULLUP); 
      delay(100);
      if (digitalRead(COMPIN) == LOW) report_success();
      else report_failure();
    }
    Serial.print(F("ADC pin to test: "));
    Serial.println(adc);
    phase = 4;
    start = millis();
    Serial.println(F("Phase 4: Check ADC pin"));
    break;
  case 4:
    digitalWrite(HPIN(adc), LOW);
    pinMode(HPIN(adc), OUTPUT);
    delay(50);
    digitalWrite(HPIN(adc), HIGH);
    delay(50);
    pinMode(HPIN(adc), INPUT);
    start = millis();
    phase = 3;
    break;
  default:
    report_failure();
  }
}

void report_success()
{
  Serial.println(F("Analog read test successfully completed!"));
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
  Serial.print(F("*** after testing pin "));
  Serial.println(lastadc);
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

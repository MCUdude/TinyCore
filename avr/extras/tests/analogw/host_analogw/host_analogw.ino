#define TIMEOUT_MS 30000
#define BLINKFAST_MS 100
#define BLINKSLOW_MS 1000
#define WAITFORPULSE_uS 10000UL
#define PWMTIMEOUT_MS 100
#define MAXPULSE_MS 15
#define HPIN(p) (p>=LED_BUILTIN-2 ? p+3 : p+2)
#define COMPIN 2

unsigned long start;
int phase, pwm;
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
    Serial.println(F("Phase 3: Wait for PWM pin number"));
    start = millis();
    break;
  case 3: /* wait for PWM pin number */
    pinMode(COMPIN, INPUT_PULLUP); 
    if (digitalRead(COMPIN) == HIGH)
      return;
    pwm = count_pulses() - 1; // number is one to high to allow for pin 
    pinMode(COMPIN, INPUT); 
    if (pwm >= iocount) { // we are ready
      report_success();
    }
    Serial.print(F("PWM pin to test: "));
    Serial.println(pwm);
    phase = 4;
    start = millis();
    Serial.println(F("Phase 4: Check PWM pin"));
    break;
  case 4:
    pinMode(pwm, INPUT);
    if (!waitfor(pwm, 10)) report_failure();
    if (!waitfor(pwm, 50)) report_failure();
    if (!waitfor(pwm, 90)) report_failure();
    if (!waitfor(pwm,100)) report_failure();
    delay(20);
    Serial.println("PWM check OK");
    start = millis();
    phase = 3;
    break;
  default:
    report_failure();
  }
}

bool waitfor(int pin, int percent)
{
  unsigned long start = millis();
  long low, high, duty;
  bool succ = false;

  while (millis() - start < PWMTIMEOUT_MS && !succ) {
    high = pulseIn(HPIN(pin), HIGH, WAITFORPULSE_uS);
    low = pulseIn(HPIN(pin), LOW, WAITFORPULSE_uS);
    if (high == 0 || low == 0) continue;
    duty = high*100/(high+low);
    if (duty*3 >= percent-20 && duty*3 <= percent + 20) {
      succ = true; 
    }
  }
  if (succ) { 
    high = pulseIn(HPIN(pin), HIGH, WAITFORPULSE_uS);  
    low = pulseIn(HPIN(pin), LOW, WAITFORPULSE_uS);
    duty = high*100/(high+low);
    Serial.print(F("PWM check ")); 
    Serial.print(percent);
    Serial.print(F("%: "));
    Serial.print(duty);
    Serial.println(F("% duty cycle"));
  }
  return succ;
}


void report_success()
{
  Serial.println(F("Analog write test successfully completed!"));
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
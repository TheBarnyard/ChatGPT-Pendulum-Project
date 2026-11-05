// ArduinoSlaveRamp.ino: a copy of the team's ArduinoSlave.ino (Code/ArduinoSlave, not changed)
// for the cart-jolt tests, 2026-11-05.
//
// Same pins, same wiring, same serial protocol, same step counting. ONE difference:
// ArduinoSlave jumps to each new step rate the instant a command arrives (a speed staircase:
// every 10 ms the motor is asked for an instant speed change, which a stepper can only follow
// with a torque spike). This sketch ramps the step rate linearly from the old to the new value
// over RAMP_MS (one control period), updated every 1 ms from Timer1. The cart then gets a
// constant acceleration through each period, which is what the swing-up planner assumes.
//
// Expect about 5 ms more effective delay than ArduinoSlave; if the swing-up drifts, try
// CONTROL_LATENCY_MS 10 in the Pi script. To go back, flash the original ArduinoSlave.ino.
//
// Pins (as ArduinoSlave): D5 = STEP (OC3A, Timer3 toggle), D7 = DIR, D9 = ENA (LOW = enabled),
// D18 (INT3) counts the step pulses (wire D5 -> D18), D22/D27 limit switches, D23 stop button.

const uint8_t RAMP_MS = 10;               // ramp length, ms (1..50). 1 = behaves like ArduinoSlave
const int32_t RATE_SCALE = 16;            // rates are kept in 1/16 steps/s
const int32_t TIMER_HZ_X = 125000L * 16;  // step rate = 125000 / (N + 1) steps/s
const int32_t CREEP_RATE = 31;            // 1.9 steps/s (= N 65535, the "stopped" creep), x16

volatile int16_t position = 0;
volatile int8_t direction = 0;
int16_t positionTrue = 0;

volatile int32_t rateNow = 0;      // applied signed rate (x16); > 0 = DIR HIGH = direction -1
volatile int32_t rateTarget = 0;
volatile int32_t rateStep = 0;
volatile uint8_t rampLeft = 0;

int32_t message = 65535;

void motorInit();
void rampTimerInit();
void interruptInit();
void applyRate(int32_t r);
void command(int32_t n);

void setup() {
  motorInit();
  interruptInit();
  applyRate(CREEP_RATE);

  pinMode(22, INPUT_PULLUP);   // limit switches
  pinMode(27, INPUT_PULLUP);
  pinMode(23, INPUT_PULLUP);   // stop button

  digitalWrite(9, HIGH);       // disable motor

  Serial.begin(115200);
  while (!Serial) {}
  while (Serial.available() <= 3) {}   // wait for the Pi's first command

  digitalWrite(9, LOW);        // enable motor
  rampTimerInit();
}

void loop() {
  if (digitalRead(22) == 0 || digitalRead(27) == 0 || digitalRead(23) == 0) {
    digitalWrite(9, HIGH);     // disable motor
    TIMSK1 = 0;                // stop ramping
    TCCR3B = 0x00;             // stop step pulses
    while (1) {}
  }

  if (Serial.available() >= 4) {
    Serial.readBytes((char*)&message, 4);
    if (message == (int32_t)0xFFFFFFFF) {   // termination: timer off, motor disabled
      TIMSK1 = 0;
      TCCR3B = 0x00;
      digitalWrite(9, HIGH);
    } else {
      command(message);
    }
    noInterrupts();
    positionTrue = position;   // atomic copy (the ISR changes it)
    interrupts();
    Serial.write(positionTrue >> 8);
    Serial.write(positionTrue);
  }
}

// Same meaning as ArduinoSlave: N > 0 -> DIR HIGH, |N| >= 65535 -> creep, rate = 125000/(|N|+1).
void command(int32_t n) {
  if (digitalRead(9) == 1) n = 65535;      // motor disabled: no stepping
  int32_t mag = n < 0 ? -n : n;
  int32_t r;
  if (mag == 0) return;
  if (mag >= 65535) r = CREEP_RATE;
  else r = TIMER_HZ_X / (mag + 1);
  if (n < 0) r = -r;
  noInterrupts();
  rateTarget = r;
  rateStep = (r - rateNow) / RAMP_MS;
  rampLeft = RAMP_MS;
  interrupts();
}

// Called with interrupts off (from the Timer1 ISR or setup).
void applyRate(int32_t r) {
  int32_t mag = r < 0 ? -r : r;
  uint16_t top;
  if (mag <= CREEP_RATE) top = 65535;
  else {
    int32_t n = TIMER_HZ_X / mag - 1;
    if (n < 1) n = 1;
    if (n > 65535) n = 65535;
    top = (uint16_t)n;
  }
  if (r > 0) { PORTH |= (1 << PH4); direction = -1; }        // D7 = PH4: DIR HIGH
  else if (r < 0) { PORTH &= ~(1 << PH4); direction = 1; }   // DIR LOW
  ICR3 = top;
  if (TCNT3 > ICR3) TCNT3 = ICR3 - 1;   // as ArduinoSlave: do not wait for a timer wrap
}

ISR(TIMER1_COMPA_vect) {   // every 1 ms
  if (rampLeft == 0) return;
  rampLeft--;
  if (rampLeft == 0) rateNow = rateTarget;
  else rateNow += rateStep;
  applyRate(rateNow);
}

void motorInit() {
  TCCR3A = 0x00;
  TCCR3B = 0x00;
  TCNT3 = 0;
  OCR3A = 0;
  OCR3B = 0;
  TIMSK3 = 0x00;
  TIFR3 = 0x00;
  TCCR3A = (1 << COM3A0);                                            // toggle OC3A (D5) on match
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31) | (1 << CS30);  // CTC, TOP = ICR3, /64

  pinMode(9, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(9, LOW);
  digitalWrite(7, LOW);
}

void rampTimerInit() {     // Timer1: CTC 1 kHz (16 MHz / 64 / 250)
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
  OCR1A = 249;
  TCNT1 = 0;
  TIMSK1 = (1 << OCIE1A);
  interrupts();
}

void interruptInit() {
  EICRA = (1 << ISC31);    // INT3 (D18) on the falling edge
  EIMSK = (1 << INT3);
  sei();
}

ISR(INT3_vect) {
  position = position + direction;
}

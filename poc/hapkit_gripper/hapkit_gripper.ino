/*
 * Hapkit firmware for the SO-101 gripper PoC. Milestone 4: streaming, a local wall, an impact transient.
 *
 * Serial, 115200 baud:
 *   Arduino -> PC  "# hapkit_gripper m4.1" once at boot, identifies this firmware
 *                  "P <xh>"                handle position in cm, every 5 ms
 *                  "# isr_max_us <n>"      longest ISR time in the last second
 *   PC -> Arduino  "W <x_c> <k>"           wall on: push right when xh < x_c, PWM = k * (x_c - xh)
 *                  "R"                     wall off
 *                  "C <freq_hz> <decay_s>" set the impact transient's shape; decay_s is the time to 1%
 *                  "I <amp>"               start a decaying sinusoid of PWM amplitude amp on the wall
 *
 * The transient is the hard-surface model from baseline case 9, with its amplitude chosen by the PC.
 * It uses a recursive oscillator so the ISR needs a few multiplies, not sin() and exp() every tick.
 *
 * The wall is rendered here in the 1 kHz ISR because the PC only sees the handle after USB delay.
 * It is off at boot, and turns itself off if no valid W arrives for WATCHDOG_MS.
 *
 * xh is relative: 0 is wherever the handle was at reset.
 * Pins, PWM, timer and encoder setup come from poc/HW3FinalTeleop.ino.
 */

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <math.h>
#include <stdlib.h>

// PWM must be on pin 9: Timer1 drives it at 20 kHz, with commands from -400 to 400.
const int MotIn1 = 5;
const int MotIn2 = 4;
const int pwm = 9;

const int encA = 2;
const int encB = 3;

Encoder encoder(encB, encA);

// Geometry
const float encoderResolution = 48.0;  // counts per motor revolution
const float rh = 8.5;                  // handle length [cm]
const float rp = 0.5;                  // motor pulley radius [cm]
const float rs = 7.5;                  // sector radius [cm]

// About 0.074 cm per count. Precomputed so the ISR needs only one float multiply.
const float CM_PER_COUNT = (rh * rp * 2.0 * PI) / (encoderResolution * rs);

const unsigned long STREAM_PERIOD_US = 5000;
const unsigned long REPORT_PERIOD_MS = 1000;

// Wall limits. The current hardware has run a static wall at 300 (baseline case 2), and 600 against
// a moving virtual mass (case A). Raise this toward 600 only after testing.
const float K_MAX_PWM_PER_CM = 300.0;
const float MAX_WALL_POSITION_CM = 30.0;  // far beyond the handle's travel; rejects garbage
const unsigned int WATCHDOG_MS = 250;     // ISR ticks at 1 kHz

// Transient limits and defaults. The PC normally sets the shape with a C command.
const float TRANSIENT_DEFAULT_FREQ_HZ = 25.0;
const float TRANSIENT_DEFAULT_DECAY_S = 0.12;
const float TRANSIENT_MAX_FREQ_HZ = 200.0;
const float TRANSIENT_MIN_DECAY_S = 0.01;
const float TRANSIENT_MAX_DECAY_S = 1.0;
const float TRANSIENT_MAX_AMP = 400.0;

volatile float xh = 0;
volatile bool wallActive = false;
volatile float wallX = 0;
volatile float wallK = 0;
volatile unsigned int commandAgeMs = 0;
volatile unsigned int isrMaxUs = 0;

// Decaying sinusoid: y[n] = a1 * y[n-1] + a2 * y[n-2]
volatile float transientA1 = 0;
volatile float transientA2 = 0;
volatile float transientFirst = 0;  // first output per unit amplitude
volatile unsigned int transientLengthMs = 0;
volatile float transientY1 = 0;
volatile float transientY2 = 0;
volatile unsigned int transientTicksLeft = 0;

unsigned long lastSend = 0;
unsigned long lastReport = 0;

const int LINE_SIZE = 32;
char line[LINE_SIZE];
int lineLength = 0;
bool lineOverflowed = false;

void setup() {
  pinMode(MotIn1, OUTPUT);
  pinMode(MotIn2, OUTPUT);
  pinMode(pwm, OUTPUT);

  digitalWrite(MotIn1, HIGH);
  digitalWrite(MotIn2, LOW);

  Serial.begin(115200);
  Serial.println(F("# hapkit_gripper m4.1"));

  // 20 kHz phase and frequency correct PWM on OC1A (pin 9), TOP = ICR1, prescaler 1.
  // freq = 16 MHz / (2 * 1 * 400) = 20 kHz
  TCCR1A = (1 << COM1A1);
  TCCR1B = (1 << WGM13) | (1 << CS10);
  ICR1 = 400;
  OCR1A = 0;

  // 1 kHz ISR: Timer2 in CTC mode, prescaler 64, 16 MHz / 64 / 250 = 1 kHz
  cli();
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);
  OCR2A = 249;
  TIMSK2 |= (1 << OCIE2A);
  sei();

  setTransientShape(TRANSIENT_DEFAULT_FREQ_HZ, TRANSIENT_DEFAULT_DECAY_S);

  lastSend = micros();
  lastReport = millis();
}

void setMotor(int command) {
  if (command > 400) command = 400;
  if (command < -400) command = -400;

  if (command >= 0) {
    digitalWrite(MotIn1, HIGH);
    digitalWrite(MotIn2, LOW);
    OCR1A = command;
  } else {
    digitalWrite(MotIn1, LOW);
    digitalWrite(MotIn2, HIGH);
    OCR1A = -command;
  }
}

void setWall(float x, float k) {
  if (k > K_MAX_PWM_PER_CM) k = K_MAX_PWM_PER_CM;

  // The ISR reads these, and a float is 4 bytes, so update them together with interrupts off.
  noInterrupts();
  wallX = x;
  wallK = k;
  commandAgeMs = 0;
  wallActive = true;
  interrupts();
}

void clearWall() {
  noInterrupts();
  wallActive = false;
  transientTicksLeft = 0;
  interrupts();
}

void setTransientShape(float freqHz, float decayS) {
  float theta = 2.0 * PI * freqHz * 0.001;
  float r = exp(log(0.01) * 0.001 / decayS);
  float a1 = 2.0 * r * cos(theta);
  float a2 = -r * r;
  float first = r * sin(theta);
  unsigned int lengthMs = (unsigned int)(decayS * 1000.0);

  noInterrupts();
  transientA1 = a1;
  transientA2 = a2;
  transientFirst = first;
  transientLengthMs = lengthMs;
  interrupts();
}

void startTransient(float amp) {
  noInterrupts();
  if (wallActive) {
    transientY2 = 0;
    transientY1 = amp * transientFirst;
    transientTicksLeft = transientLengthMs;
  }
  interrupts();
}

// Parse one number from text, requiring it to be finite and followed by the given character.
bool parseNumber(char *text, char terminator, float *value, char **next) {
  char *end;
  float parsed = strtod(text, &end);
  if (end == text || *end != terminator || isnan(parsed) || isinf(parsed)) {
    return false;
  }
  *value = parsed;
  *next = end + 1;
  return true;
}

// Malformed lines are ignored and do not reset the watchdog.
void handleLine(char *text) {
  if (text[0] == 'R' && text[1] == '\0') {
    clearWall();
    return;
  }
  if (text[1] != ' ') {
    return;
  }

  char *next;
  float first;
  float second;
  switch (text[0]) {
    case 'W':
      if (parseNumber(text + 2, ' ', &first, &next) && parseNumber(next, '\0', &second, &next)
          && first >= -MAX_WALL_POSITION_CM && first <= MAX_WALL_POSITION_CM && second >= 0) {
        setWall(first, second);
      }
      break;

    case 'C':
      if (parseNumber(text + 2, ' ', &first, &next) && parseNumber(next, '\0', &second, &next)
          && first > 0 && first <= TRANSIENT_MAX_FREQ_HZ
          && second >= TRANSIENT_MIN_DECAY_S && second <= TRANSIENT_MAX_DECAY_S) {
        setTransientShape(first, second);
      }
      break;

    case 'I':
      if (parseNumber(text + 2, '\0', &first, &next) && first >= 0 && first <= TRANSIENT_MAX_AMP) {
        startTransient(first);
      }
      break;
  }
}

void readCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (!lineOverflowed) {
        line[lineLength] = '\0';
        handleLine(line);
      }
      lineLength = 0;
      lineOverflowed = false;
    } else if (c != '\r') {
      if (lineLength < LINE_SIZE - 1) {
        line[lineLength++] = c;
      } else {
        lineOverflowed = true;
      }
    }
  }
}

void loop() {
  readCommands();

  // Fixed steps from the previous send, so small delays do not accumulate into a slower rate.
  if (micros() - lastSend >= STREAM_PERIOD_US) {
    lastSend += STREAM_PERIOD_US;

    // A float is 4 bytes, so the ISR could change it partway through an unprotected copy.
    noInterrupts();
    float x = xh;
    interrupts();

    Serial.print(F("P "));
    Serial.println(x, 2);
  }

  if (millis() - lastReport >= REPORT_PERIOD_MS) {
    lastReport += REPORT_PERIOD_MS;

    noInterrupts();
    unsigned int maxUs = isrMaxUs;
    isrMaxUs = 0;
    interrupts();

    Serial.print(F("# isr_max_us "));
    Serial.println(maxUs);
  }
}

// Encoder::read() (v1.4.4) re-enables interrupts when it returns, so the rest of this ISR can be
// preempted. That is safe only while the ISR finishes well inside its 1 ms period, so its longest
// time is measured and reported.
ISR(TIMER2_COMPA_vect) {
  unsigned long start = micros();

  float x = encoder.read() * CM_PER_COUNT;
  xh = x;

  int command = 0;
  if (wallActive) {
    if (++commandAgeMs >= WATCHDOG_MS) {
      wallActive = false;
      transientTicksLeft = 0;
    } else {
      float force = 0;
      if (x < wallX) {
        force = wallK * (wallX - x);
      }
      if (transientTicksLeft > 0) {
        transientTicksLeft--;
        float y = transientA1 * transientY1 + transientA2 * transientY2;
        transientY2 = transientY1;
        transientY1 = y;
        force += y;
      }
      command = (int)force;
    }
  }
  setMotor(command);

  unsigned int elapsed = micros() - start;
  if (elapsed > isrMaxUs) {
    isrMaxUs = elapsed;
  }
}

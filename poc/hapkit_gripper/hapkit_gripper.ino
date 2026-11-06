/*
 * Hapkit firmware for the SO-101 gripper PoC. Milestone 2: position streaming only.
 *
 * The motor is never driven: the ISR always calls setMotor(0), and no code here can render a force.
 * Pins, PWM, timer and encoder setup come from poc/HW3FinalTeleop.ino.
 *
 * Serial, 115200 baud:
 *   Arduino -> PC  "# hapkit_gripper m2"   once at boot, identifies this firmware
 *                  "P <xh>"                handle position in cm, every 5 ms
 *   PC -> Arduino  everything is read and discarded until commands exist (milestone 4)
 *
 * xh is relative: 0 is wherever the handle was at reset.
 */

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

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

volatile float xh = 0;
unsigned long lastSend = 0;

void setup() {
  pinMode(MotIn1, OUTPUT);
  pinMode(MotIn2, OUTPUT);
  pinMode(pwm, OUTPUT);

  digitalWrite(MotIn1, HIGH);
  digitalWrite(MotIn2, LOW);

  Serial.begin(115200);
  Serial.println(F("# hapkit_gripper m2"));

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

  lastSend = micros();
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

void loop() {
  // Discard input. The baseline sketch treated any byte as a mode change.
  while (Serial.available()) {
    Serial.read();
  }

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
}

// Encoder::read() (v1.4.4) re-enables interrupts when it returns, so the rest of this ISR can be
// preempted. That is safe only while the ISR finishes well inside its 1 ms period.
ISR(TIMER2_COMPA_vect) {
  xh = encoder.read() * CM_PER_COUNT;
  setMotor(0);
}

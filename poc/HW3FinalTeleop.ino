/*
 * ME 384 - Haptic and Teleoperated Systems
 * Assignment 3: Haptic Rendering on a 1-DoF Kinesthetic Haptic Device 
 *
 * Authors: Jatin Sikka and James Belanger
 *
 */

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

//PWM MUST BE ON PIN 9 for this version to run (PWM is configured on Timer1 which controls pin 9 and 10 pwm)

//This code is equivalent to all_environments.ino except with fast PWM enabled
//PWM commands are now scaled to 400 instead of 255

/*
=======================
Haptic Modes:
=======================
0 - Reset. 
1 - Spring
2 - Wall
3 - Wall with Damping
4 - Damper
5 - Nonlinear Friction !!
6 - Bump
7 - Better Bump !!
8 - Bump and Valley !!
9 - Hard Surface !!
A - Mass Spring Damper !!
========================
========================
*/


// Pin Declarations
const int MotIn1 = 5;
const int MotIn2 = 4;
const int pwm = 9;

const int encA = 2;
const int encB = 3;

Encoder encoder(encB, encA);

// Encoder / Geometry
const float encoderResolution = 48.0;

const float rh = 8.5;  // handle length [cm]
const float rp = 0.5;  // motor pulley radius [cm]
const float rs = 7.5;  // sector radius [cm]

// State Variables
volatile float pos = 0;

volatile float xh = 0;
volatile float xh_prev = 0;

volatile float vh = 0;
volatile float vh_prev = 0;

volatile float bump_h = 0;

float tPulley = 0;
float tS = 0;

// Timing / Filtering
const float dt = 0.001;
const float fc = 20.0;
const float r = exp(-2.0 * PI * fc * dt); 

// Force Command
volatile float motorCommand = 0;

// Environment Selection SELECT ENVIRONMENT HERESELECT ENVIRONMENT HERESELECT ENVIRONMENT HERESELECT ENVIRONMENT HERESELECT ENVIRONMENT HERESELECT ENVIRONMENT HERESELECT ENVIRONMENT HERESELECT ENVIRONMENT HERESELECT ENVIRONMENT HERESELECT ENVIRONMENT HERE
char mode = 'A'; // 5-Fric, 7-Bump, 8-BumpValley, 9-Hard Surface, A-MSD


// Evironment variables that get reset with 0 mode
float x_msd = 0.5;
float v_msd = 0;
float a_msd = 0;

float xi_msd = 0.5;

float Decay_Sine_Amplitude = 0;
int Decay_In_Wall_Flag = 0;
float Decay_Sine_Time = 0;

volatile float xm = 0.5;  // virtual mass position [cm] (spring rest position is 0.5), Case 'A'
volatile float vm = 0;    // virtual mass velocity [cm/s], Case 'A'
// ===================================================
// Setup
// ===================================================
void setup() {
  //initialize motor outputs
  pinMode(MotIn1, OUTPUT);
  pinMode(MotIn2, OUTPUT);
  pinMode(pwm, OUTPUT);

  //initialize motor direction
  digitalWrite(MotIn1, HIGH);
  digitalWrite(MotIn2, LOW);



  Serial.begin(115200);

  //set up PWM on Timer1 so it's faster and we don't hear the annoying hum anymore
  //this stuff is more in the weeds, please consult the ATmega328/P datasheet for more detailed info (p. ~170)
  //set phase and frequency correct pwm at 20kHz:
  TCCR1A = (1 << COM1A1);               //non-inverting PWM on OC1A (pin 9)
  TCCR1B = (1 << WGM13) | (1 << CS10);  //waveform generation mode 8 (phase/freq correct, ICR1 top), prescaler = 1
  ICR1 = 400;                           //ICR1 defines TOP in WGM8
  OCR1A = 0;                            //initialize at 0 duty cycle
  // the results of the above code is:
  // freq_PWM = freq_CPU / (2 * prescaler * TOP)
  // freq_PWM = 16,000,000 / (2 * 1 * 400) = 20,000 or 20 kHz

  // Setup 1kHz ISR
  cli();

  TCCR2A = 0;
  TCCR2B = 0;

  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);

  OCR2A = 249;

  TIMSK2 |= (1 << OCIE2A);

  sei();
}

// ===================================================
// Motor Driver
// ===================================================
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

// ===================================================
// Main Loop
// ===================================================
void loop() {

  if (Serial.available()) {
    mode = Serial.read();
  }

  switch (mode) {

    // =======================
    // Reset
    // =======================
    case '0':
      {
        motorCommand = 0;

        encoder.write(0);
        pos = 0;

        x_msd = xi_msd;
        v_msd = 0;
        a_msd = 0;

        Decay_In_Wall_Flag = 0;
        Decay_Sine_Time = 0;
        Decay_Sine_Amplitude = 0;

        break;
      }

    // =======================
    // Spring F = -k * xh
    // Bidirectional spring centered at x = 0.
    // =======================
    case '1':
      {
        const float k = 30;            // spring stiffness
        motorCommand = (-1) * k * xh;  // restoring force toward center
        break;
      }

    // =======================
    // Wall
    // Free space for xh > x_wall; spring force pushes back when xh < x_wall.
    // =======================
    case '2':
      {
        const float x_wall = 0.5;  // wall location [cm]
        const float k_wall = 300;  // wall stiffness
        // Note: max stable k changed after each cable/shaft replacement;
        // one cable snapped at k = 140 during testing.

        // CODE HERE!
        if ((xh - x_wall) > 0) {
          motorCommand = 0;  // not in contact
        } else {
          motorCommand = k_wall * (x_wall - xh);
        }
        break;
      }

    // =======================
    // Wall with Damping
    // Same wall as case 2, plus damping everywhere
    // =======================
    case '3':
      {
        const float x_wall_damped = 0.5;
        const float k_wall_damped = 80;
        const float b_wall_damped = 5;

        // CODE HERE!
        if ((xh - x_wall_damped) > 0) {
          motorCommand = -1 * b_wall_damped * vh;  // damping only
        } else {
          motorCommand = (k_wall_damped * (x_wall_damped - xh)) + (-1 * b_wall_damped * vh);  // spring + damping
        }
        break;
      }

    // =======================
    // Damper  F = -b * vh 
    // =======================
    case '4':
      {
        const float b = 5.0;
        motorCommand = -b * vh;
        break;
      }

    // =======================
    // Nonlinear Friction (Karnopp model)
    // |vh| < 1      : no force (dead zone)
    // 1 < |vh| < 4  : static, -/+Fc opposing motion
    // |vh| > 4      : dynamic, Cpn + damping (-bpn*vh)
    // =======================
    case '5':
      {
        const float Fc = 200.0;
        const float vth = 4.0;
        const float k = Fc / vth;
        const float Cpn = -50;  //change this
        const float bpn = 8.0;

        if ((vh > -1) && (vh < 1)) {
          motorCommand = 0;
        }                            // dead zone
        else if (vh < (-1 * vth)) {  //dynamic negative case
          motorCommand = -Cpn + (-1 * bpn * vh);
        } else if (vh > vth) {  //dynamic positive case
          motorCommand = (Cpn - bpn * vh);
        } else if ((vh < 0) && (vh > (-1 * vth))) {  //static negative
          motorCommand = Fc;
        } else if ((vh > 0) && (vh < vth)) {  //static positive
          motorCommand = -Fc;
        }
        break;
      }

    // =======================
    // Bump. (linear force profile)
    // Force is a linear function of position on each side of the bump peak.
    // =======================
    case '6':
      {
        float bump_location = 3;
        float bump_width = 4;
        float bump_force = 150;

        // CODE HERE!
        if (xh == bump_location) {
          motorCommand = 0;
        } else if (((xh - bump_location) < 0) && (xh > (bump_location - bump_width))) {  //ascending side
          motorCommand = -bump_force * (xh - (bump_location - (bump_width / 2)));
        } else if ((xh - bump_location) > 0 && (xh < (bump_location + bump_width / 2))) {  //descending side
          motorCommand = bump_force * (xh - bump_location);
        } else {                                                                          // peak
          motorCommand = 0;
        }
        break;
      }

      // =======================
      // Better Bump. (elliptical force profile)
      // Force magnitude follows an ellipse: h(x) = e * sqrt(w^2 - d^2).
      // This is smooth and symmetric about the peak, unlike case 6.
      // =======================

    case '7':
      {
        float bump_location = 4;
        float bump_width = 2.5;
        float bump_force = 180;
        float ellipse = 1.2;
        bump_h = ellipse * sqrt((bump_width * bump_width) - ((xh - bump_location) * (xh - bump_location)));

        // CODE HERE!
        if (xh == bump_location) {
          motorCommand = 0;      //peak
        } else if (((xh - bump_location) < 0) && (xh > (bump_location - bump_width))) {  // ascending: push back 
          motorCommand = -bump_force * (bump_h);
        } else if ((xh - bump_location) > 0 && (xh < (bump_location + bump_width))) {  // descending: push forward 
          motorCommand = bump_force * (bump_h);
        } else {
          motorCommand = 0;     // flat ground
        }
        break;
      }


    // =======================
    // Bump and Valley
    // Bump uses the case 7 profile. The valley is the same shape with the
    // forces reversed, so the handle is pulled toward the valley center.
    // =======================
    case '8':
      {
        // Bump (right side, x > 0) -- identical to case 6
        float bump_location = 4;
        float bump_width = 2.5;
        float bump_force = 170;
        float ellipse = 1.2;

        // Valley (left side, x < 0) -- identical shape, forces reversed
        float valley_location = -4;
        float valley_width = 2.5;
        float valley_force = 130;
        float valley_ellipse = 1.0;

        bump_h = ellipse * sqrt((bump_width * bump_width) - ((xh - bump_location) * (xh - bump_location)));
        float valley_h = valley_ellipse * sqrt((valley_width * valley_width) - ((xh - valley_location) * (xh - valley_location)));

        // CODE HERE!
        // ---------- Bump (same as case 6) ----------
        if (xh == bump_location) {
          motorCommand = 0;
        } else if (((xh - bump_location) < 0) && (xh > (bump_location - bump_width))) {  //ascent: push left
          motorCommand = -bump_force * (bump_h);
        } else if (((xh - bump_location) > 0) && (xh < (bump_location + bump_width))) {  //descent: push right
          motorCommand = bump_force * (bump_h);
        }
        // ---------- Valley (same as bump, forces reversed) ----------
        //else if (xh == valley_location){
        else if (abs(xh - valley_location) < 0.12) {
          motorCommand = 0;
        } else if (((xh - valley_location) < 0) && (xh > (valley_location - valley_width))) {  //left slope: push right, toward center
          motorCommand = valley_force * (valley_h);
        } else if (((xh - valley_location) > 0) && (xh < (valley_location + valley_width))) {  //right slope: push left, toward center
          motorCommand = -valley_force * (valley_h);
        }
        // ---------- Flat ----------
        else {
          motorCommand = 0;
        }
        break;
      }

    // =======================
    // Hard Surface (with decaying sinusoid)
    // =======================
    case '9':
      {
        const float x_wall_hard = 0.5;
        const float k_wall_hard = 300;

        const float Decay_Sine_Amplitude_Scale = 4.0;  // A: force per unit impact speed
        const float Decay_Sine_Amplitude_Max = 170.0;     // cap on vibration amplitude (N)
        const float Decay_Sine_Frequency = 20.0;        // f in Hz
        //const float Decay_Sine_Exponent = ;        //?
        const float d = 0.25;                         // time to decay (decrease maybe?)

        static float t = 0;         // time since impact s 
        static float v_impact = 0;  //imapact velocity

        if ((xh - x_wall_hard) > 0) {  // not in contact: reset impact timer
          t = 0;
          motorCommand = 0;
        } else {
          if (t == 0) {  // first moment of contact: store impact speed
            v_impact = min(-Decay_Sine_Amplitude_Scale * vh, Decay_Sine_Amplitude_Max);
          }
          t = t + 0.001;   // advance time since impact
          motorCommand = k_wall_hard * (x_wall_hard - xh) + Decay_Sine_Amplitude_Scale * v_impact * exp(log(0.01) * t / d) * sin(2 * PI * Decay_Sine_Frequency * t);
        }
        break;
      }

    // =======================
    // Mass Spring Damper
    // =======================
    case 'A':
      {
        const float m_msd  = 0.75; // 1.5
        const float b_msd  = 0.42; // 1.1  // zeta ≈ 0.06
        const float k_msd  = 16.7;  // 59  // f_n = (1/2pi)*sqrt(59/1.5) ≈ 1.0 Hz
        const float k_user = 600; // 300
        const float b_user = 3.0; // 3.0  // push-only contact damping (set 0 to compare)
        const float x_rest = 1.75; // 1.75 // ball's spring rest position [cm]

        static unsigned long lastUpdate = 0;
        unsigned long now = micros();
        unsigned long elapsed = now - lastUpdate;

        if (elapsed < 1000) break;
        lastUpdate = now;

        if (elapsed > 50000) {
          xm = x_rest;
          vm = 0;
          elapsed = 1000;
        }

        float dt_msd = elapsed * 0.000001;

        noInterrupts();
        float handlePosition = xh;
        float handleVelocity = vh;
        interrupts();

        float f = 0;
        if (handlePosition > xm) {
          f = k_user * (handlePosition - xm) + b_user * (handleVelocity - vm);
          if (f < 0) f = 0;
        }

        float a = (f - k_msd * (xm - x_rest) - b_msd * vm) / m_msd;
        vm = vm + a * dt_msd;
        xm = xm + vm * dt_msd;

        noInterrupts();
        motorCommand = -f;
        interrupts();

        break;
      }
  }

  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint >= 20) {
    lastPrint = now;

    Serial.print("|xh: ");
    Serial.print(xh);
    Serial.print(" |vh: |");
    Serial.print(vh);
    Serial.print(" |PWM: ");
    Serial.print(motorCommand);
    Serial.print(" |xm: ");
    Serial.print(xm);
    Serial.print(" |vm: ");
    Serial.println(vm);
    //Serial.print(bump_h);
    //Serial.print(" Height: ");
  }
}

// ===================================================
// 1 kHz Interrupt Service Routine
// ===================================================

ISR(TIMER2_COMPA_vect) {

  pos = encoder.read();

  tPulley =
    (pos / encoderResolution) * 2.0 * PI;

  tS =
    (tPulley * rp) / rs;

  xh =
    (rh * rp * tPulley) / rs;

  vh =
    r * vh_prev + (1.0 - r) * (xh - xh_prev) / dt;

  xh_prev = xh;
  vh_prev = vh;

  setMotor((int)motorCommand);
}

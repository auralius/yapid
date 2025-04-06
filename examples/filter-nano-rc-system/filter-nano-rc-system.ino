/**
 * A simple serial RC system with R = 20kΩ and C = 100uF
 * PWM signal from D3 (Timer-2) is applied to the RC system.
 * Voltage across the capacitor is measured by A0.
 *
 * https://auralius.github.io/yapid/#using-the-low-pass-filter
 */

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_1                   true 

#include "TimerInterrupt.h"
#include "yapid.h"

#define TIMER1_INTERVAL_MS             1

const int pwm_port = 3;  // TIMER-2   

// Create the PID controller
float Ts = 1e-3;
YAPID pid1(Ts); // for first-order filter demo
YAPID pid2(Ts); // for second-order filter demo

volatile float SV = 0.0;  // set value from 0 to 5 

// -----------------------------------------------------------------
void setup() {
  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, Timer1Handler);

  pinMode(pwm_port, OUTPUT); 
  analogWrite(pwm_port, 0);     

  Serial.begin(250000);
}

inline void tx()
{
  Serial.print(pid1.Now(), 3);
  Serial.print(",");

  Serial.print(pid1.X());
  Serial.print(",");

  Serial.print(pid1.Y());
  Serial.print(",");

  Serial.print(pid2.Y());
  Serial.print("\n");
}

void Timer1Handler()
{ 
  float pv = (float)analogRead(A0) * 5.0 / 1024.0;
  float pv_filtered1 = pid1.FOLP(0.1, pv);
  float pv_filtered2 = pid2.SOLP(10.0, pv);

  // Sinusoid, f=0.5Hz, peak-to-peak values: 0 to 1 volt
  float pwm = (sin(2*3.1415*0.5*pid1.Now()) + 1.0) / 5.0 * 255.0;
  analogWrite(pwm_port, (int)pwm);

  pid1.UpdateTime(); // just pid1, no need for pid2
}

void loop()
{
  // Serial transmit
  tx();
}

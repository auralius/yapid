/**
 * A simple serial RC system with R = 20kΩ and C = 100uF
 * Thus, system transfer function is: 1/(2s + 1).
 * PWM signal from D3 (Timer-2) is applied to the RC system (vi).
 * Voltage across the capacitor (vc) is measured by A0.
 *
 * https://auralius.github.io/yapid/#rc-system
 */
 
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_1                   true 

#include "TimerInterrupt.h"
#include "yapid.h"
 
#define TIMER1_INTERVAL_MS             1

const int pwm_port = 3;  // TIMER-2   

// Create the PID controller
float kp = 100.;  // kp
float ki = 100.0;  // ki
float kd = 10.0;   // kd
float N  = 100.0;   // derivative filter constant (Hz)
float Ts = 1e-3;
YAPID pid(Ts, kp, ki, kd, N);

volatile float SV = 0.0;  // set value


void setup() {
  Serial.begin(921600);

  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, Timer1Handler);
  
  pinMode(pwm_port, OUTPUT);  
  analogWrite(pwm_port, 0);     

  pid.SetOutputLimits(0., 255.)  ;

  // Wait till the capacitor discharges!
  while(analogRead(A0) > 10){
     delay(100);
  }
}


inline void rx()
{
  while(Serial.available()){
    int StringCount = 0;
    String input = Serial.readString();
    SV = input.toFloat();
  }
}


inline void tx()
{
  Serial.print(pid.Now(), 3);
  Serial.print(",");
  Serial.print(pid.SV());
  Serial.print(",");
  Serial.print(pid.PV());
  Serial.print(",");
  Serial.print(pid.P());
  Serial.print(",");
  Serial.print(pid.I());
  Serial.print(",");
  Serial.print(pid.D());
  Serial.print(",");
  Serial.print(pid.CO());
  Serial.print(",");
  Serial.print(pid.SAT_CO());
  Serial.print("\n");
}

void Timer1Handler()
{ 
  float pv = (float)analogRead(A0) * 5.0 / 1024.0;

  float co = pid.Compute1(SV, pv);
  //float co = pid.Compute2(SV, pv);
  
  analogWrite(pwm_port, (int)co);
  
  pid.UpdateTime();
}

void loop()
{
  // Serial receive
  rx();
  
  // Serial transmit
  tx();
  
  /*
  // Simple test scenario:
  if (pid.Now() < 15.0)
    SV = 0.0;
  else if ((pid.Now() > 15.0) && (pid.Now() < 30.0)) 
    SV = 4.0;
  else if (pid.Now() > 30.0) 
    SV = 1.0;
  */
}


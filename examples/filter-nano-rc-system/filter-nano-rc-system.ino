/**
 * A simple serial RC system with R = 10kΩ and C = 100uF
 * PWM signal from D3 (Timer-2) is applied to the RC system.
 * Voltage across the capacitor is measured by A0.
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
YAPID pid(Ts);

volatile float SV = 0.0;  // set value from 0 to 5 


void setup() {
  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, Timer1Handler);
  
  pinMode(pwm_port, OUTPUT); 
  TCCR2B = TCCR2B & 0b11111000 | 0x01;   
  analogWrite(pwm_port, 0);     

  pid.SetOutputLimits(0., 255.)  ;
  Serial.begin(250000);
}


void rx()
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
  Serial.print(pid.X());
  Serial.print(",");
  Serial.print(pid.Y());
  Serial.print("\n");
}

void Timer1Handler()
{ 
  float pv = (float)analogRead(A0) * 5.0 / 1024.0;
  //float pv_filtered = pid.FOLP(0.1, pv);
  float pv_filtered = pid.SOLP(10.0, pv);
  
  float pwm_command = SV / 5.0 * 255.0;
  analogWrite(pwm_port, (int)pwm_command);

  pid.UpdateTime();
}

void loop()
{
  // Serial receive
  rx();
  
  // Serial transmit
  tx();
}

/*
 * About the system : https://bit.ly/TemperatureControlDevice
 *
 * Uno has 3 timers: Timer 0, 1, and 2
 * Timer 0 is reserved by the Arduino for timing purposes.
 *
 * Heater 1 is using PWM pin #9.
 * Heater 2 is using PWM pin #10.
 * PWM pin #9 and #10 are part of Timer 1.
 *
 * We put our PID as a peridic task run by Timer 2.
 */
 
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_2                   true 

#include "TimerInterrupt.h"
#include "yapid.h"
 
#define TIMER2_INTERVAL_MS             1

// PWM ports that are connected to the heaters
const int pwm1_port       = 9;     // PWM of heater #1, Timer 1A
const int pwm2_port       = 10;    // PWM of heater #2, Timer 1B

// Allow comand inputs from the serial monitor
String inputs[4]; // ["KP", "KI", "KD", "SV"]

// Create the PID controller
float KP   = 200.0; // kp
float KD   = 0.1;   // kd
float KI   = 0.1;   // ki
float SV   = 0.0;   // set value
float TAU  = 0.1;   // derivative filter time constant
YAPID pid(KP, KI, KD, TAU);


void setup() {
  ITimer2.init();
  ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS, Timer2Handler);
  
  pinMode(pwm1_port, OUTPUT);  
  pinMode(pwm2_port, OUTPUT);  
  analogWrite(pwm1_port, 0);     
  analogWrite(pwm2_port, 0);   

  pid.SetOutputLimits(0., 255.)  ;

  Serial.begin(9600);
}


void rx()
{
  while(Serial.available()){
    int StringCount = 0;
    String input = Serial.readString();

    // Split the string into substrings
    while (input.length() > 0){
      int index = input.indexOf(' ');
      if (index == -1) { // No space found
        inputs[StringCount++] = input;
        break;
      }
      else {
        inputs[StringCount++] = input.substring(0, index);
        input = input.substring(index+1);
      }
    }

    KP = inputs[0].toFloat();
    KI = inputs[1].toFloat();
    KD = inputs[2].toFloat();
    SV = inputs[3].toFloat();
  }
}


inline void tx()
{
  Serial.print((float)pid.Now()/1000000., 4);
  Serial.print(",");
  Serial.print(pid.SV());
  Serial.print(",");
  Serial.print(pid.CO());
  Serial.print(",");
  Serial.print(pid.PV());
  Serial.print("\n"); 
}


void Timer2Handler()
{ 
  float pv = (float)analogRead(A0) * 500.0 / 1024.0; // LM35 in A0 (Celcius)
  
  float co = pid.Compute3(SV, pv);
  
  analogWrite(pwm1_port, (int)co);
  
  pid.UpdateTime();
}


void loop()
{
  // Serial receive
  rx();
  
  // Serial transmit
  tx();
}

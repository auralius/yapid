/**
 * N20 DC-motor with rotary encoder. 
 * The quadrature encoder generates 7 pulses per rotation.
 * The encoder readings will be set to x1 mode, thus, only channel A 
 * will trigger the external interrupt. Motor gear ratio is 210:1.
 *
 * https://auralius.github.io/yapid/#dc-motor-control-with-quadrature-encoder
 */

#include "yapid.h"

#define PID_LOOP_INTERVAL_US    1000

// Pin/port configurations
int encoderA_pin   = 2;  // Digital pin #2, INT0
int encoderB_pin   = 3;  // Digital pin #4
const int pwm_port = 11; // PWM of motor, Timer 2
const int dir_port = 13; // Direction of the motor.

// Create the PID controller
float kp = 10.;   // kp
float ki = 4.0;   // ki
float kd = 0.01;  // kd
float N  = 100.0; // derivative filter constant (Hz)
float Ts = 1e-3;
YAPID pid(Ts, kp, ki, kd, N);

volatile float SV      = 0.0;     // set value
volatile long pulses   = 0;       // Output pulses.
const float ppr        = 7*210-1; // Pulses per rotation 

static unsigned long lastLoop = 0;


//------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);

  pinMode(pwm_port, OUTPUT);
  pinMode(dir_port, OUTPUT);
  pinMode(encoderA_pin, INPUT);
  pinMode(encoderB_pin, INPUT);

  analogWrite(pwm_port, 0);     
  digitalWrite(dir_port, HIGH);

  attachInterrupt(0, A_CHANGE, RISING); 

  pid.SetOutputLimits(-254., 254.);
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
  Serial.print(pid.SV());
  Serial.print(",");
  Serial.print(pid.PV());
  Serial.print("\n");
}

void Timer1Handler()
{ 
  float pv =  (float)pulses / ppr * 360.; // in degs

  float co = pid.Compute1(SV, pv);
  analogWrite(pwm_port, (int)abs(co));

  if (co > 0.0)
    digitalWrite(dir_port, HIGH);
  else
    digitalWrite(dir_port, LOW);

  pid.UpdateTime();
}

inline void runPID()
{
  float pv =  (float)pulses / ppr * 360.; // in degs

  float co = pid.Compute1(SV, pv);
  analogWrite(pwm_port, (int)abs(co));

  if (co > 0.0)
    digitalWrite(dir_port, HIGH);
  else
    digitalWrite(dir_port, LOW);

  pid.UpdateTime();
}

void loop()
{    
  runPID();
  
  // wait and do something else!
  while(micros()-lastLoop < PID_LOOP_INTERVAL_US) {
    rx();
    tx();

    // Experiment routines
    float now = pid.Now();
    if (now < 5.0)
      SV = 0.0;
    else if ((now > 5.0) && (now < 8.0)) 
      SV = 180.0;
    else if (now > 8.0 && (now < 11.0)) 
      SV = -180.0;
    else if (now > 11.0) 
      SV = 0.0;
  }
  lastLoop = micros();
}

void A_CHANGE()
{
  if( digitalRead(encoderA_pin) == digitalRead(encoderB_pin) )
    pulses--; // moving reverse
  else 
    pulses++; // moving forward
}  

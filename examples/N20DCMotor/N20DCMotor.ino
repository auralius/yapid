/*
 * Position control of a DC motor, equipped with a quadrature encoder.
 * Encoder-A pin goes to pin #3.
 * Encoder-B pin goes to pin #2.
 * The motor driver is the Arduino motor shield rev3.
 * PWM for the motor comes from port #11 and direction comes from port #13
 *
 * PID control is executed periodically in Timer-1 interrupt function.
 */

#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USE_TIMER_1                   true

#include "TimerInterrupt.h"  // https://github.com/khoih-prog/TimerInterrupt
#include "yapid.h"           // Our YAPID

 
#define TIMER1_INTERVAL_MS 1


int encoderA_pin         = 3;      // Digital pin #3
int encoderB_pin         = 2;      // Digital pin #2
const int pwm_port       = 11;     // PWM of the DC motor, Timer 2
const int dir_port       = 13;     // Direction of the motor. 


volatile int pulses      = 0;      // Output pulses.
const int ppr            = 2940;   // Pulses per rotation 

String inputs[4]; // ["KP", "KI", "KD", "SV"]

// Create the PID controller
float KP   = 200.0; // kp
float KD   = 0.1;   // kd
float KI   = 0.1;   // ki
float SV   = 0.0;   // set value
float TAU  = 0.1;   // derivative filter time constant

YAPID pid(KP, KI, KD, TAU);


void setup() {
  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, Timer1Handler);
   
  // https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
  TCCR2B = TCCR2B & B11111000 | B00000001; // Timer2, PWM frequency: 31372.55 Hz

  pinMode(pwm_port, OUTPUT);
  pinMode(dir_port, OUTPUT);
  
  analogWrite(pwm_port, 0);     
  digitalWrite(dir_port, HIGH);
  
  pinMode(encoderA_pin, INPUT);
  pinMode(encoderB_pin, INPUT);

  attachInterrupt(0, A_CHANGE, CHANGE);
  
  pid.SetOutputLimits(-255., 255);
  
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
  Serial.print(pid.Now()*1e-6);
  Serial.print(",");
  Serial.print(pid.SV());
  Serial.print(",");
  Serial.print(pid.CO());
  Serial.print(",");
  Serial.print(pid.PV());
  Serial.print("\n");  
}

void Timer1Handler()
{ 
  float pv = (float)pulses / (float)ppr * 360.0; // update your process value here

  float co_ = pid.Compute2(SV, pv);
  
  // Handle direction
  if (co_ < 0.0)
    digitalWrite(dir_port, HIGH);
  else
    digitalWrite(dir_port, LOW);

  // Send control output to PWM
  analogWrite(pwm_port, (int)abs(co_));

  pid.UpdateTime();
}


/*
 * Arduino's forever loop
 */
void loop()
{
  // Serial receive
  rx();
  
  // Serial transmit
  tx();
}


/*
 * The following two functions handle the quadrature encoder.
 */
void A_CHANGE() 
{
  if ( digitalRead(encoderB_pin) == 0 ) {
    if ( digitalRead(encoderA_pin) == 0 ) {
      // A fell, B is low
      pulses--; // Moving forward
    } 
    else {
      // A rose, B is high
      pulses++; // Moving reverse
    }
  } 
  else {
    if ( digitalRead(encoderA_pin) == 0 ) {
      pulses++; // Moving reverse
    } 
    else {
      // A rose, B is low
      pulses--; // Moving forward
    }
  }
}
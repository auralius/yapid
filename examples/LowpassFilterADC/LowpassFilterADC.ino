/*
 * A demo for first-order-lowpass filter
 * Filter time constant: 0.1 s
 * Device: Arduino UNO R4 WiFi
 *
 * An LM35-temperature sensor is connected to the A0. 
 * ADC resolution is set to 14 bits.
 * Readings are already in Celcius.
 * 
 * We read the sensor periodically (1000Hz, using the hardware timer).
 */
 
#include "FspTimer.h"
#include "yapid.h"

FspTimer pid_timer;

// Filter sampling rate 
#define SAMPLING_RATE_HZ 1000

// Create the object
YAPID pid;

float IN  = 0.0;
float OUT = 0.0;


void timer_callback(timer_callback_args_t __attribute((unused)) *p_args)
{
  IN = (float)analogRead(A0) * 500.0 / 16383.0;  // LM35 at A0
  OUT = pid.FirstOrderFilter(0.1, IN);
  pid.UpdateTime();
}

// https://www.pschatzmann.ch/home/2023/07/01/under-the-hood-arduino-uno-r4-timers/
bool beginTimer(float rate) 
{
  uint8_t timer_type = GPT_TIMER;
  int8_t tindex = FspTimer::get_available_timer(timer_type);
  if (tindex < 0) {
    tindex = FspTimer::get_available_timer(timer_type, true);
  }
  if (tindex < 0) {
    return false;
  }

  FspTimer::force_use_of_pwm_reserved_timer();

  if (!pid_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, timer_callback)) {
    return false;
  }

  if (!pid_timer.setup_overflow_irq()) {
    return false;
  }

  if (!pid_timer.open()) {
    return false;
  }

  if (!pid_timer.start()) {
    return false;
  }
  return true;
}


void setup() {
  analogReadResolution(14);
  beginTimer(SAMPLING_RATE_HZ);

  delay(1000);
  Serial.begin(115200);
}


inline void tx()
{
  Serial.print((float)pid.Now()/1000000., 4);
  Serial.print(",");
  Serial.print(IN);
  Serial.print(",");
  Serial.print(OUT);
  Serial.print("\n"); 
}

void loop()
{
  // Serial transmit
  tx();
}
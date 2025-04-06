/** 
 * \file yapid.cpp
 *
 * Yet Another PID (YAPID) library for Arduino
 * Auralius Manurung, Universitas Telkom
 * <auralius.manurung@ieee.org>
 */
 
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include <yapid.h>

YAPID::YAPID(float ts)
{
  Ts = ts;
   
  tbase    = micros();
  tnow     = 0;
  telapsed = 0.0;
}

YAPID::YAPID(float ts, float kp, float ki, float kd, float n)
{
  Kp = kp;
  Ki = ki;
  Kd = kd;
  N  = n;
  
  Ts = ts;
   
  tbase    = micros();
  tnow     = 0;
  telapsed = 0.0;
}

void YAPID::UpdateTime()
{
  tprev    = tnow;
  tnow     = micros() - tbase;
  telapsed = (float)(tnow - tprev) * 1e-6; // in secs
}

void YAPID::SetGains(float kp, float ki, float kd)
{
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

void YAPID::SetSamplingTime(float ts)
{
  Ts = ts;  // must be in seconds
}

void YAPID::SetOutputLimits(float min_val, float max_val)
{
  max_co = max_val;
  min_co = min_val;
}

float YAPID::Now()
{
  return (float)tnow / 1000000.0; // in seconds
}

float YAPID::PV()
{
  return pv0;
}

float YAPID::SV()
{
  return sv;
}

float YAPID::CO()
{
  return co;
}

float YAPID::SAT_CO()
{
  return sat_co;
}

float YAPID::TS()
{
  return Ts;
}

float YAPID::TE()
{
  return telapsed;
}

float YAPID::P()
{
  return P0;
}

float YAPID::I()
{
  return I0;
}

float YAPID::D()
{
  return D0;
}

void YAPID::Reverse()
{
  sign = -sign;
}

float YAPID::Compute0(float set_value, float process_value)
{  
  sv = set_value;
  pv0 = process_value;
  
  e0 = sv - pv0;
  
  co = sv;
  
  if (co > max_co)
    sat_co = max_co;
  else if (co < min_co) 
    sat_co = min_co;
  else
    sat_co = co;
    
  // Shift the variables
  e1  = e0;
  I1  = I0;
  D1  = D0;
  pv1 = pv0;
  
  return sign * sat_co;
}

float YAPID::Compute1(float set_value, float process_value)
{  
  sv = set_value;
  pv0 = process_value;
  
  e0 = sv-pv0;
  
  float I0_ = Ki * (e0 + e1) * Ts/2;
  P0 = Kp * e0;
  I0 =  I0_ + I1;
  D0 = ( 2*Kd*N * (e0-e1) / Ts - D1 * (N-2/Ts) ) / (N + 2/Ts);
  
  co = P0 + I0 + D0;
  
  if (co > max_co){
    sat_co = max_co;
    I0 = I0 - I0_; // Stop the integral
  }
  else if (co < min_co) {
    sat_co = min_co;
    I0 = I0 - I0_; // Stop the integral
  }
  else
    sat_co = co;
    
  // Shift the variables
  e1  = e0;
  I1  = I0;
  D1  = D0;
  pv1 = pv0;
    
  return sign * sat_co;
}

float YAPID::Compute2(float set_value, float process_value)
{  
  sv = set_value;
  pv0 = process_value;
  
  e0 = sv-pv0;
  
  float I0_ = Ki * (e0 + e1) * Ts/2;
  P0 = Kp * e0;
  I0 = I0_ + I1;
  D0 = ( 2*Kd*N * (-pv0+pv1) / Ts - D1 * (N-2/Ts) ) / (N + 2/Ts);
  
  co = P0 + I0 + D0;
  
  if (co > max_co){
    sat_co = max_co;
    I0 = I0 - I0_; // Stop the integral
  }
  else if (co < min_co) {
    sat_co = min_co;
    I0 = I0 - I0_; // Stop the integral
  }
  else
    sat_co = co;
    
  // Shift the variables
  e1  = e0;
  I1  = I0;
  D1  = D0;
  pv1 = pv0;
  
  return sign * sat_co;
}


float YAPID::FOLP(float tau, float in)
{
  x1 = x0;
  x0 = in;
  y1 = y0;
  y0 = (x0 + x1 - y1*(1-2*tau/Ts)) / (1+2*tau/Ts);
  return y0;
}
  
float YAPID::SOLP(float wc, float in)
{
  x2 = x1;
  x1 = x0;
  x0 = in;
  y2 = y1;
  y1 = y0;
  
  float sq22  = 2*sqrt(2);
  float Tswc  = Ts * wc;
  float Tswc2 = Tswc * Tswc;
  float A = 1 + sq22/Tswc + 4/Tswc2;
  float B = 2 - 8/Tswc2;  
  float C = 1 - sq22/Tswc + 4/Tswc2;

  y0 = (x0 + 2*x1 + x2 - y1*B - y2*C ) / A;
  return y0;
 }

float YAPID::X()
{
  return x0;
}

float YAPID::Y()
{
  return y0;
}

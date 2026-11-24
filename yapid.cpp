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
#include <string.h>

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
  
// Shared Butterworth section lookup is defined below, before SOHP().
static bool LookupC(const char* id, float &c);

float YAPID::SOLP(float wc, float in, const char* c)
{
  if (Ts <= 0.0f) return y0;
  if (wc <= 0.0f) return in;

  float cval;
  if (!LookupC(c, cval)) return y0;  // Invalid section: state untouched.

  // Tustin: s = (2/Ts)*(1 - z^-1)/(1 + z^-1).
  // H(s) = wc^2 / (s^2 + c*wc*s + wc^2).
  // With q = 2/(Ts*wc):
  //    A  = 1 + c*q + q^2
  //    b0 = 1/A, b1 = 2/A, b2 = 1/A
  //    a1 = 2*(1-q^2)/A, a2 = (1-c*q+q^2)/A.
  // y[k] = b0*x[k] + b1*x[k-1] + b2*x[k-2]
  //      - a1*y[k-1] - a2*y[k-2].
  float q  = 2.0f / (Ts * wc);
  float q2 = q * q;
  float A  = 1.0f + cval*q + q2;

  float b0 = 1.0f / A;
  float b1 = 2.0f / A;
  float b2 = b0;
  float a1 = 2.0f * (1.0f - q2) / A;
  float a2 = (1.0f - cval*q + q2) / A;

  x2 = x1;
  x1 = x0;
  x0 = in;
  y2 = y1;
  y1 = y0;

  y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2;
  return y0;
}

float YAPID::FOHP(float tau, float in)
{
  if (Ts <= 0.0f) return y0;
  if (tau <= 0.0f) return 0.0f;

  x1 = x0;
  x0 = in;
  y1 = y0;

  // First-order high-pass filter:
  //
  //              tau*s
  // H(s) = ----------------
  //          tau*s + 1
  //
  // Using Tustin substitution:
  //
  //          2   1 - z^-1
  // s  =  ----  --------
  //          Ts  1 + z^-1
  //
  // The discrete-time equation is:
  //
  // y[k] = b0*x[k] + b1*x[k-1] - a1*y[k-1]
  //
  // where:
  //
  // alpha = 2*tau/Ts
  // b0    =  alpha/(1 + alpha)
  // b1    = -alpha/(1 + alpha)
  // a1    = (1 - alpha)/(1 + alpha)

  float alpha = 2.0f * tau / Ts;

  float b0 =  alpha / (1.0f + alpha);
  float b1 = -alpha / (1.0f + alpha);
  float a1 = (1.0f - alpha) / (1.0f + alpha);

  y0 = b0*x0 + b1*x1 - a1*y1;

  return y0;
}


// Butterworth second-order sections: s^2 + c*wc*s + wc^2
// c = 2*sin((2k-1)*pi/(2N)), k = 1..floor(N/2)
// S1 is the first-order section (use FOHP with tau = 1/wc), so it is not listed.
struct SectionC {
  const char* id;
  float c;
};

static const SectionC SECTION_TABLE[] = {
  {"S2", 1.41421356f},  // N = 2
  {"S3", 1.00000000f},  // N = 3
  {"S4", 0.76536686f},  // N = 4, k = 1
  {"S5", 1.84775907f},  // N = 4, k = 2
  {"S6", 0.61803399f},  // N = 5, k = 1
  {"S7", 1.61803399f}   // N = 5, k = 2
};

// Look up c from a section ID. Returns false if the ID is unknown.
static bool LookupC(const char* id, float &c)
{
  if (id == 0) return false;

  for (unsigned int i = 0; i < sizeof(SECTION_TABLE) / sizeof(SECTION_TABLE[0]); i++) {
    if (strcmp(id, SECTION_TABLE[i].id) == 0) {
      c = SECTION_TABLE[i].c;
      return true;
    }
  }
  return false;
}

float YAPID::SOHP(float wc, float in, const char* c)
{
  if (Ts <= 0.0f) return y0;
  if (wc <= 0.0f) return in;

  float cval;
  if (!LookupC(c, cval)) return y0;  // unknown section ID: keep the state untouched

  x2 = x1;
  x1 = x0;
  x0 = in;
  y2 = y1;
  y1 = y0;

  // Second-order Butterworth high-pass filter section:
  //
  //                  s^2
  // H(s) = -------------------------
  //        s^2 + c*wc*s + wc^2
  //
  // wc must be supplied in rad/s. c comes from the section ID
  // (c = sqrt(2) for the plain second-order Butterworth filter, "S2").
  //
  // Using Tustin substitution:
  //
  //          2   1 - z^-1
  // s  =  ----  --------
  //          Ts  1 + z^-1
  //
  // Define:
  //
  // q = 2/(Ts*wc)
  //
  // The discrete-time equation is:
  //
  // y[k] = b0*x[k] + b1*x[k-1] + b2*x[k-2]
  //        - a1*y[k-1] - a2*y[k-2]
  //
  // where:
  //
  // A  = 1 + c*q + q^2
  // b0 = q^2/A
  // b1 = -2*q^2/A
  // b2 = q^2/A
  // a1 = 2*(1 - q^2)/A
  // a2 = (1 - c*q + q^2)/A

  float q  = 2.0f / (Ts * wc);
  float q2 = q * q;

  float A = 1.0f + cval*q + q2;

  float b0 =  q2 / A;
  float b1 = -2.0f * q2 / A;
  float b2 =  q2 / A;
  float a1 =  2.0f * (1.0f - q2) / A;
  float a2 = (1.0f - cval*q + q2) / A;

  y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2;

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

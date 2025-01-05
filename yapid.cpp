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


YAPID::YAPID(float kp, float ki, float kd, float tau)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;

  tau_ = tau;
  
  past_err_ = 0.0;
  err_ = 0.0;
  
  P_ = 0.0;
  I_ = 0.0;
  D_ = 0.0;
  Df_ = 0.0;
  
  tbase_ = micros();
  tnow_  = 0.0;
}

void YAPID::UpdateTime()
{
  tprev_ = tnow_;
  tnow_  = micros() - tbase_;
  ts_    = (float)(tnow_ - tprev_) * 1e-6; // in secs
}

void YAPID::SetGains(float kp, float ki, float kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void YAPID::SetSamplingTime(float ms)
{
  ts_ = ms;
}

void YAPID::SetOutputLimits(float min, float max)
{
  max_ = max;
  min_ = min;
}

float YAPID::Kp()
{
  return kp_;
}
  
float YAPID::Ki()
{
  return ki_;
}

float YAPID::Kd()
{
  return kd_;
}   

float YAPID::Ts()
{
  return ts_;
}

unsigned long YAPID::Now()
{
  return tnow_;
}

float YAPID::PV()
{
  return pv_;
}

float YAPID::SV()
{
  return sv_;
}

float YAPID::CO()
{
  return co_;
}

void YAPID::SetInvertedControlOuput(bool inverted)
{
  if (inverted == true)
    sign_ = -1.0;
  else
    sign_ = 1.0;
}

float YAPID::Compute1(float sv, float pv)
{
  UpdateTime();
  
  sv_      = sv;
  
  past_pv_ = pv_;
  pv_      = pv;
  
  past_err_ = err_;
  err_      = sv_ - pv_;
  
  P_        = kp_ * err_;
  D_        = kd_ * (err_ - past_err_) / ts_;
  I_        = I_ + ki_ * ts_ * err_;
  
  co_       = P_ + I_ + D_;
  
  return sign_ * co_;
}

float YAPID::Compute2(float sv, float pv)
{
  UpdateTime();
  
  sv_      = sv;
  
  past_pv_ = pv_;
  pv_      = pv;
  
  past_err_ = err_;
  err_      = sv_ - pv_;
  
  P_        = kp_ * err_;
  D_        = kd_ * (err_ - past_err_) / ts_;
  I_        = I_ + ki_ * ts_ * err_;
  Df_       = (tau_ * Df_ + ts_ * D_) / (tau_ + ts_) ;
  
  co_       = P_ + I_ + Df_;
  
  return sign_ * co_;
}

float YAPID::Compute3(float sv, float pv)
{
  UpdateTime();
  
  sv_      = sv;
  
  past_pv_ = pv_;
  pv_      = pv;
  
  past_err_ = err_;
  err_      = sv_ - pv_;

  P_        = kp_ * err_;
  D_        = kd_ * (err_ - past_err_) / ts_;
  I_        = I_ + ki_ * ts_ * err_;
  Df_       = (tau_ * Df_ + ts_ * D_) / (tau_ + ts_) ;
  
  co_       = P_ + I_ + Df_;
  
  if (co_ > max_)
    co_ = max_;
  else if (co_ < min_)
    co_ = min_;
  
  return sign_ * co_;
}

float YAPID::FirstOrderFilter(float fofilt_tau, float fofilt_in)
{
  float fofilt_out = (fofilt_tau * past_fofilt_out_ + ts_ * fofilt_in) / 
                     (fofilt_tau + ts_) ;
  past_fofilt_out_ = fofilt_out;
  
  return fofilt_out;
}

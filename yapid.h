/** 
 * \file yapid.h
 *
 * Yet Another PID (YAPID) library for Arduino
 * Auralius Manurung, Universitas Telkom
 * <auralius.manurung@ieee.org>
 */
 
#ifndef YAPID_h
#define YAPID_h
#define LIBRARY_VERSION	0.0.1

class YAPID
{
public:
  YAPID();
  YAPID(float kp, float ki, float kd, float tau);
  
  void SetGains(float kp, float ki, float kd);
  void SetSamplingTime(float ms);
  void SetOutputLimits(float min, float max);
  void SetInvertedControlOuput(bool inverted);
  
  unsigned long Now();
  
  float Kp();
  float Ki();
  float Kd();
  float Ts();

  // Type-1 PID control
  // D-term is computed from the errors (no filter)
  // Noisy D-term
  float Compute1(float sv, float pv);
  
  // Type-2 PID control
  // Less noisy D-term, D-term is computed from the filtered errors
  // Derivative kicks can happen
  float Compute2(float sv, float pv);
  
  // Type-3 PID control
  // Less noisy D-term, D-term is computed from the filtered process values
  // No more derivative kicks
  float Compute3(float sv, float pv);
  
  // First-order filter
  float FirstOrderFilter(float fofilt_tau, float fofilt_in);
  
  float PV();
  float SV();
  float CO();
  
  // Time update
  void UpdateTime();
  
private:
  float kp_;
  float ki_;
  float kd_;
  float tau_;
  
  float pv_;
  float sv_;
  float co_;
  float past_pv_;
  
  float sign_ = 1.0;
  
  float max_ = 255.0;
  float min_ = 0.0;
  
  float past_err_;  // previous error
  float err_;       // current error
  float P_;         // proportional term
  float I_;         // integral term
  float D_;         // derivative term
  float Df_;        // filtered derivative term
  
  unsigned long tbase_;       
  unsigned long tnow_;       // current time stamp in micro-secs
  unsigned long tprev_;      // previous time stamp in micro-secs
  float ts_;
  
  float past_fofilt_out_;
};

#endif

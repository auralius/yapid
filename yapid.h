/** 
 * \file yapid.h
 *
 * Yet Another PID (YAPID) library for Arduino
 * Auralius Manurung, Universitas Telkom
 * <auralius.manurung@ieee.org>
 */
 
#ifndef YAPID_h
#define YAPID_h
#define LIBRARY_VERSION 0.0.2

class YAPID
{
public:
  /** 
   * The constructor.
   * @param ts Sampling period (in seconds)
   */
  YAPID(float ts);
  
  /** 
   * The constructor.
   * @param ts Sampling period (in seconds)
   * @param kp P-gain
   * @param ki I-gain
   * @param kd D-gain
   * @param n  Derivative filter coefficient 
   */
  YAPID(float ts, float kp, float ki, float kd, float n);
  
  /**
   * Set the PID gains.
   * @param kp P-gain
   * @param ki I-gain
   * @param kd D-gain
   */
  void SetGains(float kp, float ki, float kd);
  
  /**
   * Set the sampling period.
   * @param ts Sampling period in seconds
   */
  void SetSamplingTime(float ts);
  
  /**
   * Set the saturation values of the control output.
   * @param min_val Lower-limit value 
   * @param max_val Upper-limit value
   */
  void SetOutputLimits(float min_val, float max_val);
  
  //! Multiply the output with -1.
  void Reverse();
  
  //! Get current process value.
  float PV();
  
  //! Get current set value
  float SV();
  
  //! Get current output value.
  float CO();

  //! Get current saturated output value.
  float SAT_CO();
    
  //! Get defined sampling rate (in seconds).
  float TS();
  
  //! Get current actual elapsed time (in seconds).
  float TE();
  
  //! Get current P control output.
  float P();
  
  //! Get current I control output.
  float I();
  
  //! Get current D control output.
  float D();
   
  //! Get current time (in micro-seconds).
  float Now();
  
  //! Update the time once.
  void UpdateTime();

  //! Run the PID once.
  /*! Simply send the input as the control ouput (no feedback system!)*/
  float Compute0(float set_value, float process_value);
    
  //! Run the PID once.
  /*! D-term uses the error signal */
  float Compute1(float set_value, float process_value);
  
  //! Run the PID once.
  /*! D-term uses the prcess-value signal */
  float Compute2(float set_value, float process_value);
  
  /**
   * First-order low-pass filter.
   * @param tau Filter time constant
   * @param in Input signal to filter
   */
  float FOLP(float tau, float in);
  
  /**
   * Second-order low-pass filter.
   * @param wc Filter cut-off frequency
   * @param in Input signal to filter
   */  
   float SOLP(float wc, float in);
  
  //! Return filter current input
  float X();
  
  //! Returns filter current output
  float Y();
  
  
private:
  float P0 = 0.0; //!< P(k) 
  float I0 = 0.0; //!< I(k) 
  float I1 = 0.0; //!< I(k-1)
  float D0 = 0.0; //!< D(k) 
  float D1 = 0.0; //!< D(k-1)
  
  float Kp = 0.0; //!< P-gain
  float Ki = 0.0; //!< I-gain
  float Kd = 0.0; //!< D-gain
  float N = 0.0;  //!< Derivative filter coefficient

  float pv0 = 0.0;     //!< process value at k
  float pv1 = 0.0;     //!< process value at k-1
  
  float sv = 0.0;     //!< set value
  float co = 0.0;     //!< control output
  float sat_co = 0.0; //!< control output after the saturator
  
  float sign = 1.0;    //!< for inverting the output if needed
  
  float max_co = 255.0; //!< control output upper limit
  float min_co = 0.0;   //!< control output lower limit
  
  float e0 = 0.0;       //!< e(k)
  float e1 = 0.0;       //!< e(k-1)

  float x0 = 0.0;       //!< x(k)
  float x1 = 0.0;       //!< x(k-1)
  float x2 = 0.0;       //!< x(k-2)
  float y0 = 0.0;       //!< y(k)
  float y1 = 0.0;       //!< y(k-1)
  float y2 = 0.0;       //!< y(k-2)
  
  unsigned long tbase = 0;       
  unsigned long tnow  = 0;   //!< current time stamp in micro-seconds
  unsigned long tprev = 0;  //!< previous time stamp in micro-seconds
  
  float telapsed = 0.0; //!< elapsed time in micro-seconds
  float Ts       = 1.0; //!< sampling period in seconds
};

#endif

# Yet Another PID (YAPID) library for Arduino   

Contact:
Auralius Manurung, Universitas Telkom, <auralius.manurung@ieee.org>   

Detailed documentations:
https://auralius.github.io/yapid/

---

In YAPID, the discrete implementations of both the control and the filter use bilinear transformation (Tustin or trapezoidal) method. **Check [this page](https://auralius.github.io/control-systems-with-sympy/digital-pid-2.html) for details on the control derivation.**


## Implemented functions: 

* `float YAPID::Compute0(float set_value, float process_value)`
  * This simply sends `set_value` as control output.
  * There is no feedback control happening here.
  * This function is useful to testing system's open-loop respose.
    
* `float YAPID::Compute1(float set_value, float process_value)`
  * D-term is computed from the filtered errors.  
  * Derivative kicks will happen
  * Simple integral windup prevention with clamping technique 
    
* `float YAPID::Compute2(float set_value, float process_value)`
  * D-term is computed from the filtered process values    
  * No more derivative kicks
  * Simple integral windup prevention with clamping technique 
    
* `float YAPID::FOLP(float tau, float in)`
  * First-order low-pass filter (time-constant filter)
 
* `float YAPID::SOLP(float tau, float in)`
  * Second-order low-pass filter (Butterworth filter)

## How to use the YAPID library

__Abbreviations__:

* ```sv```: set value
* ```pv```: process value
* ```co```: control output
* ```P```: P-term control output
* ```I```: I-term control output
* ```D```: D-term control output
* ```Kp```: proportinal gain
* ```Ki```: integral gian
* ```Kd```: derivative gain
* ```N```: derivative filter coefficient


__How to use__:

YAPID **DOES NOT** measure the elapsed time every iteration. It simply uses the provided sampling time during the setup. Therefore, to guarantee control determinism, it is easier if we use a timer interrupt handler to run the control periodically. All provided examples use timer interrupts.   

To use YAPID library, first, we need to include the header file:

```cpp
#include "yapid.h"
```

Next, we create a global YAPID object and several global variables:

```cpp
// Create the PID controller
float kp = 100.;  // kp
float ki = 10.0;  // ki
float kd = 1.0;   // kd
float N  = 1.0;   // derivative filter coefficient (Hz)
float Ts = 1e-3;
YAPID pid(Ts, kp, ki, kd, N);
```

Within the Arduino's ```setup()``` function, we define the limits for the control's output. The default limit is $0.0 \leq \text{CO} < 255.0$. Since we use a timer interrupt, we setup out timer interrupt also in this ```setup()``` function.
```cpp
#define TIMER2_INTERVAL_MS 1  // 1kHz

void setup()
{
  // Setup the timer interrupt (we use NANO, timer-1, and
  // https://github.com/khoih-prog/TimerInterrupt)
  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, Timer1Handler);

  // Define the control output limits
  // Here, the output becomes PWM signals (0 to 255)
  pid.SetOutputLimits(0., 255)

  // ...
  // ...
}
```

Finally, in the timer interrupt function handler, we can put our PID control. The steps are: 
* read the sensor (```pv```)
* compute the PID (```co```)
* apply the output to the actuator/plant
* measure the elapsed time

```cpp
void Timer1Handler()
{ 
  float pv = (float)analogRead(A0) * 5.0 / 1024.0;

  float co = pid.Compute1(SV, pv);
  
  analogWrite(pwm_port, (int)co);
  
  pid.UpdateTime();
}
```

## Comparison with MATLAB Simulink

![](https://github.com/auralius/yapid/blob/main/compare_matlab.png)


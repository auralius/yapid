# Yet Another PID (YAPID) library for Arduino   
```
* Auralius Manurung, Universitas Telkom  
* <auralius.manurung@ieee.org>   
```
**Check [this note](https://github.com/auralius/arduino-pid-template/blob/main/Notes%20on%20PID%20control%20with%20Arduino.pdf) for details on the control derivation.**


## Implemented functions: 

* _PID control type-1_  
  * Noisy D-term, D-term is computed from the errors (no filter)
    
* _PID control type-2_  
  * Less noisy D-term, D-term is computed from the filtered errors  
  * Derivative kicks can happen  
    
* _PID control type-3_  
  * Also less noisy D-term, D-term is computed from the filtered process values    
  * No more derivative kicks  
    
* _First-order lowpass filter_   
  <img src="https://github.com/auralius/yapid/blob/main/filter-demo.gif" alt="Alt Text" style="width:70%; height:auto;">

## How to use the YAPID library

__Abbreviations__:

* ```SV```: set value
* ```PV```: process value
* ```CO```: control output

__How to use__:

YAPID measures the elapsed time every iteration and uses the value for the PID control. However, <ins>YAPID does not attempt to control the elapsed time</ins>.
To guarantee control determinism, it is easier if we put the control in a timer interrupt. All provided examples use timer interrupts.  

To use YAPID library, first, we need to include the header file:

```cpp
#include "yapid.h"
```

Next, we create a global YAPID object and several global variables:

```cpp
// Create the PID controller
float KP   = 200.0; // kp
float KD   = 0.1;   // kd
float KI   = 0.1;   // ki
float SV   = 0.0;   // set value
float TAU  = 0.1;   // derivative filter time constant
YAPID pid(KP, KI, KD, TAU);
```

Within the Arduino's ```setup()``` function, we define the limits for the control's output. The default limit is $0.0 \leq \text{CO} < 255.0$. Since we use a timer interrupt, we setup out timer interrupt also in this ```setup()``` function.
```cpp
#define TIMER2_INTERVAL_MS 1  // 1kHz

void setup()
{
  // Setup the timer interrupt (we use UNO, timer-2, and
  // https://github.com/khoih-prog/TimerInterrupt)
  ITimer2.init();
  ITimer2.attachInterruptInterval(TIMER2_INTERVAL_MS,
                                  Timer2Handler);

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
* send the output out
* measure the elapsed time

```cpp
void Timer2Handler()
{ 
  float pv = (float)analogRead(A0) * 500.0 / 1024.0; // LM35 in A0 (Celcius)
  
  float co = pid.Compute3(SV, pv);
  
  analogWrite(pwm1_port, (int)co);
  
  pid.UpdateTime();
}
```

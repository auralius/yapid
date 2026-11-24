# Yet Another PID (YAPID) library for Arduino

Contact:  
Auralius Manurung, Universitas Telkom, <auralius.manurung@ieee.org>

Detailed documentation:  
https://auralius.github.io/yapid/

---

YAPID provides discrete PID controllers and first- and second-order digital filters for Arduino. The implementations use the **bilinear transformation (Tustin or trapezoidal method)**. See the [digital PID derivation](https://auralius.github.io/control-systems-with-sympy/digital-pid-2.html) for the controller and the [digital filter derivation](https://auralius.github.io/control-systems-with-sympy/digital-filter.html) for filter design.

The second-order low-pass (`SOLP`) and high-pass (`SOHP`) functions also support **cascaded Butterworth sections**.

## Implemented functions

### PID control

- `float YAPID::Compute0(float set_value, float process_value)`  
  Sends the set value as the control output (subject to the configured output limits). There is no feedback regulation; useful for open-loop tests.

- `float YAPID::Compute1(float set_value, float process_value)`  
  Computes the derivative term from the **error** signal. A setpoint step can produce derivative kick. Includes simple integral windup prevention by clamping.

- `float YAPID::Compute2(float set_value, float process_value)`  
  Computes the derivative term from the **process value**, avoiding derivative kick caused by a setpoint step. Includes simple integral windup prevention by clamping.

### Digital filters

- `float YAPID::FOLP(float tau, float in)`  
  First-order low-pass filter. `tau` is the time constant, in **seconds**.

- `float YAPID::FOHP(float tau, float in)`  
  First-order high-pass filter. `tau` is the time constant, in **seconds**.

- `float YAPID::SOLP(float wc, float in, const char* c = "S2")`  
  Second-order low-pass Butterworth section. `wc` is the cutoff **angular frequency in rad/s**. Optional `c` selects the section; omission uses `"S2"` and preserves old two-argument calls.

- `float YAPID::SOHP(float wc, float in, const char* c = "S2")`  
  Second-order high-pass Butterworth section with the same angular-frequency and section-ID convention.

**Frequency units:** `wc = 2 * PI * fc`, where `fc` is in Hz. For example, a 5 Hz cutoff means `wc = 2 * PI * 5` rad/s. Do not confuse `wc` with the first-order filter's `tau`.

## More on the Butterworth filter

Each second-order section has the denominator

$$
s^2+c\omega_c s+\omega_c^2.
$$

For the corresponding low-pass and high-pass sections:

$$
H_{LP}(s)=\frac{\omega_c^2}{s^2+c\omega_c s+\omega_c^2},\qquad
H_{HP}(s)=\frac{s^2}{s^2+c\omega_c s+\omega_c^2}.
$$

The optional section ID selects the coefficient `c`:

| ID | `c` | Application |
|---|---:|---|
| `"S2"` | 1.41421356 | Complete 2nd-order Butterworth filter (default) |
| `"S3"` | 1.00000000 | 2nd-order section of a 3rd-order filter |
| `"S4"` | 0.76536686 | First 2nd-order section of a 4th-order filter |
| `"S5"` | 1.84775907 | Second 2nd-order section of a 4th-order filter |
| `"S6"` | 0.61803399 | First 2nd-order section of a 5th-order filter |
| `"S7"` | 1.61803399 | Second 2nd-order section of a 5th-order filter |

For odd orders (3 and 5), **also cascade one first-order section** with `tau = 1 / wc` using `FOLP()` or `FOHP()` as appropriate.

**Important:** Each cascaded section **must use a separate `YAPID` object**. The filter functions share each object's input/output history (`x0`, `x1`, `x2`, `y0`, `y1`, `y2`). Do not use one object for multiple simultaneous sections or independent filter channels.

### Second-order low-pass (unchanged usage)

```cpp
const float Ts = 0.01f;               // 100 Hz sample rate
const float wc = 2.0f * PI * 5.0f;    // 5 Hz cutoff
YAPID lp2(Ts);

// Call once per sample:
float filterSample(float input) {
  return lp2.SOLP(wc, input);  // same as SOLP(wc, input, "S2")
}
```

### Fourth-order low-pass (two cascaded sections)

```cpp
const float Ts = 0.01f;
const float wc = 2.0f * PI * 5.0f;
YAPID lp4a(Ts);
YAPID lp4b(Ts);

// Call once per sample:
float filterSample(float input) {
  float y1 = lp4a.SOLP(wc, input, "S4");
  return lp4b.SOLP(wc, y1, "S5");
}
```

### Fifth-order low-pass (one first-order + two second-order sections)

```cpp
const float Ts = 0.01f;
const float wc = 2.0f * PI * 5.0f;
YAPID lp5a(Ts);
YAPID lp5b(Ts);
YAPID lp5c(Ts);

float filterSample(float input) {
  float y1 = lp5a.FOLP(1.0f / wc, input);
  float y2 = lp5b.SOLP(wc, y1, "S6");
  return lp5c.SOLP(wc, y2, "S7");
}
```

### Fourth-order high-pass

```cpp
const float Ts = 0.01f;
const float wc = 2.0f * PI * 5.0f;
YAPID hp4a(Ts);
YAPID hp4b(Ts);

float filterSample(float input) {
  float y1 = hp4a.SOHP(wc, input, "S4");
  return hp4b.SOHP(wc, y1, "S5");
}
```

Third-order high-pass: cascade `FOHP(1.0f/wc, input)` with `SOHP(wc, ..., "S3")` using separate objects. Fifth-order high-pass: cascade `FOHP()` with `SOHP(..., "S6")` and `SOHP(..., "S7")`.

The sections share a **common sampling period and cutoff frequency**. They use direct Tustin discretization without frequency prewarping, so the realized digital cutoff can differ from the requested analog cutoff, especially near the Nyquist limit.

### Digital difference equation

For `SOLP()`, with sampling period `Ts` and section coefficient `c`, define

$$
q=\frac{2}{T_s\omega_c},\qquad A=1+cq+q^2.
$$

Its normalized IIR coefficients are

$$
\begin{aligned}
b_0&=1/A, & b_1&=2/A, & b_2&=1/A,\\
a_1&=2(1-q^2)/A, & a_2&=(1-cq+q^2)/A.
\end{aligned}
$$

The implementation evaluates

$$
y[k]=b_0x[k]+b_1x[k-1]+b_2x[k-2]-a_1y[k-1]-a_2y[k-2].
$$

Cascading sections means feeding the output of one section into the input of the next. The overall transfer function is the **product** of the individual section transfer functions.

## How to use the YAPID library

### Abbreviations

- `sv`: set value (setpoint)
- `pv`: process value (measurement)
- `co`: control output
- `P`, `I`, `D`: proportional, integral, and derivative control terms
- `Kp`, `Ki`, `Kd`: PID gains
- `N`: derivative filter coefficient
- `Ts`: intended sampling period, in seconds
- `tau`: first-order filter time constant, in seconds
- `wc`: cutoff angular frequency for second-order filters, in rad/s

### Setup and periodic control

YAPID **does not automatically measure elapsed time to choose its sampling period**. Its controller and filter calculations use the `Ts` supplied to the constructor (or `SetSamplingTime()`). Call the controller or filter at that interval. `UpdateTime()` tracks the observed elapsed time for diagnostics; it does not change the configured `Ts`.

A periodic timer interrupt is one way to obtain repeatable sampling. The example below uses an **Arduino Nano (ATmega328P)** and the [TimerInterrupt library](https://github.com/khoih-prog/TimerInterrupt). D3 is used for PWM so Timer1 remains available for the interrupt.

```cpp
#define TIMER_INTERRUPT_DEBUG        0
#define _TIMERINTERRUPT_LOGLEVEL_    0
#define USE_TIMER_1                  true

#include "TimerInterrupt.h"
#include "yapid.h"

#define TIMER1_INTERVAL_MS 1

const int pwm_port = 3;  // PWM on Timer2; Timer1 used for sampling
const float Ts = 1e-3f;
const float SV = 2.5f;   // Example voltage setpoint

YAPID pid(Ts, 100.0f, 10.0f, 1.0f, 1.0f);

void Timer1Handler() {
  float pv = (float)analogRead(A0) * 5.0f / 1024.0f;
  float co = pid.Compute1(SV, pv);
  analogWrite(pwm_port, (int)co);
  pid.UpdateTime();  // Optional timing diagnostic
}

void setup() {
  pinMode(pwm_port, OUTPUT);
  analogWrite(pwm_port, 0);

  pid.SetOutputLimits(0.0f, 255.0f);

  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, Timer1Handler);
}

void loop() {
  // Other non-time-critical tasks, if needed.
}
```

`Compute1()` differentiates the error; change it to `Compute2()` to differentiate the process value instead. Keep real-time interrupt handlers short and verify the actual sample timing on your hardware.

### Using the cascaded filter inside a periodic sampler

Filters also need **one update per sample**. Instantiate their state objects globally, not inside the interrupt:

```cpp
const float Ts = 0.001f;
const float wc = 2.0f * PI * 5.0f;
YAPID stage1(Ts);
YAPID stage2(Ts);

void Timer1Handler() {
  float sample = (float)analogRead(A0);
  float lowpass1 = stage1.SOLP(wc, sample, "S4");
  float lowpass2 = stage2.SOLP(wc, lowpass1, "S5");
  // Use or store lowpass2 here.
}
```

This fragment illustrates the filter calls; combine it with the timer setup from the previous example rather than defining two handlers in one sketch.



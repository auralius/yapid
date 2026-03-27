This example demonstrates a **state-space implementation of a PID controller with derivative filtering**, validated across:

* Python simulation
* Arduino (Model-in-the-loop / Processor-in-the-loop)
* Live serial plotting and logging

---

__1. `PID-DERIVATIVE-FILTER-STATE-SPACE.pdf`__

This document explains the ****mathematical foundation behind the implementation**. The contents 
include derivation the state-space form of a continuous-time PID with derivative filter

---

__2. `pid-ss-model-in-the-loop.ino`__

This is the Arduino implementation of the discrete PID on a discrete 2nd-order plant (all in 
state-space format).

The Arduino program logs:

  ```
  t,r,y,e,u,xc1,xc2,xp1,xp2
  ```

---

__3. `serial-plot.py`__

This is a Python script for **real-time visualization and logging**.


---

__4. `simulation.py`__

This is the pure Python simulation of the same system, which is used to verify Arduino 
implementation.

---

__5. `yapid_utils.py`__

Utility functions for generating models.


# 📂 File Overview

This example demonstrates a **state-space implementation of a PID controller with derivative filtering**, validated across:

* Python simulation
* Arduino (Model-in-the-loop / Processor-in-the-loop)
* Live serial plotting and logging

---

## 1. `PID-DERIVATIVE-FILTER-STATE-SPACE.pdf`

This document explains the ****mathematical foundation behind the implementation**.

Contents include:

* Derivation of PID with derivative filter
* Conversion from transfer function to state-space
* Interpretation of:

  * integrator state
  * derivative filter state
* Final continuous-time state-space form

---

## 2. `pid-ss-model-in-the-loop.ino`

Arduino implementation of the system.

What it does:

* Implements **two `YAPID_SS` blocks**:

  * PID controller (state-space)
  * 2nd-order plant (state-space)
* Runs a **closed-loop simulation on MCU**
* Streams data over serial in CSV format

Other features:

* Command interface:

  * `START` → begin simulation
  * `STOP` → stop simulation
  * `RESET` → reset states
  * `PING` → health check
* Fixed sampling time using `micros()`
* Logs:

  ```
  t,r,y,e,u,xc1,xc2,xp1,xp2
  ```

---

## 3. `serial-plot.py`

Python script for **real-time visualization and logging**.

What it does:

* Connects to Arduino via serial
* Sends `START` command
* Receives streamed data
* Logs to CSV file
* Plots live:

  * reference vs output (`r`, `y`)
  * control signal (`u`)

---

## 4. `simulation.py`

Pure Python simulation of the same system.

What it does:

* Simulates:

  * PID (state-space)
  * Plant (state-space)
* Uses identical update order as Arduino
* Generates:

  * CSV logs
  * plots for comparison

Key purpose:

* Provides a **reference baseline**
* Used to verify Arduino implementation

---

## 5. `yapid_utils.py`

Utility functions for generating models.


```
***************************************************************
* Yet Another PID (YAPID) library for Arduino   
* Auralius Manurung, Universitas Telkom  
* <auralius.manurung@ieee.org>   
***************************************************************
```
---

**Check [this note](https://github.com/auralius/arduino-pid-template/blob/main/Notes%20on%20PID%20control%20with%20Arduino.pdf) for details on the control derivation.**


**Implemented functions:**
  
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



***************************************************************
Yet Another PID (YAPID) library for Arduino   
Auralius Manurung, Universitas Telkom  
<auralius.manurung@ieee.org>   
***************************************************************

Implemented functions:  
  
* PID control type-1  
  * Noisy D-term, D-term is computed from the errors (no filter)
    
* PID control type-2  
  * Less noisy D-term, D-term is computed from the filtered errors  
  * Derivative kicks can happen  
    
* PID control type-3  
  * Also less noisy D-term, D-term is computed from the filtered process values    
  * No more derivative kicks  
    
* First-order lowpass filter   
* <img src="https://github.com/auralius/yapid/blob/main/filter-demo.gif" alt="Alt Text" style="width:70%; height:auto;">


**Check [this note](https://github.com/auralius/arduino-pid-template/blob/main/Notes%20on%20PID%20control%20with%20Arduino.pdf) for details on control derivation.**

# yapid

***************************************************************
* Yet Another PID (YAPID) library for Arduino
* Auralius Manurung, Universitas Telkom
* <auralius.manurung@ieee.org>
***************************************************************

Implemented functions:  
  
* PID control type-1  
    * Noisy D-term, D-term is computed from the errors (no filter)
    
* PID control type-2  
    * Less noisy D-term, D-term is computed from the filterd errors  
    * Derivative kicks can happen  
    
* PID control type-3  
    * Also less noisy D-term, D-term is computed from the errors (no filter)  
    * No more derivative kicks  
    
* First-order-low-pass filter  

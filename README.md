# F28379D_temp_measurement
In this project, I used the ADS124S08EVM, a 4-wire RTD and a LAUNCHXL-F28379D to implement a temperature control system of the air in a box. The list of other
materials used are in the header files. Anyone is free to use it and if you have any question you can contact me using lbertolossi@hotmail.com

I used this project as my bachelor thesis and most importantly to learn how to:
- program a TI microcontroller (C2000 F28379D) - about memory, (external) interrupts, how to use different peripherals, etc.
- program SPI communication
- program a ADC (ADS124S08)
- adapt a firmware to another microcontroller (this project could have been easier if implemented on a Tiva-C)
- program a discrete PID controller from scratch
- tune a PID controller manually
- interface with a sensor
- use HRPWM with a H-bridge to control the temperature with peltier cells. 

>>> Actually HRPWM might not be so impactful/needed since the peltier cell might not be triggered for such small changes on voltage (less than 50 uV).
>>> a more simple microcontroller could have been used to simplify the project.

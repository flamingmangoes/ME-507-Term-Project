The hardware for this project consists of electrical hardware and mechanical hardware. 

The electrical hardware main components are the ESP32 Firebeetle microcontroller and the Texas Instruments DRV8308 BLDC motor driver chip. Much of the software architecture is designed around these two components; the rest of the components are either gate transistors for the motor drive or passive components to support the driver such as resistors and capacitors. The schematic and layout are provided in the "board" folder along with datasheets for the motor driver and motor. 

The datasheet for the ESP32 Firebeetle microcontroller can be found here: https://wiki.dfrobot.com/FireBeetle_Board_ESP32_E_SKU_DFR0654. 


The mechanical hardware consists of the reaction wheel and motor assembly, designed by Caleb Nalley in his previous thesis with Cal Poly SLO, and the hardware for the 1-degree of freedom SACS (Spacecraft Attitude Control Simulator) platform, designed by Bricen Rigby as part of a SURP project with Cal Poly SLO. All CAD files are included in the "mech" folder. 

Caleb Nalley's thesis can be found here: https://digitalcommons.calpoly.edu/theses/3111/

Bricen Rigby's SACS development poster can be found here: https://digitalcommons.calpoly.edu/ceng_surp/50/

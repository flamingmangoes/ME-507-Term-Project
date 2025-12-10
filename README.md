This project is a 1 rotational degree of freedom prototype for an autonomously controlled momentum exchange device. It includes a reaction flywheel driven by a brushless DC motor, a printed circuit board schematic and layout for the motor controller, and a software architecture to interact with the ESP32 microcontroller.

The hardware consists of a reaction flywheel, its housing, the brushless DC motor, a frictionless air spindle, a test stand, an air compressor, and the PCB with its necessary components. More information can be found in the README.md in the hardware folder: (https://github.com/flamingmangoes/ME-507-Term-Project/blob/main/hardware/README.md)

The software relies on a FreeRTOS-based task architecture, and it has functionality for wireless speed and torque command of the motor, live speed readout, live gain tuning, and downloading a CSV of test data.
The task diagram is as follows: 
![taskdia](https://github.com/user-attachments/assets/21b8d215-8508-47d6-9404-0e94fe6045d7)

The interrupt service handler in the Driver class sends a timestamp to the edge_time queue when it detects a rising edge on the FGOUT pin. The readActual task takes each rising edge and uses the previous rising edge to calculate the electrical frequency of the motor. It then uses that with the direction to calculate the signed speed in RPM, and places that into the speed_actual share. This task runs whenever it detects a value in edge_time, with a latency of 1ms since it uses an ISR.

This share is then read by the webserver task with a period of 10ms, which plots it on a live readout. It is also read by the speedControl task, which then uses the embedded finite state machine (discussed in the next subsection) to decide how to command the motor. It calls the command_speed_PWM method shown earlier to command the motor speed. 

The webserver can command speeds and torques. When a value is input to the form, it places the command in its respective queue. When a speed is commanded, the speedControl task reads it directly. When a torque is commanded, the calcSetpoint task calls the Euler integrator method from the Controller class to convert that into a speed, then sends that to the speedControl task.

The state diagram for the speedControl task is as follows:
![statedia](https://github.com/user-attachments/assets/be05c1c3-1453-478f-9898-6013e76083c9)

The task starts in idle state. Once a value is placed in the speed_cmd queue, the state machine compares the magnitudes and signs of speed_cmd and the current value of speed_actual and decides in what state to place the motor. In the acceleration state, the motor is using its internal control loop to accelerate to the desired speed, then returns to IDLE once it reaches a deadband of 20rpm. In the deceleration state, the BRAKE pin is set to HIGH and the motor decelerates. (Right now, we cannot control the deceleration torque, but we are planning to implement a bang-bang controller to do so). In zero crossing states, once the motor reaches a deadband of 20 rpm around zero, the DIR pin switches polarity and starts accelerating to the desired speed.

When not in IDLE state, the task compares speed_cmd to speed_actual at a period of 10ms. When in IDLE state, the task blocks until another value is placed in the speed_cmd queue. 


Software documentation is included as a Doxygen-generated HTML file structure in the docs folder. The code itself is also well commented and defines all functions, classes, and variables.

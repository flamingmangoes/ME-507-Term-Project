/** @file main.cpp
 *  This program creates an interface to actuate a brushless DC motor as part of a momentum exchange device meant to 
 *  simulate attitude control maneuvers on a frictionless test platform. It allows for speed and torque control, and 
 *  has a simple web interface to wirelessly command the BLDC motor to any speed +/- 10 RPM.
 * 
 *  @author Neil Bedagkar and Bricen Rigby. 
 * 
 *  The code was written using help from ChatGPT 5.1 and Claude Sonnet 4.5. Most of the HTML script was written by AI. 
 *  The original code to setup a hotspot on the ESP32 was developed by John Ridgely, taken from an in-class example.
 * 
 *  In addition, the files @c taskshare.h and @c taskqueue.h are used; these files are in
 *  the ME507 support package at @c https://github.com/spluttflob/ME507-Support
 *  from which code can be used in PlatformIO or Arduino using the link which 
 *  is accessed using the green @b Code button on the GitHub page. 
 *  @c PrintStream.h is used to support these two files, which is in the package at
 *  @c https://github.com/spluttflob/Arduino-PrintStream.git.
 * 
 *  @date 12/9/2025
 * 
 *  @copyright (c) 2025 by Neil Bedagkar and Bricen Rigby, released under GPL 3.0
 */
#include <Arduino.h>
#include <SPI.h>
#include "Shares.h"
#include "Driver.h"
#include "Controller.h"
#include "taskshare.h"
#include "taskqueue.h"
#include <PrintStream.h>
#include "Server.h"
#include "CtrlTasks.h"

// A queue which holds torque command values from the webserver and passes them to the calcSetpoint task
Queue<float> torque_cmd (2, "Torque Command");

// A queue which holds speed command values from the webserver and passes them to the speedControl state machine
Queue<float> speed_cmd (2, "Speed Command"); 

// A share which populates using an ISR and holds the current speed of the motor
Share<float> speed_actual ("Speed Actual");

// A queue which uses an ISR to trigger the readActual task and calculate the motor speed by reading the square wave 
// frequency on the FGOUT pin
Queue<uint32_t> edge_time (4, "Rising Edge Timestamp");



// Create one object for the motor driver
Driver Peripheral;

// Create one object for the motor controller 
// (only used for integration right now, but will be used for PID calcs in the future)
Controller Controller_1;



/** @brief The Arduino setup function which runs once at setup. 
 * 
 *  @details This function sets up the serial monitor, initializes the DRV8308 chip, 
 *  sets up the webserver, and initializes each task.
 */
void setup()
{
    // Start the serial port and delay to give time for the user to open it
    Serial.begin(115200);
    delay(6000);

    // Initialize the motor driver with default gains and settings as defined in Driver.cpp
    Peripheral.begin();
    Serial.println("DRV initialized");

    // Set up the webserver
    setup_wifi();

    // Task which runs the web server, handling live plotting and speed/torque/gain commanding
    // This task runs every 10ms
    xTaskCreate (task_webserver, "Web Server", 8192, NULL, 1, NULL);

    // Task which calculates the actual speed of the motor based on an ISR
    // This task runs every time a rising edge is detected on FGOUT
    xTaskCreate(task_readActual, "Calculate RPM", 4096, NULL, 5, NULL);

    // Task which uses an integrator to calculate the speed from a commanded torque
    // This task runs every time a value is placed into torque_cmd
    // In the future this is how we will set our control loop frequency
    xTaskCreate(task_calcSetpoint, "Calculate Setpoint", 4096, NULL, 4, NULL);

    // Task which uses a state machine to command the motor speed
    // If in idle state, this task will not run until a value is placed into speed_cmd
    // This task will run every 10ms until back in idle state
    xTaskCreate(task_speedControl, "Speed Control", 4096, NULL, 3, NULL);
}



/** @brief The Arduino loop function which we do not use here */
void loop()
{
    vTaskDelay(5000);
}
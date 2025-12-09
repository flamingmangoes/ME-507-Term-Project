/** @file CtrlTasks.cpp
 *  This file contains the tasks used to command the motor speed or torque.
 */

#include <Arduino.h>
#include "Shares.h"
#include "Driver.h"
#include "Controller.h"
#include "taskshare.h"
#include "taskqueue.h"
#include "CtrlTasks.h"

/** Extern declarations for the objects created in main.cpp */
extern Driver Peripheral;
extern Controller Controller_1;

/** Extern declarations for the shares defined in main.cpp */
extern Queue<float> torque_cmd;
extern Queue<float> speed_cmd; 
extern Share<float> speed_actual;
extern Queue<uint32_t> edge_time;


/** @brief Function which returns the sign of the input
 * 
 *  @param x Floating point integer
 * 
 *  @return Sign of x
 */
inline int sign(float x) {
    return (x >= 0.0f) ? +1 : -1;  // 0.0 → +1
}


/** @brief Task which reads the speed of the motor
 * 
 *  @details The BLDC motor has Hall sensors which output a square wave at the electrical
 *  frequency of the motor. The DRV8308 chip outputs a square wave of this frequency, which is
 *  read by an ISR that puts a timestamp in the edge_time queue each time there is a rising edge. 
 *  This task gets that timestamp, compares it to the previous timestamp, calculates the frequency 
 *  of the motor in RPM and places that in the speed_actual queue. This task does not run until 
 *  there is a value in the edge_time queue, so it runs at the period of the motor spinning. The 
 *  motor speed is clamped to 2500 RPM by the Controller class, so the maximum speed of this task is 
 *  (2500/15) = 166.67 Hz or 6 ms.
*/
void task_readActual(void* parameters) 
{
    
    uint32_t last_time = micros();  // initialize the last_time to the time at setup
    uint32_t current_time = 0;      // initialize current_time to zero    
    float frequency = 0.0;          // initialize electrical frequency to zero
    float rpm = 0.0;                // initialize RPM to zero
    uint32_t dt_us = 0;             // initialize timestamp delta to zero
    bool direction = LOW;           // initialize direction boolean to low

    while (true) 
    {
        current_time = edge_time.get();     // Task only runs once there is a value in edge_time
        dt_us = current_time - last_time;   // calculate dt between rising edges
        last_time = current_time;           // set last_time to the current time after dt calculation

        // prevent dividing by zero error
        if (dt_us == 0)                     
        {
          continue;
        }

        // Calculate electrical frequency of motor by inverting dt_us and converting to seconds (Hz)
        frequency = 1 / (((float)dt_us) / 1000000); 

        // Use the direction of current motor spin to calculate positive or negative rpm
        direction = Peripheral.get_dir(); 
        if (direction == LOW) {rpm = frequency * 15.0f;}
        else {rpm = frequency * -15.0f;}

        // Place the calculated speed in the speed_actual share
        speed_actual.put(rpm);
    }
}



/** @brief Task which calculates the speed from a commanded torque
 * 
 *  @details This task calls the Controller class integrator to calculate a speed 
 *  command from a torque command. It runs once it detects a value in the torque_cmd queue
 *  so we can define the period of this task as the frequency of our control loop once we 
 *  expand the project.
 */
void task_calcSetpoint(void* parameters) 
{
    while (true) 
    {
        float torque = torque_cmd.get();
        float omega = Controller_1.calculate_omega(torque);
        speed_cmd.put(omega);
    }
}


/** @brief Task which commands the speed using a state machine
 *  
 *  @details This task uses a state machine to command the speed of the motor. The motor driver has an 
 *  internal control loop for acceleration but not for deceleration. It has a pin that applies an on/off 
 *  BRAKE and a pin to control the direction. The state machine uses the commanded speed and actual speed 
 *  torques to switch between an idle / stable state (acceleration = zero), an acceleration state (which 
 *  uses the internal control loop of the driver), a deceleration state (which uses the brake pin), and 
 *  zero crossing states in each direction which switch the direction pin polarity in a deadband of 20rpm. 
 *  In the future, we will write a controller to apply a bang-bang brake PWM to control the deceleration torque.
 *  This state machine triggers from state 0 (idle/stable) when there is a value in the speed_cmd queue then 
 *  runs every 10ms until the respective deadband is met.
 */
void task_speedControl(void* parameters)
{
    // 0 = idle / stable, 1 = accel, 2 = decel, 3 = hi to lo crossing, 4 = lo to hi crossing
    uint8_t speed_state = 0; 

    float speed_command = 0.0;      // initialize internal speed command variable to zero
    float speed_real = 0.0;         // initialize internal actual speed variable to zero
    bool direction = LOW;           // initialize direction to positive (LO = + in this convention)
    Peripheral.set_dir(direction);  // set initial direction to positive

    while (true) 
    {        
        speed_real = speed_actual.get();  // every time the while loop runs it reads the actual speed
        direction = Peripheral.get_dir(); // read direction pin

        // idle / stable state
        // includes logic for each of six possibilities for speed comparisons
        // pos to larger or smaller pos, neg to larger or smaller neg, pos to neg, neg to pos
        // includes state transitions to accel or decel for each case
        if (speed_state == 0) 
        {
            // when speed is zero or stable (+/-20rpm), the task will not run until it gets a speed command
            speed_command = speed_cmd.get(); 

            // logic for + > ++, - > --, or - > +
            if (speed_command > speed_real) 
            {
                if (sign(speed_command) == sign(speed_real)) 
                {
                    // positive to larger positive
                    if (direction == LOW) 
                    {
                        // Command speed 
                        Peripheral.cmd_speed_PWM(fabsf(speed_command)); 

                        // accel
                        speed_state = 1; 
                    }

                    // negative to smaller negative
                    else 
                    {
                        // Command zero and brake the motor
                        Peripheral.cmd_speed_PWM(0);
                        Peripheral.brake();

                        // decel
                        speed_state = 2; 
                    }
                }

                // negative to positive
                else
                {
                    Peripheral.cmd_speed_PWM(0);
                    Peripheral.brake();
                    // decel
                    speed_state = 2; 
                }
            }

            // logic for ++ > +, -- > -, or + > -
            else if (speed_command < speed_real) 
            {
                if (sign(speed_command) == sign(speed_real))
                {
                    // positive to smaller positive
                    if (direction == LOW)
                    {
                        // Command zero and brake the motor
                        Peripheral.cmd_speed_PWM(0);
                        Peripheral.brake();

                        // decel
                        speed_state = 2; 
                    }

                    // negative to larger negative
                    else 
                    {
                        // Command speed
                        Peripheral.cmd_speed_PWM(fabsf(speed_command));

                        // accel
                        speed_state = 1; 
                    }
                }

                 // positive to negative
                else
                {
                    // Command zero and brake the motor
                    Peripheral.cmd_speed_PWM(0); 
                    Peripheral.brake();

                    // decel
                    speed_state = 2;
                }
            }
        }
        
        // acceleration state, cmd_speed_PWM to desired speed
        // includes state transition from acceleration state back to idle state
        else if (speed_state == 1) 
        {
            // state transition from accel back to idle, deadband 20rpm
            if (fabsf(speed_real-speed_command) <= 20) 
            {
                speed_state = 0;
            }

            // or delay 10ms until the deadband is reached
            else {vTaskDelay(10);}
        }

        // deceleration state, cmd_speed_PWM is at zero
        // includes logic for no direction change and state transitions for direction changes
        else if (speed_state == 2) 
        {
            // no direction change
            if (sign(speed_command) == sign(speed_real))
            { 
                // state transition from decel back to idle, deadband 20rpm
                if (fabsf(speed_real-speed_command) <= 20) 
                {
                    // unbrake first, then command speed to speed command
                    Peripheral.unbrake();
                    Peripheral.cmd_speed_PWM(fabsf(speed_command));

                    // return to stable state
                    speed_state = 0;
                }
            }

            // direction change
            else
            {
                if (direction == HIGH) // negative to positive
                {
                    if (fabsf(speed_real) < 20.0f) // deadband 20rpm
                    speed_state = 3;
                }
                else // positive to negative
                {
                    if (fabsf(speed_real) < 20.0f) // deadband 20rpm
                    speed_state = 4;
                }
            }   

            // or delay 10ms if the deadband conditions are not met and a state transition
            // does not occur
            vTaskDelay(10); 
        }

        // HI to LO zero crossing
        // negative to positive speed
        // then state transition to accel
        else if (speed_state == 3) 
        {
            // change direction
            direction = LOW;
            Peripheral.set_dir(direction);

            // unbrake the motor and command the proper speed
            Peripheral.unbrake();
            Peripheral.cmd_speed_PWM(fabsf(speed_command));

            // accel
            speed_state = 1;
        }

        // LO to HI zero crossing
        // positive to negative speed
        // then state transition to accel
        else if (speed_state == 4) 
        {
            // change direction
            direction = HIGH;
            Peripheral.set_dir(direction);

            // unbrake the motor and command the proper speed
            Peripheral.unbrake();
            Peripheral.cmd_speed_PWM(fabsf(speed_command));

            // accel
            speed_state = 1;
        }
    }
}
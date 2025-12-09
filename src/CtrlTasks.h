/** @file CtrlTasks.h
 *  This file contains the tasks used to command the motor speed or torque.
 */

#ifndef _CTRLTASKS_H_
#define _CTRLTASKS_H_

/** These tasks are commented in CtrlTasks.cpp */
void task_readActual(void* p_params);
void task_calcSetpoint(void* p_params);
void task_speedControl(void* p_params);

#endif

/*
 * Motor_control.h
 *
 *  Created on: Oct 15, 2025
 *      Author: idgaf
 */

#ifndef DRIVERS_INCLUDES_MOTOR_CONTROL_H_
#define DRIVERS_INCLUDES_MOTOR_CONTROL_H_
#include "structs.h"
/**
 * @brief Set the commanded position for a motor.
 * @param motor_index Index of the motor in the array.
 * @param position Target position in degrees.
 */
void Set_Motor_Position(struct MotorStruct *m, int position);

void Set_Motor_Angle(struct MotorStruct *m, float angle);

/**
 * @brief Set the commanded velocity for a motor.
 * @param motor_index Index of the motor in the array.
 * @param velocity Target velocity (in steps per second).
 */
void Set_Motor_Velocity(struct MotorStruct *motor, float velocity);
void Set_Motor_Angular_Velocity(struct MotorStruct *motor, float velocity);

/**
 * @brief Stop the motor (sets velocity to 0, disables motion).
 * @param motor_index Index of the motor in the array.
 */
void Stop_Motor(struct MotorStruct *m);

void Home_Motors();

#endif /* DRIVERS_INCLUDES_MOTOR_CONTROL_H_ */

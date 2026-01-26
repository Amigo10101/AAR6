/*
 * limit_it_config.c
 *
 *  Created on: Oct 22, 2025
 *      Author: idgaf
 */
#include "structs.h"
#include "constants.h"
#include "Motor_control.h"
extern struct MotorStruct motors[NUMBER_OF_JOINTS];
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{	    uint32_t now = HAL_GetTick();   // current time in milliseconds

    // Loop through all motors
    for (int i = 0; i < NUMBER_OF_JOINTS-1; i++) {
        struct MotorStruct *m = &motors[i];

        // Check if this interrupt is for the motor's limit switch
        if (GPIO_Pin == m->LIMIT) {

        	if (now - m->last_limit_trigger_time < 20) {
        	                return;    // ignore bounce
        	            }
			m->last_limit_trigger_time = now;
            if (m->homing){

                    m->moving = false;
                    m->commanded_mode = STOPPED;
                    m->commanded_velocity = 0;

                    HAL_TIM_PWM_Stop_IT(m->timer_instance, m->timer_channel);
                    HAL_TIM_PWM_Stop(m->timer_instance, m->timer_channel);
            	m->position=m->homed_position;
            	Set_Motor_Position(m, m->standby_position);
            	m->homed=true;
            	m->homing=false;

            }

        }
    }
}


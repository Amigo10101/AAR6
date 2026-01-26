/*
 * step_timer_it_config.c
 *
 * Motor Step Timer ISR - handles position counting for all motors
 *
 * With the tick-based S-curve architecture:
 * - TIM7 ISR (20kHz): Updates S-curve velocity profile, sets PWM ARR
 * - This ISR (per step): Counts position, checks if target reached
 *
 * Created on: Sep 1, 2025
 * Updated: Jan 17, 2026 - Simplified for tick-based S-curve
 * Author: idgaf
 */
#include "constants.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "structs.h"

extern struct MotorStruct motors[NUMBER_OF_JOINTS];

/**
 * @brief Motor Timer Interrupt Service Routine
 *
 * Called on each step pulse (timer update event).
 * Updates motor position counter and checks if target is reached.
 *
 * @param htim  Timer handle that triggered the interrupt
 */
void MotorTimerISR(TIM_HandleTypeDef *htim) {
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    struct MotorStruct *m = &motors[i];

    if (htim->Instance != m->timer_instance->Instance)
      continue;

    /* -------------------------------------------------
     * STEP OCCURRED → UPDATE POSITION
     * ------------------------------------------------- */
    if (m->direction == cw)
      m->position++;
    else
      m->position--;

    /* Sync S-curve position with motor position */
    m->s.guipos = (float)m->position;

    /* -------------------------------------------------
     * CHECK IF TARGET REACHED → STOP MOTOR
     * ------------------------------------------------- */
    if (m->commanded_mode == POSITION) {
      if (m->position == m->commanded_position) {
        /* Target reached - stop everything */
        m->s.active = 0;
        m->moving = false;
        m->commanded_mode = STOPPED;
        m->velocity = 0.0f;
        m->acceleration = 0.0f;
        HAL_TIM_PWM_Stop_IT(m->timer_instance, m->timer_channel);
      }
    }

    /* Only process one motor per ISR call */
    break;
  }
}

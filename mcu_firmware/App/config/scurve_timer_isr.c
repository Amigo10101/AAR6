/*
 * scurve_timer_isr.c
 *
 * TIM7 ISR for S-curve motion profile updates
 *
 * This ISR runs at 20kHz and updates the velocity profile for all motors.
 * It then updates each motor's PWM timer ARR to achieve the calculated
 * velocity.
 *
 * Created on: Jan 17, 2026
 * Author: idgaf
 */

#include "scurve_engine.h"
#include "stm32f4xx_hal.h"
#include "structs.h"
#include "tim.h"

extern struct MotorStruct motors[NUMBER_OF_JOINTS];
extern TIM_HandleTypeDef htim7;

/**
 * @brief S-Curve Tick Processor - call from TIM7_IRQHandler in stm32f4xx_it.c
 *
 * Called at 20kHz (50µs intervals). For each motor in POSITION mode:
 * 1. Updates the S-curve state
 * 2. Gets the new ARR value based on calculated velocity
 * 3. Updates the motor's PWM timer to generate steps at the new rate
 */
void SCurve_Tick_Update_ISR(void) {
  /* Process each motor */
  for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
    struct MotorStruct *m = &motors[i];

    /* Only process motors in POSITION mode with active S-curve */
    if (m->commanded_mode == POSITION && m->s.active) {

      /* Update S-curve state - returns new ARR value */
      uint32_t new_arr = scurve_update(&m->s);

      if (new_arr == 0 || scurve_is_finished(&m->s)) {
        /* Motion complete - stop PWM */
        m->s.active = 0;
        m->moving = false;
        m->commanded_mode = STOPPED;
        m->velocity = 0.0f;
        m->acceleration = 0.0f;
        HAL_TIM_PWM_Stop_IT(m->timer_instance, m->timer_channel);
      } else {
        /* Update PWM timer with new ARR */
        m->arr_value = new_arr;
        __HAL_TIM_SET_AUTORELOAD(m->timer_instance, m->arr_value);
        __HAL_TIM_SET_COMPARE(m->timer_instance, m->timer_channel,
                              m->arr_value / 2);

        /* Update telemetry values */
        m->velocity = scurve_get_velocity(&m->s);
        m->acceleration = scurve_get_acceleration(&m->s);
      }
    }
  }
}

/**
 * @brief Start TIM7 for S-curve tick processing
 *
 * Call this once during initialization after TIM7 is configured.
 */
void SCurve_Tick_Start(void) {
  /* Enable TIM7 interrupt */
  HAL_NVIC_SetPriority(TIM7_IRQn, 4, 0); /* Higher priority than motor ISRs */
  HAL_NVIC_EnableIRQ(TIM7_IRQn);

  /* Start TIM7 in interrupt mode */
  HAL_TIM_Base_Start_IT(&htim7);
}

/**
 * @brief Stop TIM7 tick processing
 */
void SCurve_Tick_Stop(void) {
  HAL_TIM_Base_Stop_IT(&htim7);
  HAL_NVIC_DisableIRQ(TIM7_IRQn);
}

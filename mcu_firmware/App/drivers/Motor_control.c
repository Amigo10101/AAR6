#include "Motor_control.h"
#include "main.h"
#include "scurve_engine.h"
#include <cmsis_os2.h>
#include <constants.h>
#include <math.h>
#include <stdbool.h>
#include <stm32f446xx.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_tim.h>
#include <sys/_stdint.h>

#define VEL_GAP 0.001
extern struct MotorStruct motors[NUMBER_OF_JOINTS];

static inline void Motor_UpdatePhysicalDIR(struct MotorStruct *m,
                                           Directionenum dir) {
  m->direction = dir;

  // Base logic: CW=RESET (0), CCW=SET (1)
  // This matches the enum values where cw=0, ccw=1
  GPIO_PinState pin_state = (dir == cw) ? GPIO_PIN_RESET : GPIO_PIN_SET;

  if (m->direction_flipped) {
    pin_state = (pin_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }
  HAL_GPIO_WritePin(m->DIR_Port, m->DIR, pin_state);
}

static inline void Motor_SetDirection(struct MotorStruct *m, int32_t target) {
  if (target > m->position) {
    Motor_UpdatePhysicalDIR(m, cw);
  } else {
    Motor_UpdatePhysicalDIR(m, ccw);
  }
}

void Motor_scurve_start(struct MotorStruct *m, int32_t target) {
  if (target == m->position)
    return;

  Motor_SetDirection(m, target);
  m->commanded_mode = POSITION;
  m->commanded_position = target;

  // Initialize S-curve state with motor limits
  scurve_init(&m->s, (float)m->motor_max_jerk, (float)m->motor_max_acceleration,
              (float)m->motor_max_speed);

  // Start move and get first ARR
  uint32_t initial_arr =
      scurve_move_to(&m->s, (float)m->position, (float)target);

  m->moving = true;
  m->arr_value = initial_arr;

  // Initialize PWM for motor
  __HAL_TIM_SET_AUTORELOAD(m->timer_instance, m->arr_value);
  __HAL_TIM_SET_COMPARE(m->timer_instance, m->timer_channel, m->arr_value / 2);
  HAL_TIM_PWM_Start_IT(m->timer_instance, m->timer_channel);
  __HAL_TIM_ENABLE_IT(m->timer_instance, TIM_IT_UPDATE);
}

void Motor_scurve_retarget(struct MotorStruct *m, int32_t target) {
  if (target == m->position) {
    scurve_stop(&m->s);
    m->moving = false;
    m->commanded_mode = STOPPED;
    HAL_TIM_PWM_Stop_IT(m->timer_instance, m->timer_channel);
    return;
  }

  Motor_SetDirection(m, target);
  m->commanded_position = target;

  // For retargeting, we restart the move with new target
  // The scurve_engine will handle the transition smoothly
  scurve_move_to(&m->s, scurve_get_position(&m->s), (float)target);
}

void Set_Motor_Angle(struct MotorStruct *m, float angle) {
  int32_t target = (int32_t)llroundf(angle * m->steps_per_radian);

  /* Always start fresh - Motor_scurve_start handles initialization properly
   * This avoids issues with stale s.active values */
  Motor_scurve_start(m, target);
}

/**
 * @brief Set the commanded velocity for a motor
 * @param motor_index Index of the motor in the array
 * @param velocity Target velocity
 */
void Set_Motor_Velocity(struct MotorStruct *m, float velocity) {
  double vel_abs = fabs((double)velocity);
  m->commanded_velocity = velocity;
  m->commanded_mode = VELOCITY;

  /* Ensure S-curve is deactivated when entering velocity mode */
  m->s.active = 0;
  m->s.finish = 1;
  if (vel_abs <= VEL_GAP) {
    Stop_Motor(m);
    return;
  }
  if (velocity >= 0) {
    Motor_UpdatePhysicalDIR(m, cw);
  } else {
    Motor_UpdatePhysicalDIR(m, ccw);
    velocity = -velocity; // Make velocity positive for PWM calculation
  }
  m->moving = true;
  uint32_t timer_freq =
      APB1_TIMER_CLOCK_FREQ / (APB1_TIMER_PSC + 1); // Timer tick frequency
  uint32_t arr = timer_freq / velocity; // Auto-reload for desired PWM freq
  m->arr_value = arr;
  // Limit ARR to 16-bit (for TIMx)
  if (arr > 0xFFFF)
    arr = 0xFFFF;

  __HAL_TIM_SET_AUTORELOAD(m->timer_instance, arr - 1);
  __HAL_TIM_SET_COMPARE(m->timer_instance, m->timer_channel,
                        arr / 2); // 50% duty

  HAL_TIM_PWM_Start(m->timer_instance, m->timer_channel);
  HAL_TIM_PWM_Start_IT(m->timer_instance, m->timer_channel);
  __HAL_TIM_ENABLE_IT(m->timer_instance, TIM_IT_UPDATE);
}

void Set_Motor_Angular_Velocity(struct MotorStruct *m, float velocity) {
  float vel_abs = fabsf(velocity);

  // Deadband
  if (vel_abs <= VEL_GAP) {
    Stop_Motor(m);
    return; // <-- MUST EXIT
  }

  m->commanded_velocity = velocity;
  m->commanded_mode = VELOCITY;

  /* Ensure S-curve is deactivated when entering velocity mode */
  m->s.active = 0;
  m->s.finish = 1;

  if (velocity >= 0) {
    Motor_UpdatePhysicalDIR(m, cw);
  } else {
    Motor_UpdatePhysicalDIR(m, ccw);
    velocity = -velocity;
  }

  velocity *= m->steps_per_radian;

  uint32_t timer_freq = APB1_TIMER_CLOCK_FREQ / (APB1_TIMER_PSC + 1);
  uint32_t arr = timer_freq / velocity;

  if (arr > 0xFFFF)
    arr = 0xFFFF;
  m->arr_value = arr;
  __HAL_TIM_SET_AUTORELOAD(m->timer_instance, arr);
  __HAL_TIM_SET_COMPARE(m->timer_instance, m->timer_channel, arr / 2);

  if (!m->moving) {
    HAL_TIM_PWM_Start(m->timer_instance, m->timer_channel);
    HAL_TIM_PWM_Start_IT(m->timer_instance, m->timer_channel);
    __HAL_TIM_ENABLE_IT(m->timer_instance, TIM_IT_UPDATE);
  }

  m->moving = true;
}

/**
 * @brief Stop the motor (sets velocity to 0, keeps position if needed)
 * @param motor_index Index of the motor in the array
 */
void Stop_Motor(struct MotorStruct *m) {
  m->commanded_velocity = 0;
  m->commanded_mode = STOPPED;
  m->moving = false;
  HAL_TIM_PWM_Stop_IT(m->timer_instance, m->timer_channel);
  HAL_TIM_PWM_Stop(m->timer_instance, m->timer_channel);
}

void Set_Motor_Position(struct MotorStruct *m, int position) {

  Motor_scurve_start(m, position);
}

static void wait_motor_stop(uint8_t motor_index) {
  struct MotorStruct *m = &motors[motor_index];
  while (m->moving) {
    osDelay(10);
  }
}

void Home_Motors() {
  for (int i = 0; i < NUMBER_OF_JOINTS-2; i++) {
    struct MotorStruct *m = &motors[i];
    GPIO_PinState state = HAL_GPIO_ReadPin(m->LIMIT_Port, m->LIMIT);
    if (state == m->limit_val) {
      m->position = m->homed_position;
      m->homed = true;
      m->homing = false;
      Set_Motor_Position(m, m->standby_position);
    } else {
      m->homing = true;
      int8_t multiplier = 1;
      if (m->homing_direction == ccw) {
        multiplier = -1;
      }
      Set_Motor_Velocity(m, multiplier * m->homing_speed);
    }
  }
}
void Dehome_motors() {
  float vals[NUMBER_OF_JOINTS] = {0, -51, 74, 0, 0, 0, 0};
  for (int i = 0; i < NUMBER_OF_JOINTS - 3; i++) {
    Set_Motor_Angle(&motors[i], vals[i]);
  }
}

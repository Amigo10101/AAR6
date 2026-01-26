/*
 * scurve_engine.h
 *
 * S-Curve Motion Profile Engine
 * EXACT port from grotius-cnc/scurve_construct
 * Only change: double -> float for STM32 FPU
 */

#ifndef SCURVE_ENGINE_H
#define SCURVE_ENGINE_H

#include <math.h>
#include <stdint.h>

/*============================================================================
 * CONFIGURATION
 *============================================================================*/
#define SCURVE_TICK_INTERVAL 0.00005f   /* 50µs = 1/20000 Hz */
#define SCURVE_PWM_TIMER_FREQ 2000000UL /* 2 MHz */
#define SCURVE_ARR_MIN 50
#define SCURVE_ARR_MAX 50000

/*============================================================================
 * DATA STRUCTURES - Exact copy from grotius scurve_struct.h
 *============================================================================*/

/* Struct used to save scurve periodic data */
typedef struct {
  float accbeg, accend;
  float velini, velbeg, velend;
  float disbeg, disend;
  float timbeg, timend;
  float jermax, accinf;
} scurve_period_t;

/* Position control data */
typedef struct {
  float stopdist;
  float stoptime;
  float overshoot;
  float dist_remove_a_cycle;
  int stopinit;
  int btn_fwd;
  int btn_rev;
} position_data_t;

/* Main scurve state - exact copy from grotius scurve_data */
typedef struct {
  scurve_period_t c0, c1, c2, c3,
      c4;       /* Periods to construct forward/stop scurve */
  float intval; /* Cycletime */
  float jermax; /* Jerk max */
  float curtim; /* Time current */
  float endacc; /* Acceleration end */
  float endvel; /* Velocity end */
  float maxvel; /* Velocity max */
  float maxacc; /* Acceleration max */
  float guipos; /* Current position */
  float guivel; /* Current velocity */
  float guiacc; /* Current acceleration */
  int finish;   /* Curve has no velocity */

  float incpos; /* Increment position for each cycle */
  float oldpos; /* Previous displacements */

  float tarpos; /* Target position */

  float vr, ar, sr; /* Results: velocity, acceleration, displacement */

  int revers;         /* Reverse motion active */
  int modpos;         /* Mode position */
  position_data_t pd; /* Position control data */

  /* Added for MCU integration */
  int active;            /* Motion active flag */
  uint32_t arr_value;    /* PWM ARR value */
  uint16_t tick_counter; /* Counter for periodic decel checks */
} scurve_state_t;

/* How often to check decel (every N ticks) - 20 = 1kHz check rate */
#define SCURVE_DECEL_CHECK_INTERVAL 20

/*============================================================================
 * API FUNCTIONS
 *============================================================================*/

/* Initialize with motion limits */
void scurve_init(scurve_state_t *s, float jerk_max, float accel_max,
                 float vel_max);

/* Start position move - call once when target changes */
uint32_t scurve_move_to(scurve_state_t *s, float current_pos, float target_pos);

/* Update - call at fixed rate (20kHz) - LIGHTWEIGHT, just plays curves */
uint32_t scurve_update(scurve_state_t *s);

/* Stop motion */
void scurve_stop(scurve_state_t *s);

/* Inline getters */
static inline int scurve_is_finished(const scurve_state_t *s) {
  return s->finish;
}
static inline float scurve_get_velocity(const scurve_state_t *s) {
  return s->guivel;
}
static inline float scurve_get_position(const scurve_state_t *s) {
  return s->guipos;
}
static inline float scurve_get_acceleration(const scurve_state_t *s) {
  return s->guiacc;
}

/* Convert velocity to ARR */
uint32_t scurve_velocity_to_arr(float velocity);

#endif /* SCURVE_ENGINE_H */

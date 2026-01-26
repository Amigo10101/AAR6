/*
 * structs.c
 *
 *  Created on: Aug 31, 2025
 *      Author: idgaf
 */

#ifndef INCLUDES_STRUCTS_H_
#define INCLUDES_STRUCTS_H_
#include "constants.h"
#include "main.h"
#include "stdbool.h"
typedef enum {
  STOPPED,
  POSITION,
  VELOCITY,
} MotionType;
typedef enum {
  cw,
  ccw,
} Directionenum;

typedef enum {
  ROBOT_STATE_ERROR = 0,    // Any fault or error state
  ROBOT_STATE_WAIT_PC = 1,  // Waiting for PC connection
  ROBOT_STATE_START = 2,    // Robot has just powered on, before initialization
  ROBOT_STATE_DISABLED = 3, // Robot is disabled
  ROBOT_STATE_HOMING = 4,   // Robot is performing homing routine
  ROBOT_STATE_HOMED = 5,    // Robot has completed homing
  ROBOT_STATE_ESTOP = 6,    // Emergency stop active
  ROBOT_STATE_STANDSTILL = 7, // Robot is idle, motors not moving also homed
  ROBOT_STATE_MOVING = 8,     // Robot is executing a motion
  ROBOT_STATE_EXECUTING = 9,  // Robot executing a command/trajectory

} RobotState_t;

typedef enum {
  CMD_NONE = 0,          // No command
  CMD_ACK = 1,           // found by PC
  CMD_START = 2,         // start the hoimng process
  CMD_ESTOP = 3,         // Emergency stop
  CMD_ESTOP_RELEASE = 4, // Release E-stop
  CMD_HOME = 5,          // Start homing
  CMD_DEHOME = 6,        // Dehome (reset home)
  CMD_DISABLE = 7        // Disable motors
} ControlCommand_t;

typedef enum {
  CMD_MOTION_POSITION = 0x20, // use position, ignore velocity
  CMD_MOTION_VELOCITY = 0x21, // use velocity, ignore position
} MotionCommand_t;

typedef enum {
  MOTION_START = 0x01,
  MOTION_UPDATE = 0x02,
  MOTION_STOP = 0x03,
  MOTION_HOLD = 0x04,
} MotionAction_t;

typedef enum {
  PKT_TELEMETRY = 0x01,
  PKT_LOG = 0xF0,
} PacketType_t;

typedef enum {
  LOG_INFO = 0,
  LOG_WARN = 1,
  LOG_ERROR = 2,
  LOG_DEBUG = 3
} LogLevel_t;

/* Include S-curve engine (ported from grotius scurve_construct) */
#include "scurve_engine.h"

/* Alias for MotorStruct compatibility */
typedef scurve_state_t SCurveProfile;
/**
 * @brief Defines the complete state and configuration for a single motor joint.
 */
struct MotorStruct {

  // -- 1. Identification --
  // General identifier for the joint.
  char name[32];
  SCurveProfile s;

  float lqr_x_hat[2]; // [est_pos, est_vel]
  float lqr_x_i;      // [integrated_error]
  float lqr_r_cmd;    // [commanded_position_target]
  float lqr_current_velocity;
  // -- 2. Hardware Assignments (Constant) --
  // These map the logical joint to physical MCU resources.
  // They are 'const' as they are set at compile-time.
  TIM_HandleTypeDef *const timer_instance; // Timer for step pulse generation
  const uint32_t timer_channel;            // <-- ADDED (e.g., TIM_CHANNEL_1)
  const uint16_t STEP;                     // STEP output pin
  const uint16_t DIR;                      // DIRECTION output pin
  const uint16_t LIMIT;                    // LIMIT switch input pin
  const uint8_t limit_val;
  const GPIO_TypeDef *STEP_Port;
  GPIO_TypeDef *DIR_Port;
  const GPIO_TypeDef *LIMIT_Port;
  uint32_t last_limit_trigger_time; // for debounce

  // -- 3. Mechanical & Kinematic Configuration (Constant) --s
  // Static properties of the joint's mechanics.
  const float reduction_ratio; // Gearbox reduction ratio
  const int microstep;         // Microstep setting (e.g., 16, 256)
  const float steps_per_radian;
  const float radians_per_step;
  const bool direction_flipped;

  // -- 4. Electrical Configuration (Constant) --
  // Settings for the stepper driver IC.
  const int motor_max_current; // Rated RMS current of the motor
  const int irun;              // Run current scale factor (0-31)
  const int ihold;             // Hold current scale factor (0-31)

  // -- 5. Motion Profile & Dynamic Limits (Constant) --
  // Defines the physical performance limits for motion planning.
  const int motor_max_speed;
  const int motor_min_speed;
  const int motor_max_acceleration;
  const int motor_min_acceleration;
  const int motor_max_jerk;
  const float homing_speed; // Specific speed for homing routine

  // -- 6. Positional Limits & Setpoints (Constant & Variable) --
  // Defines the working envelope and key positions.
  const float joint_range_positive;     // Max positive angle (radians)
  const float joint_range_negative;     // Max negative angle (radians)
  const int joint_range_positive_steps; // Max positive position (steps)
  const int joint_range_negative_steps; // Max negative position (steps)

  // These are set by the application, often after homing.
  int standby_position; // Position to move to when idle (steps)
  float homed_position; // Target position after homing (steps)

  /**
   * @brief The motor step count that corresponds to the encoder's
   * zero-degree position. This is set during calibration.
   */
  int encoder_zero_step;
  float encoder_position;
  int encoder_channel;

  // -- 7. Command Interface --
  // These members are set by the main control loop to command the joint.
  MotionType commanded_mode; // Target mode (e.g., POSITION, VELOCITY)
  int commanded_position;    // Target position (steps)
  double commanded_velocity; // Target velocity (steps/s)
  int commanded_current;     // Target current (raw value or mA)
  int start_position;        // Position at the start of a new move

  // -- 8. Real-Time State --
  // These values are updated continuously by the control system/ISRs.
  int prev_velocity;
  int position;              // Current position (steps)
  float velocity;            // Current velocity (steps/s)
  float acceleration;        // Current acceleration (steps/s^2)
  int current;               // Current motor current
  Directionenum direction;   // Current direction of motion
  bool moving;               // True if a motion profile is active
  bool limit_switch_trigger; // Current state of the limit switch

  // --  ENCODER DATA --
  /**
   * @brief Raw 12-bit (0-4095) angle value from the AS5600 encoder.
   */
  uint16_t encoder_raw_position;

  // Internal timer/state values
  uint16_t arr_value; // Timer ARR value for pulse generation

  // -- 9. Homing State Machine --
  // Flags that manage the multi-stage homing process.
  int homed;          // True if homing is complete
  int homing;         // True if homing is in progress
  int homing_stage_1; // Flag for homing stage 1
  int homing_stage_2; // Flag for homing stage 2
  const Directionenum homing_direction;

  // -- 10. Diagnostic & Error Flags --
  // Flags reported by the driver or control logic for fault detection.
  int error;          // A general error flag
  int position_error; // Flag: Tried to move out of valid range

  // Driver-reported temperature flags
  int temperature_warning; // Driver temperature warning (TWARNN)
  int temperature_error;   // Driver temperature error (OT)

  // Duplicates/Alternative names from original struct
  int over_temp_pre_warning; // (Recommend merging with temperature_warning)
  int over_temp_warning;     // (Recommend merging with temperature_error)

  // Driver-reported diagnostic flags
  int diag0;         // DIAG0 pin state
  int open_load_A;   // Open load on phase A
  int open_load_B;   // Open load on phase B
  int short_2_gnd_A; // Short to ground on phase A
  int short_2_gnd_B; // Short to ground on phase B
};
struct Robot {

  int command;
  int affected_joint;
  int Estop;
  int communication_error;
  int disabled;
  RobotState_t state;
};

#endif /* INCLUDES_STRUCTS_C_ */

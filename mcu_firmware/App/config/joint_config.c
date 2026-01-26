/*
 * motor_config.c
 *
 * Created on: Aug 31, 2025
 * Author: idgaf
 */
#include "constants.h"
#include "main.h"
#include "stdbool.h"
#include "structs.h"
#include "tim.h"
#include <string.h>
// --- Assumed Motor Constants ---
// Standard 1.8 deg/step motor
#define MOTOR_BASE_STEPS_PER_REV 200.0f
// Define Pi for radian calculations
#define M_PI 3.14159265358979323846f

#define SPEED_CALC(x) ((x) * (MICROSTEP))

#define STEP_CALC(x) ((int)((x) * (MICROSTEP)))

static const float g_reduction_ratios[NUMBER_OF_JOINTS] = {
    [0] = 6.4f,        // L1: 96 / 15
    [1] = 20.0f,       // L2
    [2] = 18.0952381f, // L3: 20 * (38 / 42.0f)
    [3] = 4.0f,        // L4
    [4] = 4.0f,        // L5
    [5] = 10.0f,       // L6
    [6] = 1.0f         // GRP
};

struct Robot robot = {.command = 0,
                      .affected_joint = 0,
                      .Estop = 0,
                      .communication_error = 0,
                      .disabled = 0,
                      .state = ROBOT_STATE_START};

struct MotorStruct motors[NUMBER_OF_JOINTS] =
    {
        [0] =
            {
                // J1 is special because we usually go to the position that
                // is +90 deg from its Denevit Hartenberg standby positon
                // + 90 deg is equal to 10240 steps in 32microstep range
                // -- 1. Identification --
                .name = "L1",

		        .timer_instance = &htim12,
		                .timer_channel = TIM_CHANNEL_1,
		                .STEP = STEP7_Pin, // Note: STEP pin is shared with L6
		                .STEP_Port =
		                    STEP7_GPIO_Port,        // Note: STEP port is shared with L6
		                .DIR = DIR7_Pin,            // Using L1's DIR pin
		                .DIR_Port = DIR7_GPIO_Port, // Using L1's DIR port
                .LIMIT = LIMIT1_Pin,
                .LIMIT_Port = LIMIT1_GPIO_Port,
                .direction_flipped = true,
                .limit_val = 1,
                // -- 3. Mechanical & Kinematic Configuration (Constant) --
                .reduction_ratio = g_reduction_ratios[0], // 6.4f (96 / 15)
                .microstep = MICROSTEP,
                .steps_per_radian =
                    (MOTOR_BASE_STEPS_PER_REV * MICROSTEP / (2.0f * M_PI)) *
                    g_reduction_ratios[0],
                .radians_per_step =
                    ((2.0f * M_PI) / MOTOR_BASE_STEPS_PER_REV / MICROSTEP) /
                    g_reduction_ratios[0],

                // -- 4. Electrical Configuration (Constant) --
                .motor_max_current = MOTOR1_MAX_CURRENT,
                .irun = 12,
                .ihold = 12,

                // -- 5. Motion Profile & Dynamic Limits (Constant) --
                // S-curve tuning: t_jerk = accel/jerk = 8000/40000 = 0.2s
                // d_accel = v²/(2a) = 6000²/16000 = 2250 steps per phase
                .motor_max_speed = 6000, // steps/s - moderate speed
                .motor_min_speed = 0,
                .motor_max_acceleration = 8000, // steps/s² - gentler
                .motor_min_acceleration = 0,
                .motor_max_jerk = 40000, // steps/s³ - smooth transitions
                .homing_speed = SPEED_CALC(100),

                // -- 6. Positional Limits & Setpoints --
                .joint_range_positive = 0.0f,
                .joint_range_negative = 0.0f,
                .joint_range_positive_steps = 14000,
                .joint_range_negative_steps = -14000,
                .standby_position = 0,
                .homed_position = STEP_CALC(540),
                .encoder_zero_step = 0,
                .encoder_channel = 4,

                // -- 7. Command Interface --
                .commanded_mode = STOPPED,
                .commanded_position = 0,
                .commanded_velocity = 0.0,
                .commanded_current = 0,
                .start_position = 0,

                // -- 8. Real-Time State (Volatile) --
                .position = 0,
                .velocity = 0.0f,
                .acceleration = 0.0f,
                .current = 0,
                .direction = 0,
                .moving = false,
                .limit_switch_trigger = false,
                .encoder_raw_position = 0,
                .arr_value = 0,

                // -- 9. Homing State Machine --
                .homed = 0,
                .homing = 0,
                .homing_direction = cw,

                // -- 10. Diagnostic & Error Flags --
                .error = 0,
                .position_error = 0,
                .temperature_warning = 0,
                .temperature_error = 0,
                .over_temp_pre_warning = 0,
                .over_temp_warning = 0,
                .diag0 = 0,
                .open_load_A = 0,
                .open_load_B = 0,
                .short_2_gnd_A = 0,
                .short_2_gnd_B = 0,
            },
        [1] =
            {
                // L2
                // -- 1. Identification --
                .name = "L2",
                // -- 2. Hardware Assignments (Constant) --
                .timer_instance = &htim4,
                .timer_channel = TIM_CHANNEL_1,
                .STEP = STEP2_Pin,
                .STEP_Port = STEP2_GPIO_Port,
                .DIR = DIR2_Pin,
                .DIR_Port = DIR2_GPIO_Port,
                .LIMIT = LIMIT2_Pin,
                .LIMIT_Port = LIMIT2_GPIO_Port,
                .limit_val = 1,
                .direction_flipped = false,


                // -- 3. Mechanical & Kinematic Configuration (Constant) --
                .reduction_ratio = g_reduction_ratios[1], // 20.0f
                .microstep = MICROSTEP,
                .steps_per_radian =
                    (MOTOR_BASE_STEPS_PER_REV * MICROSTEP / (2.0f * M_PI)) *
                    g_reduction_ratios[1],
                .radians_per_step =
                    ((2.0f * M_PI) / MOTOR_BASE_STEPS_PER_REV / MICROSTEP) /
                    g_reduction_ratios[1],
                // -- 4. Electrical Configuration (Constant) --
                .motor_max_current = MOTOR2_MAX_CURRENT,
                .irun = 22,
                .ihold = 15,
                // -- 5. Motion Profile & Dynamic Limits (Constant) --
                // High gear ratio (20:1) allows faster motor speeds
                .motor_max_speed = 8000,
                .motor_min_speed = 0,
                .motor_max_acceleration = 12000,
                .motor_min_acceleration = 0,
                .motor_max_jerk = 60000,
                .homing_speed = SPEED_CALC(200),
                // -- 6. Positional Limits & Setpoints --
                .joint_range_positive = 0.0f,
                .joint_range_negative = 0.0f,
                //.did not work had to chage joint_range_positive_steps = -1200,
                //.joint_range_negative_steps = -51587,
                .joint_range_positive_steps = +20372,
                .joint_range_negative_steps = -19965,
                .standby_position = 0,
                .homed_position = STEP_CALC(-632.125),
                .encoder_zero_step = 0,
                .encoder_channel = 6,
                // -- 7. Command Interface --
                .commanded_mode = STOPPED,
                .commanded_position = 0,
                .commanded_velocity = 0.0,
                .commanded_current = 0,
                .start_position = 0,
                // -- 8. Real-Time State (Volatile) --
                .position = 0,
                .velocity = 0.0f,
                .acceleration = 0.0f,
                .current = 0,
                .direction = 0,
                .moving = false,
                .limit_switch_trigger = false,
                .encoder_raw_position = 0,
                .arr_value = 0,
                // -- 9. Homing State Machine --
                .homed = 0,
                .homing = 0,
                .homing_stage_1 = 0,
                .homing_stage_2 = 0,
                .homing_direction = ccw,
                // -- 10. Diagnostic & Error Flags --
                .error = 0,
                .position_error = 0,
                .temperature_warning = 0,
                .temperature_error = 0,
                .over_temp_pre_warning = 0,
                .over_temp_warning = 0,
                .diag0 = 0,
                .open_load_A = 0,
                .open_load_B = 0,
                .short_2_gnd_A = 0,
                .short_2_gnd_B = 0,
            },
        [2] =
            {
                // L3
                // -- 1. Identification --
                .name = "L3",
                // -- 2. Hardware Assignments (Constant) --
                .timer_instance = &htim5,
                .timer_channel = TIM_CHANNEL_1,
                .STEP = STEP3_Pin,
                .STEP_Port = STEP3_GPIO_Port,
                .DIR = DIR3_Pin,
                .DIR_Port = DIR3_GPIO_Port,
                .LIMIT = LIMIT3_Pin,
                .LIMIT_Port = LIMIT3_GPIO_Port,

                .limit_val = 1,
				.direction_flipped=true,

                // -- 3. Mechanical & Kinematic Configuration (Constant) --
                .reduction_ratio = g_reduction_ratios[2], // 18.0952381f
                .microstep = MICROSTEP,
                .steps_per_radian =
                    (MOTOR_BASE_STEPS_PER_REV * MICROSTEP / (2.0f * M_PI)) *
                    g_reduction_ratios[2],
                .radians_per_step =
                    ((2.0f * M_PI) / MOTOR_BASE_STEPS_PER_REV / MICROSTEP) /
                    g_reduction_ratios[2],
                // -- 4. Electrical Configuration (Constant) --
                .motor_max_current = MOTOR3_MAX_CURRENT,
                .irun = 12,
                .ihold = 12,
                // -- 5. Motion Profile & Dynamic Limits (Constant) --
                // High gear ratio (18:1) - similar to L2
                .motor_max_speed = 8000,
                .motor_min_speed = 0,
                .motor_max_acceleration = 12000,
                .motor_min_acceleration = 0,
                .motor_max_jerk = 60000,
                .homing_speed = SPEED_CALC(200),
                .homing_direction = cw,
                // -- 6. Positional Limits & Setpoints --
                .joint_range_positive = 0.0f,
                .joint_range_negative = 0.0f,
                .joint_range_positive_steps = 92605,
                .joint_range_negative_steps = 34700,
                .standby_position = 0,
                .homed_position = STEP_CALC(760),
                .encoder_zero_step = 0,
                .encoder_channel = 5,
                // -- 7. Command Interface --
                .commanded_mode = STOPPED,
                .commanded_position = 0,
                .commanded_velocity = 0.0,
                .commanded_current = 0,
                .start_position = 0,
                // -- 8. Real-Time State (Volatile) --
                .position = 0,
                .velocity = 0.0f,
                .acceleration = 0.0f,
                .current = 0,
                .direction = STOPPED,
                .moving = false,
                .limit_switch_trigger = true,
                .encoder_raw_position = 0,
                .arr_value = 0,
                // -- 9. Homing State Machine --
                .homed = 0,
                .homing = 0,
                .homing_stage_1 = 0,
                .homing_stage_2 = 0,
                // -- 10. Diagnostic & Error Flags --
                .error = 0,
                .position_error = 0,
                .temperature_warning = 0,
                .temperature_error = 0,
                .over_temp_pre_warning = 0,
                .over_temp_warning = 0,
                .diag0 = 0,
                .open_load_A = 0,
                .open_load_B = 0,
                .short_2_gnd_A = 0,
                .short_2_gnd_B = 0,
            },
        [3] =
            {
                // L4
                // -- 1. Identification --
                .name = "L4",
                // -- 2. Hardware Assignments (Constant) --
                .timer_instance = &htim8,
                .timer_channel = TIM_CHANNEL_1,
                .STEP = STEP4_Pin,
                .STEP_Port = STEP4_GPIO_Port,
                .DIR = DIR4_Pin,
                .DIR_Port = DIR4_GPIO_Port,
                .LIMIT = LIMIT4_Pin,
                .LIMIT_Port = LIMIT4_GPIO_Port,
                .limit_val = 1,
                // -- 3. Mechanical & Kinematic Configuration (Constant) --
                .reduction_ratio = g_reduction_ratios[3], // 4.0f
                .microstep = MICROSTEP,
                .steps_per_radian =
                    (MOTOR_BASE_STEPS_PER_REV * MICROSTEP / (2.0f * M_PI)) *
                    g_reduction_ratios[3],
                .radians_per_step =
                    ((2.0f * M_PI) / MOTOR_BASE_STEPS_PER_REV / MICROSTEP) /
                    g_reduction_ratios[3],
                // -- 4. Electrical Configuration (Constant) --
                .motor_max_current = MOTOR4_MAX_CURRENT,
                .irun = 16,
                .ihold = 10,
                // -- 5. Motion Profile & Dynamic Limits (Constant) --
                // Lower gear ratio (4:1) - gentler parameters
                .motor_max_speed = 4000,
                .motor_min_speed = 0,
                .motor_max_acceleration = 6000,
                .motor_min_acceleration = 0,
                .motor_max_jerk = 30000,
                .homing_speed = SPEED_CALC(200),
                .homing_direction = ccw,
                // -- 6. Positional Limits & Setpoints --
                .joint_range_positive = 0.0f,
                .joint_range_negative = 0.0f,
                .joint_range_positive_steps = 7500,
                .joint_range_negative_steps = -7500,
                .standby_position = 0,
                .homed_position = STEP_CALC(-510.9375),
                .encoder_zero_step = 7,
                // -- 7. Command Interface --
                .commanded_mode = STOPPED,
                .commanded_position = 0,
                .commanded_velocity = 0.0,
                .commanded_current = 0,
                .start_position = 0,
                // -- 8. Real-Time State (Volatile) --
                .position = 0,
                .velocity = 0.0f,
                .acceleration = 0.0f,
                .current = 0,
                .direction = 1,
                .moving = false,
                .limit_switch_trigger = false,
                .encoder_raw_position = 0,
                .arr_value = 0,
                // -- 9. Homing State Machine --
                .homed = 0,
                .homing = 0,
                .homing_stage_1 = 0,
                .homing_stage_2 = 0,
                // -- 10. Diagnostic & Error Flags --
                .error = 0,
                .position_error = 0,
                .temperature_warning = 0,
                .temperature_error = 0,
                .over_temp_pre_warning = 0,
                .over_temp_warning = 0,
                .diag0 = 0,
                .open_load_A = 0,
                .open_load_B = 0,
                .short_2_gnd_A = 0,
                .short_2_gnd_B = 0,
            },
        [4] =
            {// L5
             // -- 1. Identification --
             .name = "L5",
             // -- 2. Hardware Assignments (Constant) --
             .timer_instance = &htim10,
             .timer_channel = TIM_CHANNEL_1,
             .STEP = STEP5_Pin,
             .STEP_Port = STEP5_GPIO_Port,
             .DIR = DIR5_Pin,
             .DIR_Port = DIR5_GPIO_Port,
             .LIMIT = LIMIT5_Pin,
             .LIMIT_Port = LIMIT5_GPIO_Port,
             .limit_val = 1,
             // -- 3. Mechanical & Kinematic Configuration (Constant) --
             .reduction_ratio = g_reduction_ratios[4], // 4.0f
             .microstep = MICROSTEP,
             .steps_per_radian =
                 (MOTOR_BASE_STEPS_PER_REV * MICROSTEP / (2.0f * M_PI)) *
                 g_reduction_ratios[4],
             .radians_per_step =
                 ((2.0f * M_PI) / MOTOR_BASE_STEPS_PER_REV / MICROSTEP) /
                 g_reduction_ratios[4],
             // -- 4. Electrical Configuration (Constant) --
             .motor_max_current = MOTOR5_MAX_CURRENT,
             .irun = 15,
             .ihold = 8,
             // -- 5. Motion Profile & Dynamic Limits (Constant) --
             // Lower gear ratio (4:1) - same as L4
             .motor_max_speed = 4000,
             .motor_min_speed = 0,
             .motor_max_acceleration = 6000,
             .motor_min_acceleration = 0,
             .motor_max_jerk = 30000,
             .homing_speed = SPEED_CALC(200),
             // -- 6. Positional Limits & Setpoints --
             .joint_range_positive = 0.0f,
             .joint_range_negative = 0.0f,
             .joint_range_positive_steps = 6400,
             .joint_range_negative_steps = -6400,
             .standby_position = 0,
             .homed_position = STEP_CALC(250),
             .encoder_zero_step = 0,
             .encoder_channel = 3,
             .homing_direction = cw,
             // -- 7. Command Interface --
             .commanded_mode = STOPPED,
             .commanded_position = 0,
             .commanded_velocity = 0.0,
             .commanded_current = 0,
             .start_position = 0,
             // -- 8. Real-Time State (Volatile) --
             .position = 0,
             .velocity = 0.0f,
             .acceleration = 0.0f,
             .current = 0,
             .direction = 0,
             .moving = false,
             .limit_switch_trigger = true,
             .encoder_raw_position = 0,
             .arr_value = 0,
             // -- 9. Homing State Machine --
             .homed = 0,
             .homing = 0,
             .homing_stage_1 = 0,
             .homing_stage_2 = 0,
             // -- 10. Diagnostic & Error Flags --
             .error = 0,
             .position_error = 0,
             .temperature_warning = 0,
             .temperature_error = 0,
             .over_temp_pre_warning = 0,
             .over_temp_warning = 0,
             .diag0 = 0,
             .open_load_A = 0,
             .open_load_B = 0,
             .short_2_gnd_A = 0,
             .short_2_gnd_B = 0,
             .encoder_channel = 3},
        [5] =
            {
                // L6
                // -- 1. Identification --
                .name = "L6",
                // -- 2. Hardware Assignments (Constant) --
                .timer_instance = &htim11,
                .timer_channel = TIM_CHANNEL_1,
                .STEP = STEP6_Pin,
                .STEP_Port = STEP6_GPIO_Port,
                .DIR = DIR6_Pin,
                .DIR_Port = DIR6_GPIO_Port,
                .LIMIT = LIMIT6_Pin,
                .LIMIT_Port = LIMIT6_GPIO_Port,
                .limit_val = 1,
                // -- 3. Mechanical & Kinematic Configuration (Constant) --
                .reduction_ratio = g_reduction_ratios[5], // 10.0f
                .microstep = MICROSTEP,
                .steps_per_radian =
                    (MOTOR_BASE_STEPS_PER_REV * MICROSTEP / (2.0f * M_PI)) *
                    g_reduction_ratios[5],
                .radians_per_step =
                    ((2.0f * M_PI) / MOTOR_BASE_STEPS_PER_REV / MICROSTEP) /
                    g_reduction_ratios[5],
                // -- 4. Electrical Configuration (Constant) --
                .motor_max_current = MOTOR6_MAX_CURRENT,
                .irun = 7,
                .ihold = 5,
                // -- 5. Motion Profile & Dynamic Limits (Constant) --
                // Medium gear ratio (10:1)
                .motor_max_speed = 6000,
                .motor_min_speed = 0,
                .motor_max_acceleration = 8000,
                .motor_min_acceleration = 0,
                .motor_max_jerk = 40000,
                .homing_speed = SPEED_CALC(200),
                // -- 6. Positional Limits & Setpoints --
                .joint_range_positive = 0.0f,
                .joint_range_negative = 0.0f,
                .joint_range_positive_steps = 64000,
                .joint_range_negative_steps = 0,
                .standby_position = 32000,
                .homed_position = STEP_CALC(30.109375),
                .encoder_zero_step = 0,
                .encoder_channel = 2,
                // -- 7. Command Interface --
                .commanded_mode = STOPPED,
                .commanded_position = 0,
                .commanded_velocity = 0.0,
                .commanded_current = 0,
                .start_position = 0,
                // -- 8. Real-Time State (Volatile) --
                .position = 0,
                .velocity = 0.0f,
                .acceleration = 0.0f,
                .current = 0,
                .direction = STOPPED,
                .moving = false,
                .limit_switch_trigger = false,
                .encoder_raw_position = 0,
                .arr_value = 0,
                // -- 9. Homing State Machine --
                .homed = 0,
                .homing = 0,
                .homing_stage_1 = 0,
                .homing_stage_2 = 0,
                // -- 10. Diagnostic & Error Flags --
                .error = 0,
                .position_error = 0,
                .temperature_warning = 0,
                .temperature_error = 0,
                .over_temp_pre_warning = 0,
                .over_temp_warning = 0,
                .diag0 = 0,
                .open_load_A = 0,
                .open_load_B = 0,
                .short_2_gnd_A = 0,
                .short_2_gnd_B = 0,
            },
        [6] =
            {
                // GRP (Gripper)
                // -- 1. Identification --
                .name = "left_finger_joint",
                // -- 2. Hardware Assignments (Constant) --
                .timer_instance = &htim3,
                .timer_channel = TIM_CHANNEL_1,
                .STEP = STEP1_Pin, // Note: STEP pin is shared with L6
                .STEP_Port =
                    STEP1_GPIO_Port,        // Note: STEP port is shared with L6
                .DIR = DIR7_Pin,            // Using L1's DIR pin
                .DIR_Port = DIR7_GPIO_Port, // Using L1's DIR port
                .LIMIT = LIMIT1_Pin,        // Using L1's LIMIT pin
                .LIMIT_Port = LIMIT1_GPIO_Port, // Using L1's LIMIT port
                                                // -- 3. Mechanical & Kinematic
                                                // Configuration (Constant) --
                .reduction_ratio = g_reduction_ratios[6], // 1.0f
                .microstep = MICROSTEP,
                .steps_per_radian =
                    (MOTOR_BASE_STEPS_PER_REV * MICROSTEP / (2.0f * M_PI)) *
                    g_reduction_ratios[6],
                .radians_per_step =
                    ((2.0f * M_PI) / MOTOR_BASE_STEPS_PER_REV / MICROSTEP) /
                    g_reduction_ratios[6],
                // -- 4. Electrical Configuration (Constant) --
                .motor_max_current = MOTOR6_MAX_CURRENT, // Assumed same as L6
                .irun = 12,
                .ihold = 10,
                // -- 5. Motion Profile & Dynamic Limits (Constant) --
                // Direct drive (1:1) with very small moves (~0.03 rad)
                // Use very gentle parameters for precision
                .motor_max_speed = 500,
                .motor_min_speed = 0,
                .motor_max_acceleration = 1000,
                .motor_min_acceleration = 0,
                .motor_max_jerk = 5000,
                .homing_speed = 500,
                // -- 6. Positional Limits & Setpoints --
                .joint_range_positive = 0.0f,
                .joint_range_negative = 0.0f,
                .joint_range_positive_steps = 64000,
                .joint_range_negative_steps = 0,
                .standby_position = 0,
                .homed_position = 0,
                .encoder_zero_step = 0,
                // -- 7. Command Interface --
                .commanded_mode = STOPPED,
                .commanded_position = 0,
                .commanded_velocity = 0.0,
                .commanded_current = 0,
                .start_position = 0,
                // -- 8. Real-Time State (Volatile) --
                .position = 0,
                .velocity = 0.0f,
                .acceleration = 0.0f,
                .current = 0,
                .direction = STOPPED,
                .moving = false,
                .limit_switch_trigger = false,
                .encoder_raw_position = 0,
                .arr_value = 0,
                // -- 9. Homing State Machine --
                .homed = 0,
                .homing = 0,
                .homing_stage_1 = 0,
                .homing_stage_2 = 0,
                // -- 10. Diagnostic & Error Flags --
                .error = 0,
                .position_error = 0,
                .temperature_warning = 0,
                .temperature_error = 0,
                .over_temp_pre_warning = 0,
                .over_temp_warning = 0,
                .diag0 = 0,
                .open_load_A = 0,
                .open_load_B = 0,
                .short_2_gnd_A = 0,
                .short_2_gnd_B = 0,
            },
};

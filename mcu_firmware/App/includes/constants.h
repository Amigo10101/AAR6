/*
 * constants.h
 *
 *  Created on: Aug 31, 2025
 *      Author: idgaf
 */

#ifndef INCLUDES_CONSTANTS_H_
#define INCLUDES_CONSTANTS_H_

#define VERSION 1

#define ID 1

#define ADC_RESOLUTION 12

#define R_SENSE 0.075f

#define NUMBER_OF_JOINTS 7

#define MICROSTEP 32

#define TIMER_FREQ = 1000000;

#define MOTOR1_MAX_CURRENT 2000 // Max 2100
#define MOTOR2_MAX_CURRENT 2000 // Max 2100
#define MOTOR3_MAX_CURRENT 1900 // Max 2000
#define MOTOR4_MAX_CURRENT                                                     \
  1700 // Max 2000 but since the motors are enclosed reduce it
#define MOTOR5_MAX_CURRENT                                                     \
  1700 // Max 2000 but since the motors are enclosed reduce it
#define MOTOR6_MAX_CURRENT 965 // Max 1000

#define Motor_Steps 200
#define APB1_TIMER_CLOCK_FREQ 90000000UL // 90 MHz
#define APB1_TIMER_PSC 44
/*
 * S-Curve Motion Profile
 *
 * The old 51-point lookup table has been replaced with real-time
 * recursive trajectory generation. See scurve.h for the new
 * implementation which provides:
 *   - Infinite resolution (no quantization)
 *   - Jerk-limited 7-segment profiles
 *   - Automatic triangular profiles for short moves
 *   - Per-motor configurable limits
 */
#endif /* INCLUDES_CONSTANTS_H_ */

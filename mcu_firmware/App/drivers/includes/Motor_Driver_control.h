/*
 * Motor_Driver_control.h
 *
 *  Created on: Oct 14, 2025
 *      Author: idgaf
 */

#ifndef DRIVERS_INCLUDES_MOTOR_DRIVER_CONTROL_H_
#define DRIVERS_INCLUDES_MOTOR_DRIVER_CONTROL_H_


#include <Driver_config.h>
#include "main.h" //  main project header (includes HAL, etc.)
#include <stdio.h> // For vsnprintf
#include <string.h> // For strlen
#include <stdarg.h> // For va_list
#include <stdbool.h> // For bool type
// --- All TMC5160A Register Addresses (from Datasheet) ---
#define REG_GCONF       0x00
#define REG_GSTAT       0x01
#define REG_IFCNT       0x02
#define REG_NODECONF    0x03
#define REG_IOIN        0x04
#define REG_X_COMPARE   0x05
#define REG_OTP_PROG    0x06
#define REG_OTP_READ    0x07
#define REG_FACTORY_CONF 0x08
#define REG_SHORT_CONF  0x09
#define REG_DRV_CONF    0x0A
#define REG_GLOBALSCALER 0x0B
#define REG_OFFSET_READ 0x0C
#define REG_IHOLD_IRUN  0x10
#define REG_TPOWERDOWN  0x11
#define REG_TSTEP       0x12
#define REG_TPWMTHRS    0x13
#define REG_TCOOLTHRS   0x14
#define REG_THIGH       0x15
#define REG_RAMPMODE    0x20
#define REG_XACTUAL     0x21
#define REG_VACTUAL     0x22
#define REG_VSTART      0x23
#define REG_A1          0x24
#define REG_V1          0x25
#define REG_AMAX        0x26
#define REG_VMAX        0x27
#define REG_DMAX        0x28
#define REG_D1          0x2A
#define REG_VSTOP       0x2B
#define REG_TZEROWAIT   0x2C
#define REG_XTARGET     0x2D
#define REG_VDCMIN      0x33
#define REG_SW_MODE     0x34
#define REG_RAMP_STAT   0x35
#define REG_XLATCH      0x36
#define REG_ENCMODE     0x38
#define REG_X_ENC       0x39
#define REG_ENC_CONST   0x3A
#define REG_ENC_STATUS  0x3B
#define REG_ENC_LATCH   0x3C
#define REG_ENC_DEVIATION 0x3D
#define REG_MSCNT       0x6A
#define REG_MSCURACT    0x6B
#define REG_CHOPCONF    0x6C
#define REG_COOLCONF    0x6D
#define REG_DCCTRL      0x6E
#define REG_DRV_STATUS  0x6F
#define REG_PWMCONF     0x70
#define REG_PWM_SCALE   0x71
#define REG_PWM_AUTO    0x72
#define REG_LOST_STEPS  0x73

// --- Tuning Constants ---
#define PWM_SCALE_AUTO_TUNED_THRESHOLD 5 // Threshold to consider tuning successful
#define AT2_REQUIRED_FULLSTEPS 4000 // Number of fullsteps for AT#2 movement
#define AT2_MICROSTEPS_PER_FULLSTEP 256 // Assuming MRES = 0b0000 for 256 microsteps
#define AT2_TOTAL_STEPS (AT2_REQUIRED_FULLSTEPS * AT2_MICROSTEPS_PER_FULLSTEP)
#define AT2_STEP_DELAY_US 5// Delay in microseconds for AT#2 step frequency
#define AT2_STATUS_CHECK_INTERVAL 1000 // Check tuning status every this many steps
#define AT2_TIMEOUT_MS 10000 // Timeout for the AT#2 tuning phase

// --- Public Function Prototypes ---
/**
 * @brief Writes a 40-bit datagram to the TMC5160A.
 */
void tmc5160_write(uint8_t address, uint32_t data);
/**
 * @brief Reads a 32-bit value from a TMC5160A register.
 */
uint32_t tmc5160_read( uint8_t address);
/**
 * @brief Safely DISABLES the motor driver outputs by setting TOFF=0.
 */
void tmc5160_disable_driver();
/**
 * @brief Safely ENABLES the motor driver outputs by setting TOFF > 0.
 * @param toff_value A value between 1 and 15. A good default is 3-5.
 */
void tmc5160_enable_driver(uint8_t toff_value);
/**
 * @brief Prints a human-readable interpretation of the DRV_STATUS register.
 */
void print_drv_status(uint32_t status);

/**
 * @brief The main diagnostic function. Reads and reports on the full status of the TMC5160A.
 */
void tmc5160_full_diagnostic();

/**
 * @brief Initializes the TMC5160A with the absolute minimum safe current settings.
 */
void tmc5160_minimal_current_init();


void tmc5160_init();


#endif /* DRIVERS_INCLUDES_MOTOR_DRIVER_CONTROL_H_ */

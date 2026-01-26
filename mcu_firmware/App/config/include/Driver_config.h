#ifndef DRIVER_CONFIG_H
#define DRIVER_CONFIG_H
#include "constants.h"
/*
 * ============================================================================
 * FILE: driver_config.h
 * AUTHOR: IDGAF
 * DESCRIPTION: Configuration constants for the TMC5160 stepper motor driver.
 *
 * This file centralizes all user-configurable parameters for the motor's
 * operational state (applied *after* tuning is complete).
 * ============================================================================
 */

// --- Final Motor Configuration Constants ---

// --- Motor Configuration Constants as Arrays ---
// (Based on your provided values, repeated for 7 joints)
#define FINAL_TPWMTHRS 2000



// Overall current scaling. 128 = 50% of the maximum current set by the sense resistors.

#define FINAL_GLOBAL_SCALER 128



// Motor run current (0-31). 20/31 is ~65% of the scaled current.

#define FINAL_IRUN 12



// Motor hold current (0-31) when stationary. 10/31 is ~32%.

#define FINAL_IHOLD 2



// Delay before current reduces to IHOLD. 0 = instant.

#define FINAL_IHOLD_DELAY 0



// SpreadCycle chopper off time. A value of 3-5 is typical.

#define FINAL_CHOPCONF_TOFF 4



// SpreadCycle hysteresis start.

#define FINAL_CHOPCONF_HSTRT 3



// SpreadCycle hysteresis end.

#define FINAL_CHOPCONF_HEND 0



// Chopper blanking time. 1=24 clocks, 2=36 clocks. A setting of 1 is good for most motors.

#define FINAL_CHOPCONF_TBL 2

#endif // DRIVER_CONFIG_H

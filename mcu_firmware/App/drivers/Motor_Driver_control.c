/*
 * Driver.c
 *
 *  Created on: Sep 1, 2025
 *      Author: idgaf
 */
#define LOGGER_NAME "MotorDriver_logger"

#include "tim.h"
#include "spi.h"
#include "gpio.h"
#include <Motor_Driver_control.h>
#include "constants.h"
#include "structs.h"
#include "Motor_control.h"
extern struct MotorStruct motors[NUMBER_OF_JOINTS];
extern struct Robot robot;
#define HOMING_POLL_DELAY_MS 5  // Poll status every 5ms
#define HOMING_TIMEOUT_MS 10000 // 10-second timeout for a homing move
#define HOMING_TIMEOUT_CYCLES (HOMING_TIMEOUT_MS / HOMING_POLL_DELAY_MS)
#define HOMING_VALID_SG_THRESHOLD 50  // We must see a reading ABOVE this before stall detection is armed.
#define HOMING_GRACE_PERIOD_MS 500    // Max time to wait for a valid SG reading (motor to get up to speed).
#define HOMING_GRACE_PERIOD_CYCLES (HOMING_GRACE_PERIOD_MS / HOMING_POLL_DELAY_MS)
#define HOMING_SENSITIVITY  -5
// ============================================================================
// --- Register Control FUNCTIONS ---
// ============================================================================
void configure_global_scaler(uint8_t scaler) {
    if (scaler < 32 && scaler != 0) {
        scaler = 32;    }
    tmc5160_write(REG_GLOBALSCALER, (uint32_t)scaler);
}

void configure_motor_current(uint8_t index , int8_t irun, uint8_t ihold, uint8_t ihold_delay) {
	uint32_t value = tmc5160_read(REG_IHOLD_IRUN);
    value &= 0xFFF00000;
	value |= ((uint32_t)(ihold_delay & 0x0F)) << 16;
	value |= ((uint32_t)(irun & 0x1F)) << 8;
	value |= (uint32_t)(ihold & 0x1F);
	tmc_5160_write_single(index,REG_IHOLD_IRUN, value);
}

/**
 * @brief Configures CHOPCONF for high-performance STEP/DIR operation.
 * @note This sets MRES to 256 and enables interpolation (intpol=1).
 */
void configure_chopconf(uint16_t microsteps, uint8_t toff, uint8_t hstrt, uint8_t hend, uint8_t tbl) {
    uint32_t chopconf_val = tmc5160_read(REG_CHOPCONF);


    uint8_t mres = 0;
    switch(microsteps) {
        case 256: mres = 0; break; // %0000: Native 256
        case 128: mres = 1; break; // %0001: 128
        case  64: mres = 2; break; // %0010: 64
        case  32: mres = 3; break; // %0011: 32
        case  16: mres = 4; break; // %0100: 16
        case   8: mres = 5; break; // %0101: 8
        case   4: mres = 6; break; // %0110: 4
        case   2: mres = 7; break; // %0111: Halfstep
        case   1: mres = 8; break; // %1000: Fullstep
        default:  mres = 0; break; // Default to 256 if invalid
    }

    chopconf_val &= 0x00F80000;
    chopconf_val |= (1UL << 28);

    // mres (Bits 27..24)
    chopconf_val |= ((uint32_t)mres << 24);

    // tbl (Bits 16..15): Blank time
    chopconf_val |= ((uint32_t)(tbl & 0x03) << 15);


    // hend (Bits 10..7): Hysteresis Low Value
    chopconf_val |= ((uint32_t)(hend & 0x0F) << 7);

    // hstrt (Bits 6..4): Hysteresis Start Value
    chopconf_val |= ((uint32_t)(hstrt & 0x07) << 4);

    // toff (Bits 3..0): Off Time
    chopconf_val |= (uint32_t)(toff & 0x0F);

    tmc5160_write(REG_CHOPCONF, chopconf_val);
}


// ============================================================================
// --- COMMUNICATION & UTILITY FUNCTIONS ---
// ============================================================================

static void tmc5160_spi_write_daisy(uint8_t *p_tx_data, uint8_t *p_rx_data) {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    // Transmit and receive the entire chain's worth of data
    HAL_SPI_TransmitReceive(&hspi1, p_tx_data, p_rx_data, 5 * NUMBER_OF_JOINTS, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

}
void tmc5160_write(uint8_t address, uint32_t data) {
	uint8_t tx_data[5 * NUMBER_OF_JOINTS];
	    uint8_t rx_dummy[5 * NUMBER_OF_JOINTS];

	    // 1. Prepare identical packets for all motors
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
	        int offset = i * 5;
	        tx_data[offset + 0] = address | 0x80;
	        tx_data[offset + 1] = (data >> 24) & 0xFF;
	        tx_data[offset + 2] = (data >> 16) & 0xFF;
	        tx_data[offset + 3] = (data >> 8) & 0xFF;
	        tx_data[offset + 4] = data & 0xFF;
	    }

	    // 2. Send the full daisy-chain packet
	    tmc5160_spi_write_daisy(tx_data, rx_dummy);
}
void tmc_5160_write_single(uint8_t index, int8_t address, uint32_t data) {
    uint8_t tx_data[5 * NUMBER_OF_JOINTS] = {0};
    uint8_t rx_dummy[5 * NUMBER_OF_JOINTS] = {0};

    // In daisy chain: first data goes to last motor
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        int offset = i * 5;
        int motor_pos = NUMBER_OF_JOINTS - 1 - i; // Reverse order
        if (motor_pos == index) {
            tx_data[offset + 0] = address | 0x80;
            tx_data[offset + 1] = (data >> 24) & 0xFF;
            tx_data[offset + 2] = (data >> 16) & 0xFF;
            tx_data[offset + 3] = (data >> 8) & 0xFF;
            tx_data[offset + 4] = data & 0xFF;
        }
        // Other motors remain 0x00
    }

    tmc5160_spi_write_daisy(tx_data, rx_dummy);
}


/**
 * @brief Reads a register from all daisy-chained TMC5160s following datasheet pipelined read.
 * @param address Register address to read (MSB=0 for read).
 * @param results Pointer to an array of size NUMBER_OF_JOINTS to store the values.
 */
void tmc5160_read_all(uint8_t address, uint32_t *results)
{
    uint8_t tx_data[5 * NUMBER_OF_JOINTS] = {0};
    uint8_t rx_data[5 * NUMBER_OF_JOINTS] = {0};

    // --- Step 1: Send read command to all devices (first frame) ---
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        int offset = i * 5;
        tx_data[offset + 0] = address & 0x7F; // MSB=0 → read
        tx_data[offset + 1] = 0x00;
        tx_data[offset + 2] = 0x00;
        tx_data[offset + 3] = 0x00;
        tx_data[offset + 4] = 0x00;
    }

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, sizeof(tx_data), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    osDelay(1); // small delay to allow pipeline

    // --- Step 2: Send the same read command again (second frame) ---
    memset(tx_data, 0, sizeof(tx_data));
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        int offset = i * 5;
        tx_data[offset + 0] = address & 0x7F; // read again to clock out previous data
        tx_data[offset + 1] = 0x00;
        tx_data[offset + 2] = 0x00;
        tx_data[offset + 3] = 0x00;
        tx_data[offset + 4] = 0x00;
    }

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, sizeof(tx_data), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    // --- Step 3: Extract 32-bit register values from each 5-byte frame ---
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        int offset = i * 5;
        results[i] =
            ((uint32_t)rx_data[offset + 1] << 24) |
            ((uint32_t)rx_data[offset + 2] << 16) |
            ((uint32_t)rx_data[offset + 3] << 8)  |
            ((uint32_t)rx_data[offset + 4]);
    }
}


uint32_t tmc5160_read(uint8_t address) {
    uint8_t tx_dummy[5] = {address & 0x7F, 0, 0, 0, 0};
    uint8_t rx_data[5] = {0};
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_dummy, rx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_dummy, rx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
    return ((uint32_t)rx_data[1] << 24) | ((uint32_t)rx_data[2] << 16) | ((uint32_t)rx_data[3] << 8) | rx_data[4];
}


// ============================================================================
// --- aNALYSIS AND CONTROL FUNCTIONS ---
// ============================================================================
void tmc5160_clear_status_flags() {
    tmc5160_write(REG_GSTAT, 0x07);
    HAL_Delay(1);
}

void tmc5160_disable_driver() {
    uint32_t chopconf_val = tmc5160_read(REG_CHOPCONF);
    chopconf_val &= 0xFFFFFFF0;
    tmc5160_write(REG_CHOPCONF, chopconf_val);
}

void tmc5160_enable_driver(uint8_t toff_value) {
    if (toff_value == 0 || toff_value > 15) {
        toff_value = 4;
    }
    uint32_t chopconf_val = tmc5160_read(REG_CHOPCONF);
    chopconf_val &= 0xFFFFFFF0;
    chopconf_val |= toff_value;
    tmc5160_write(REG_CHOPCONF, chopconf_val);
}

void tmc5160_enable_driver_single(int index,uint8_t toff_value ){
	   if (toff_value == 0 || toff_value > 15) {
	        toff_value = 4;
	    }
	    uint32_t chopconf_val = tmc5160_read(REG_CHOPCONF);
	    chopconf_val &= 0xFFFFFFF0;
	    chopconf_val |= toff_value;
	    tmc_5160_write_single(index, REG_CHOPCONF, chopconf_val);

}

void tmc5160_analyze_shutdown_cause() {
	    uart_printf("\n--- MOTOR STOPPED: Analyzing Shutdown Cause ---\n");

	    // Read the primary status registers
	    uint32_t gstat = tmc5160_read(REG_GSTAT);
	    uint32_t drv_status = tmc5160_read(REG_DRV_STATUS);
	    uint32_t ramp_stat = tmc5160_read(REG_RAMP_STAT);
	    uint32_t sw_mode = tmc5160_read(REG_SW_MODE);

	    uart_printf("GSTAT: 0x%02lX, DRV_STATUS: 0x%08lX\n\n", gstat, drv_status);

	    int fault_found = 0;

	    // Check for the most critical faults first
	    if (drv_status & (1 << 25)) { // Overtemperature Shutdown (ot)
	        uart_printf(">>> CAUSE IDENTIFIED: Overtemperature Shutdown (ot)\n");
	        uart_printf("    MEANING: The driver chip exceeded its maximum temperature and shut down to protect itself.\n");
	        uart_printf("    HOW TO FIX:\n");
	        uart_printf("    1. Improve cooling: Add a heatsink and/or a fan to the driver chip.\n");
	        uart_printf("    2. Reduce motor current: Lower the IRUN setting in the IHOLD_IRUN register.\n");
	        fault_found = 1;
	    }
	    if (drv_status & ( (1 << 28) | (1 << 27) | (1 << 13) | (1 << 12) )) { // Any Short Circuit flag
	        uart_printf(">>> CAUSE IDENTIFIED: Short Circuit Protection\n");
	        uart_printf("    MEANING: The driver detected a short circuit on the motor outputs (s2g, s2vs) and shut down immediately.\n");
	        uart_printf("    HOW TO FIX:\n");
	        uart_printf("    1. POWER DOWN IMMEDIATELY.\n");
	        uart_printf("    2. Inspect all motor wiring and connectors for loose strands or bad connections.\n");
	        uart_printf("    3. A very noisy chopper (e.g., Fullstep mode with short TBL) can cause false triggers. Use a smoother microstep setting.\n");
	        fault_found = 1;
	    }
	    if (gstat & (1 << 2)) { // Undervoltage on Charge Pump (uv_cp)
	        uart_printf(">>> CAUSE IDENTIFIED: Charge Pump Undervoltage (uv_cp)\n");
	        uart_printf("    MEANING: The internal voltage required for the MOSFETs failed. This is a critical power fault.\n");
	        uart_printf("    HOW TO FIX:\n");
	        uart_printf("    1. Ensure your main power supply (VM) is stable and >= 12V.\n");
	        uart_printf("    2. Ensure you have a large bulk capacitor (>= 100uF) right at the driver's VM and GND pins.\n");
	        fault_found = 1;
	    }

	    // Check for a StallGuard-triggered stop
	    if ((ramp_stat & (1 << 6)) || (drv_status & (1 << 24))) { // event_stop_sg OR StallGuard flag in DRV_STATUS
	         if(sw_mode & (1 << 10)) { // Check if stop_on_stall is actually enabled
	            uart_printf(">>> CAUSE IDENTIFIED: StallGuard Stop Triggered (sg_stop)\n");
	            uart_printf("    MEANING: The driver detected a motor stall (high load) and executed a controlled stop as configured.\n");
	            uart_printf("    HOW TO FIX:\n");
	            uart_printf("    1. This is expected behavior if the motor was mechanically blocked.\n");
	            uart_printf("    2. If it was a false trigger, the SGT value in COOLCONF may be too sensitive. Try increasing it.\n");
	            uart_printf("    3. The motor may not have enough torque at high speed. Increase the IRUN current setting.\n");
	            fault_found = 1;
	         }
	    }

	    if (!fault_found) {
	        uart_printf(">>> No specific fault flags found. Analyzing other possibilities...\n");
	        if ((drv_status & (1 << 29)) || (drv_status & (1 << 30))) { // Open Load
	            uart_printf("    - SYMPTOM: Open Load (ola/olb) was detected.\n");
	            uart_printf("      While this doesn't cause a shutdown, it means the motor may have briefly lost connection, which can lead to a stall.\n");
	            uart_printf("      FIX: Check motor connector and wiring for intermittent connections.\n");
	        } else {
	            uart_printf("    - The motor may have lost steps due to mechanical overload without triggering StallGuard.\n");
	            uart_printf("      This happens if the load exceeds the motor's torque curve at the current speed.\n");
	            uart_printf("      FIX: Try increasing the motor current (IRUN) or reducing the maximum speed (VMAX) and acceleration (AMAX).\n");
	        }
	    }

	    uart_printf("\n--- Analysis Complete ---\n");
	}

void tmc5160_full_diagnostic() {
    uart_printf("\n\n--- Starting Full TMC5160A Diagnostic ---\n");
    HAL_Delay(500);

    uint32_t reg_vals[NUMBER_OF_JOINTS];

    // --- 1. IOIN & Version Check ---
    tmc5160_read_all(REG_IOIN, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        uint8_t version = (reg_vals[i] >> 24) & 0xFF;
        uart_printf("Motor %d IOIN (0x04): 0x%08lX, Version: 0x%02X\n", i, reg_vals[i], version);
        if (version != 0x30) {
            uart_printf("  !!! CRITICAL FAILURE: Cannot communicate with motor driver %d. Halting diagnostics. !!!\n", i);

        }
        print_IOIN_details(reg_vals[i]);
    }

    // --- 2. GSTAT ---
    tmc5160_read_all(REG_GSTAT, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        uart_printf("Motor %d GSTAT (0x01): 0x%02lX", i, reg_vals[i] & 0x07);
        if (reg_vals[i] & 1) uart_printf(" [Reset]");
        if (reg_vals[i] & 2) uart_printf(" [Driver Error]");
        if (reg_vals[i] & 4) uart_printf(" [Charge Pump Undervoltage]");
        uart_printf("\n");
    }
    osDelay(5);
    // --- 3. GCONF ---
    tmc5160_read_all(REG_GCONF, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        print_GCONF_details(reg_vals[i]);
        osDelay(2);
    }
    osDelay(10);
    // --- 4. GLOBALSCALER ---
    tmc5160_read_all(REG_GLOBALSCALER, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        uart_printf("Motor %d GLOBALSCALER(0x0B): %lu (Overall current scale: %.1f%%)\n",
                    i, reg_vals[i] & 0xFF, (float)(reg_vals[i] & 0xFF) * 100.0f / 256.0f);
    }
    osDelay(5);
    // --- 5. IHOLD_IRUN ---
    tmc5160_read_all(REG_IHOLD_IRUN, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        uart_printf("Motor %d IHOLD_IRUN (0x10): 0x%08lX (IRUN=%lu/31, IHOLD=%lu/31, DELAY=%lu)\n",
                    i, reg_vals[i], (reg_vals[i] >> 8) & 0x1F, reg_vals[i] & 0x1F, (reg_vals[i] >> 16) & 0xF);
    }
    osDelay(5);
    // --- 6. CHOPCONF ---
    tmc5160_read_all(REG_CHOPCONF, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        print_CHOPCONF_details(reg_vals[i]);
        osDelay(2);
    }
    osDelay(10);
    // --- 7. COOLCONF ---
    tmc5160_read_all(REG_COOLCONF, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        uart_printf("Motor %d COOLCONF (0x6D): 0x%08lX\n", i, reg_vals[i]);
    }
    osDelay(5);
    // --- 8. PWMCONF ---
    tmc5160_read_all(REG_PWMCONF, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        uart_printf("Motor %d PWMCONF (0x70): 0x%08lX\n", i, reg_vals[i]);
    }
    osDelay(5);
    // --- 9. PWM_SCALE ---
    tmc5160_read_all(REG_PWM_SCALE, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        uart_printf("Motor %d PWM_SCALE (0x71): SUM=%lu, AUTO=%d\n",
                    i, reg_vals[i] & 0xFF, (int8_t)((reg_vals[i] >> 16) & 0xFF));
    }
    osDelay(5);
    // --- 10. DRV_STATUS ---
    tmc5160_read_all(REG_DRV_STATUS, reg_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        print_DRV_STATUS_details(reg_vals[i]);
        osDelay(2);
    }

    uart_printf("\n--- Diagnostic Complete ---\n");
}

// --- Detailed Register Parsing Helper Functions ---

void print_IOIN_details(uint32_t reg_val) {
    uint8_t version = (reg_val >> 24) & 0xFF;
    uart_printf("IOIN (0x04): 0x%08lX (Version: 0x%02X)\n", reg_val, version);
    uart_printf("  - REFL_STEP (bit 0)      : %s\n", (reg_val & (1 << 0)) ? "HIGH" : "LOW");
    uart_printf("  - REFR_DIR (bit 1)       : %s\n", (reg_val & (1 << 1)) ? "HIGH" : "LOW");
    uart_printf("  - ENCB_DCEN_CFG4 (bit 2) : %s\n", (reg_val & (1 << 2)) ? "HIGH" : "LOW");
    uart_printf("  - ENCA_DCIN_CFG5 (bit 3) : %s\n", (reg_val & (1 << 3)) ? "HIGH" : "LOW");
    uart_printf("  - DRV_ENN (bit 4)        : %s\n", (reg_val & (1 << 4)) ? "HIGH (Driver Hardware Disabled)" : "LOW (Driver Hardware Enabled)");
    uart_printf("  - ENCN_DCO_CFG6 (bit 5)  : %s\n", (reg_val & (1 << 5)) ? "HIGH" : "LOW");
    uart_printf("  - SD_MODE (bit 6)        : %s\n", (reg_val & (1 << 6)) ? "HIGH (Step/Dir Mode)" : "LOW (Ramp Gen Mode)");
    uart_printf("  - SWCOMP_IN (bit 7)      : %s\n", (reg_val & (1 << 7)) ? "HIGH" : "LOW");
}

void print_GCONF_details(uint32_t reg_val) {
    uart_printf("GCONF (0x00): 0x%08lX\n", reg_val);
    uart_printf("  - recalibrate (bit 0): %s\n", (reg_val & (1<<0)) ? "ON" : "OFF");
    uart_printf("  - faststandstill (bit 1): %s\n", (reg_val & (1<<1)) ? "Short (2^18 clocks)" : "Normal (2^20 clocks)");
    uart_printf("  - en_pwm_mode (bit 2): %s\n", (reg_val & (1<<2)) ? "StealthChop ENABLED" : "StealthChop DISABLED");
    uart_printf("  - multistep_filt (bit 3): %s\n", (reg_val & (1<<3)) ? "Filtering ENABLED" : "Filtering DISABLED");
    uart_printf("  - shaft (bit 4): %s\n", (reg_val & (1<<4)) ? "Inverse motor direction" : "Normal motor direction");
}

void print_CHOPCONF_details(uint32_t reg_val) {
    const char* mres_tbl[] = {"256", "128", "64", "32", "16", "8", "4", "2", "Fullstep"};
    uint8_t mres_idx = (reg_val >> 24) & 0x0F;
    uart_printf("CHOPCONF (0x6C): 0x%08lX\n", reg_val);
    uart_printf("  - TOFF (bits 0-3): %lu (%s)\n", reg_val & 0xF, (reg_val & 0xF) == 0 ? "Driver DISABLED" : "Driver ENABLED");
    uart_printf("  - HSTRT (bits 4-6): %lu\n", (reg_val >> 4) & 0x7);
    uart_printf("  - HEND (bits 7-10): %lu (Effective Hysteresis End: %ld)\n", (reg_val >> 7) & 0xF, ((int8_t)((reg_val >> 7) & 0xF)) - 3);
    uart_printf("  - TBL (bits 15-16): %lu (%s)\n", (reg_val >> 15) & 0x3, ((reg_val >> 15) & 0x3) == 0 ? "16 clocks" : ((reg_val >> 15) & 0x3) == 1 ? "24 clocks" : ((reg_val >> 15) & 0x3) == 2 ? "36 clocks" : "54 clocks");
    uart_printf("  - chm (bit 14): %s\n", (reg_val & (1<<14)) ? "Constant Off-Time" : "SpreadCycle");
    uart_printf("  - MRES (bits 24-27): %s Microsteps\n", mres_tbl[mres_idx > 8 ? 8 : mres_idx]);
    uart_printf("  - intpol (bit 28): %s\n", (reg_val & (1<<28)) ? "Interpolation to 256 ENABLED" : "Interpolation DISABLED");
}

void print_DRV_STATUS_details(uint32_t reg_val) {
    uart_printf("DRV_STATUS (0x6F): 0x%08lX\n", reg_val);
    uart_printf("  - StallGuard Result (SG_RESULT): %lu\n", reg_val & 0x3FF);
    uart_printf("  - Short to Supply A/B (s2vsa/b): %s / %s\n", (reg_val & (1<<12)) ? "YES" : "no", (reg_val & (1<<13)) ? "YES" : "no");
    uart_printf("  - StealthChop mode: %s\n", (reg_val & (1<<14)) ? "YES" : "no");
    uart_printf("  - Fullstep active: %s\n", (reg_val & (1<<15)) ? "YES" : "no");
    uart_printf("  - Actual Current (CS_ACTUAL): %lu/31\n", (reg_val >> 16) & 0x1F);
    uart_printf("  - Stall Detected (StallGuard flag): %s\n", (reg_val & (1<<24)) ? "YES" : "no");
    uart_printf("  - Overtemperature Shutdown (ot): %s\n", (reg_val & (1<<25)) ? "YES" : "no");
    uart_printf("  - Overtemp Pre-warning (otpw): %s\n", (reg_val & (1<<26)) ? "YES" : "no");
    uart_printf("  - Short to Gnd A/B (s2ga/b): %s / %s\n", (reg_val & (1<<27)) ? "YES" : "no", (reg_val & (1<<28)) ? "YES" : "no");
    uart_printf("  - Open Load A/B (ola/b): %s / %s\n", (reg_val & (1<<29)) ? "YES" : "no", (reg_val & (1<<30)) ? "YES" : "no");
    uart_printf("  - Standstill (stst): %s\n", (reg_val & (1UL << 31)) ? "YES" : "no");
}



// ============================================================================
// --- STEALTHCHOP TUNING FUNCTIONS ---
// ============================================================================

void stealthchop_tune_at1() {

    tmc5160_write(REG_GCONF, 0x00000004);
    tmc5160_write(REG_TPWMTHRS, 0);
    configure_global_scaler(FINAL_GLOBAL_SCALER);
	configure_chopconf(MICROSTEP,FINAL_CHOPCONF_TOFF,FINAL_CHOPCONF_HSTRT,FINAL_CHOPCONF_HEND,FINAL_CHOPCONF_TBL);
	tmc5160_write(REG_TPOWERDOWN, 20);
	for (int i=0;i< NUMBER_OF_JOINTS;i++){
	    configure_motor_current(i,motors[i].irun, motors[i].irun, 0);
	}
    tmc5160_write(REG_PWMCONF, 0xC40C001E); // reset status
    osDelay(500);
	for (int i=0;i< NUMBER_OF_JOINTS;i++){
	    configure_motor_current(i,motors[i].irun, motors[i].ihold, 0);
	}

}

void stealthchop_start_at2_tuning() {
	uart_printf("AT2 started");
}

int8_t stealthchop_check_at2_status(uint32_t current_step) {
    uint32_t pwm_scale_reg = tmc5160_read(REG_PWM_SCALE);
    int16_t val = (pwm_scale_reg >> 16) & 0x1FF;
    if (val >= 256) {
        val -= 512;
    }

    // Print the value inside the function
    uart_printf("  - Check %lu, PWM_SCALE_AUTO: %d\n", current_step, val);

    if (val > -PWM_SCALE_AUTO_TUNED_THRESHOLD && val < PWM_SCALE_AUTO_TUNED_THRESHOLD) {
        return 0; // Success
    } else {
        return 1; // In progress
    }
}

void stealthchop_finalize_tuning() {
    uint32_t pwm_auto = tmc5160_read(REG_PWM_AUTO);
    uint8_t pwm_ofs_auto = pwm_auto & 0xFF;
    uint8_t pwm_grad_auto = (pwm_auto >> 16) & 0xFF;
    uart_printf("PWM AutoTune Values (Hex) -> OFS_AUTO: %#x, GRAD_AUTO: %#x\n", pwm_ofs_auto, pwm_grad_auto);
}

// ============================================================================
// --- Driver Init FUNCTIONS ---
// ============================================================================



int8_t tmc5160_run_tuning_process() {
    uart_printf("Starting StealthChop tuning process...\n");


    uart_printf("Performing AT#1 (standstill tuning)...\n");
    stealthchop_tune_at1(FINAL_IRUN);

    uart_printf("Moving motor for AT#2. Monitoring status...\n");

   Home_Motors();
    int8_t tuning_status = 1;
    const int max_checks = 10; // 10,000ms / 500ms = 20 checks
    for (int i = 0; i < max_checks; i++) {
           osDelay(200); // Wait for the check interval
          if (i==4) {stealthchop_start_at2_tuning();};
           tuning_status = stealthchop_check_at2_status(i);

           // If the function returns 0, tuning is successful
           if (tuning_status == 0) {
               uart_printf(" AT#2 tuning successful!\n");
               break;
                // Exit the loop early
           }
       }
    if (tuning_status != 0) {
        uart_printf("Warning: AT#2 movement finished, but tuning did not confirm success.\n");
    }

    uart_printf("Finalizing tuning and storing results...\n");
    stealthchop_finalize_tuning();
    uart_printf("Tuning process complete.\n");
    return 0;
}

void tmc5160_final_setup() {


	tmc5160_write(REG_TPWMTHRS, FINAL_TPWMTHRS);

	configure_global_scaler(FINAL_GLOBAL_SCALER); // Set scaler to minimal value

	for (int i =0;i <NUMBER_OF_JOINTS;i++){
		configure_motor_current(i,motors[i].irun, motors[i].ihold, FINAL_IHOLD_DELAY);
	}


	configure_chopconf(MICROSTEP,FINAL_CHOPCONF_TOFF,FINAL_CHOPCONF_HSTRT,FINAL_CHOPCONF_HEND,FINAL_CHOPCONF_TBL);


}

/**
 * @brief Configures a single motor for StallGuard (SpreadCycle) debugging.
 * This disables StealthChop and ensures StallGuard is active at all speeds.
 *
 * @param motor_index The motor to configure.
 * @param sgt_value The StallGuard sensitivity (-64 to +63). Try -5 to start.
 */
void setup_stallguard_for_motor(uint8_t motor_index, int8_t sgt_value)
{
    uint32_t temp_results[NUMBER_OF_JOINTS];
    uint32_t reg_val;
    char log_buf[128];

    snprintf(log_buf, sizeof(log_buf), "\n--- Configuring Motor %d for StallGuard Debug ---\n", motor_index);
    uart_printf(log_buf);

    // 1. Disable StealthChop (GCONF.en_pwm_mode = 0)
    tmc5160_read_all(REG_GCONF, temp_results);
    reg_val = temp_results[motor_index];
    reg_val &= ~(1 << 2); // Clear en_pwm_mode (bit 2)
    tmc_5160_write_single(motor_index, REG_GCONF, reg_val);
    snprintf(log_buf, sizeof(log_buf), "  - StealthChop Disabled (GCONF=0x%lX)\n", reg_val);
    uart_printf(log_buf);

    // 2. Enable SpreadCycle (CHOPCONF.chm = 0) and ensure driver is ON (TOFF > 0)
    tmc5160_read_all(REG_CHOPCONF, temp_results);
    reg_val = temp_results[motor_index];
    reg_val &= ~(1 << 14); // Clear chm (bit 14) to select SpreadCycle
    if ((reg_val & 0xF) == 0) {
        reg_val |= 4; // Set TOFF=4 if it's currently 0 (driver off)
    }
    tmc_5160_write_single(motor_index, REG_CHOPCONF, reg_val);
    snprintf(log_buf, sizeof(log_buf), "  - SpreadCycle Enabled (CHOPCONF=0x%lX)\n", reg_val);
    uart_printf(log_buf);

    // 3. Set IRUN and IHOLD to the same (non-zero) value
    // StallGuard needs current. Setting IHOLD=IRUN prevents current drop at low speed.
    tmc5160_read_all(REG_IHOLD_IRUN, temp_results);
    reg_val = temp_results[motor_index];
    uint8_t current_irun = motors[motor_index].irun;
    if (current_irun == 0) current_irun = 16; // Use a default if IRUN is 0
    reg_val &= 0xFFF00000; // Clear all current/delay settings
    reg_val |= ((uint32_t)current_irun) << 8; // Set IRUN
    reg_val |= ((uint32_t)current_irun);     // Set IHOLD = IRUN
    tmc_5160_write_single(motor_index, REG_IHOLD_IRUN, reg_val);
    snprintf(log_buf, sizeof(log_buf), "  - IRUN/IHOLD set to %d (IHOLD_IRUN=0x%lX)\n", current_irun, reg_val);
    uart_printf(log_buf);

    // 4. Set TCOOLTHRS to MAX (0xFFFFF)
    // This makes StallGuard active at ALL speeds (as long as TSTEP doesn't overflow)
    // This is the simplest way to guarantee it's on.
    tmc_5160_write_single(motor_index, REG_TCOOLTHRS, 0x000FFFFF);
    uart_printf("  - StallGuard Enabled at all speeds (TCOOLTHRS set to max)\n");

    // 5. Set SGT (StallGuard sensitivity)
    tmc5160_read_all(REG_COOLCONF, temp_results);
    reg_val = temp_results[motor_index];
    reg_val &= 0xFFC0FFFF; // Clear SGT bits (16-22)
    reg_val |= ((uint32_t)(sgt_value & 0x7F)) << 16; // Set new SGT value
    tmc_5160_write_single(motor_index, REG_COOLCONF, reg_val);
    snprintf(log_buf, sizeof(log_buf), "  - SGT set to %d (COOLCONF=0x%lX)\n", sgt_value, reg_val);
    uart_printf(log_buf);

    snprintf(log_buf, sizeof(log_buf), "--- Motor %d Setup Complete ---\n", motor_index);
    uart_printf(log_buf);
}
/**
 * @brief A continuous loop that prints SG_RESULT for all motors every 100ms.
 * This is intended for debugging StallGuard.
 */
void debug_print_stallguard_continuously()
{
    char log_buf[256]; // Buffer for the full log line
    uint32_t drv_status_results[NUMBER_OF_JOINTS];

    uart_printf("\n--- STARTING CONTINUOUS STALLGUARD MONITOR (100ms interval) ---\n");
    uart_printf("--- Move a motor manually (e.g., in MotorControl) to see SG_RESULT change ---\n");

    while (1)
    {
        // 1. Read DRV_STATUS from all motors
        tmc5160_read_all(REG_DRV_STATUS, drv_status_results);

        // 2. Format the log line
        snprintf(log_buf, sizeof(log_buf), "SG_RESULTS=[");
        for (int j = 0; j < NUMBER_OF_JOINTS; j++)
        {
            uint32_t status_j = drv_status_results[j];
            uint32_t sg_j = status_j & 0x3FF; // Extract SG_RESULT
            char temp_buf[20];
            snprintf(temp_buf, sizeof(temp_buf), " M%d: %-4lu", j, sg_j);

            // Append to the main log buffer
            if (strlen(log_buf) + strlen(temp_buf) < sizeof(log_buf) - 5)
            {
                strncat(log_buf, temp_buf, sizeof(log_buf) - strlen(log_buf) - 1);
            }
        }
        strncat(log_buf, " ]\n", sizeof(log_buf) - strlen(log_buf) - 1);

        // 3. Print the formatted line
        uart_printf(log_buf);

        // 4. Wait
        osDelay(100);
    }
}

void sensorless_homing()
{
    uint32_t drv_status_results[NUMBER_OF_JOINTS];
    uint32_t motor_status; // Status for the homing motor (motor 3)
    uint32_t sg_result;    // SG_RESULT for the homing motor (motor 3)
    int stall_detected = 0;
    int stall_armed = 0; // 0 = Waiting for valid reading, 1 = Armed and monitoring for stall
    char log_buf[256];   // Increased buffer size for all motor statuses

    struct MotorStruct *m = &motors[5];
    m->homing = true;
    int8_t multiplier = 1;
    if (m->homing_direction == ccw) {
        multiplier = -1;
    }

    uart_printf("1. Clearing any old stall flags (writing to RAMP_STAT 0x35)...\n");
    tmc_5160_write_single(5, REG_RAMP_STAT, 0xFFFFFFFF);

    uart_printf("1. Starting motor movement...\n");
    Set_Motor_Velocity(m, multiplier * m->homing_speed);

    snprintf(log_buf, sizeof(log_buf), "2. Polling... Waiting for valid SG_RESULT > %d on Motor 3\n", HOMING_VALID_SG_THRESHOLD);
    uart_printf(log_buf);

    for (int i = 0; i < HOMING_TIMEOUT_CYCLES; i++)
    {
        // Read DRV_STATUS from all motors
        tmc5160_read_all(REG_DRV_STATUS, drv_status_results);

        // --- NEW: Log SG_RESULT for ALL motors ---
        snprintf(log_buf, sizeof(log_buf), "   - Poll %d: SG_RESULTS=[", i);
        for (int j = 0; j < NUMBER_OF_JOINTS; j++)
        {
            uint32_t status_j = drv_status_results[j];
            uint32_t sg_j = status_j & 0x3FF; // Extract SG_RESULT
            char temp_buf[20];
            snprintf(temp_buf, sizeof(temp_buf), " M%d: %-4lu", j, sg_j);
            // Ensure we don't overflow the buffer
            if (strlen(log_buf) + strlen(temp_buf) < sizeof(log_buf) - 5)
            {
                strncat(log_buf, temp_buf, sizeof(log_buf) - strlen(log_buf) - 1);
            }
        }
        strncat(log_buf, " ]\n", sizeof(log_buf) - strlen(log_buf) - 1);
        uart_printf(log_buf);
        // --- End of new logging ---

        // Get the status for the motor we care about (motor 3)
        motor_status = drv_status_results[3];
        sg_result = motor_status & 0x3FF;

        if (!stall_armed)
        {
            // --- STAGE 1: Wait for valid SG reading on Motor 3 ---
            if (sg_result > HOMING_VALID_SG_THRESHOLD)
            {
                stall_armed = 1;
                snprintf(log_buf, sizeof(log_buf), "     -> Motor 3 Valid reading! SG_RESULT=%lu. Stall detection armed.\n", sg_result);
                uart_printf(log_buf);
            }
            // Check for grace period failure
            if (i > HOMING_GRACE_PERIOD_CYCLES && !stall_armed)
            {
                 uart_printf("     -> FAILED: Motor 3 did not reach valid speed for StallGuard (SG_RESULT stuck at 0).\n");
                 uart_printf("     -> FIX: Increase homing speed or make SGT more sensitive (e.g., -5).\n");
                 break;
            }
        }
        else
        {
            // --- STAGE 2: Valid reading seen, now actively look for a stall on Motor 3 ---

            // We already printed the SG_RESULT for all motors, so just check the logic
            if ((motor_status & (1 << 24)) || (sg_result == 0))
            {
                snprintf(log_buf, sizeof(log_buf), "     -> STALL DETECTED on Motor 3 (Flag: %d, SG_RESULT: %lu)!\n", (motor_status & (1 << 24)) >> 24, sg_result);
                uart_printf(log_buf);
                stall_detected = 1;
                Stop_Motor(&motors[5]);
                break; // Exit loop
            }
        }

        osDelay(HOMING_POLL_DELAY_MS);
    }

    // If loop finished without detecting a stall (timeout)
    if (!stall_detected)
    {
        uart_printf("   - Homing FAILED (Timeout or Grace Period Exceeded).\n");
        Stop_Motor(&motors[3]);
    }

    // Homing is complete (either by stall or timeout)
    m->homing = false;
    }
/**
 * @brief  Reads, Clears, and Verifies the GSTAT register.
 * Critical for recovering from "drv_err" or "uv_cp" latched shutdowns.
 */
void tmc5160_reset_gstat() {


    uint32_t gstat_vals[NUMBER_OF_JOINTS];


    uart_printf("\n--- GSTAT RESET ROUTINE ---\n");

    // 1. Read Initial State (to see why it might have failed)
    tmc5160_read_all(REG_GSTAT, gstat_vals);
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        uint8_t flags = gstat_vals[i] & 0x07;
        if (flags) {
            uart_printf("Motor %d PRE-CLEAR Flags: 0x%02X ", i, flags);
            if (flags & 0x01) uart_printf("[Reset] ");
            if (flags & 0x02) uart_printf("[DriverError] ");
            if (flags & 0x04) uart_printf("[UV_CP] ");
            uart_printf("\n");
        }
    }

    // 2. Clear Flags
    // We write 0x07 (binary 111) to clear bits 0 (reset), 1 (drv_err), and 2 (uv_cp).
    // In TMC chips, writing a '1' to a status bit clears it.
    tmc5160_write(REG_GSTAT, 0x07);

    // 3. Wait for Charge Pump / Logic to stabilize
    HAL_Delay(10);

    // 4. Verify they are gone
    tmc5160_read_all(REG_GSTAT, gstat_vals);
    int all_clear = 1;
    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        if (gstat_vals[i] & 0x07) {
            uart_printf("!!! WARNING: Motor %d GSTAT Failed to clear! Stuck at: 0x%02lX\n", i, gstat_vals[i] & 0x07);
            all_clear = 0;
        }
    }

    if (all_clear) {
        uart_printf(">>> All GSTAT flags cleared successfully.\n");
    }
    uart_printf("---------------------------\n");
}
void tmc5160_init()

{	robot.state=ROBOT_STATE_HOMING;
	tmc5160_reset_gstat();
	tmc5160_run_tuning_process();
	tmc5160_final_setup();
	tmc5160_full_diagnostic();
	robot.state=ROBOT_STATE_STANDSTILL;
	//setup_stallguard_for_motor(2, 60);
	//struct MotorStruct *m = &motors[2];
	//int8_t multiplier = (m->homing_direction == ccw) ? -1 : 1;
	//Set_Motor_Velocity(m, multiplier * m->homing_speed);
	//debug_print_stallguard_continuously();
}


#include "sensorsread_task.h"
#include "main.h"
#include "i2c.h"
#include "uart_task.h"
#include "cmsis_os2.h"
#include <stdio.h> // For uart_printf
#include "Motor_control.h"
#include "constants.h"
// I2C Addresses (7-bit address shifted left by 1 for HAL)
#define MUX_ADDR (0x70 << 1)
#define AS5600_ADDR (0x36 << 1)

// AS5600 Register for Raw Angle
#define AS5600_ANGLE_REGISTER 0x0E
#define AS5600_STATUS_REGISTER 0x0B

// AS5600 Status Register Bits
#define STATUS_BIT_MH (1 << 5) // Magnet too strong
#define STATUS_BIT_ML (1 << 4) // Magnet too weak
#define STATUS_BIT_MD (1 << 3) // Magnet detected

// Timeout for I2C operations
#define I2C_TIMEOUT_MS 100

extern osSemaphoreId_t uartInitSemaphore;
extern struct MotorStruct motors[NUMBER_OF_JOINTS];
// --- DMA Buffers and Semaphore ---

// These MUST be global or static for DMA to work safely.
static uint8_t g_i2c_mux_data = 0;       // Buffer for MUX channel selection
static uint8_t g_i2c_tx_reg_addr = 0;    // Buffer for sending register addresses
static uint8_t g_i2c_rx_data[2] = {0};   // Buffer for receiving I2C data

// RTOS Semaphore to signal DMA completion
// This must be created before the task starts, e.g., in main() or osKernelInitialize()
extern osSemaphoreId_t i2cDmaSemaphore;

// This variable will hold the error code from the error callback
volatile HAL_StatusTypeDef g_i2c_error_status = HAL_OK;



/**
  * @brief  Selects a specific channel on the PCA9548A I2C multiplexer.
  * @note   This function is NOW non-blocking. It only STARTS the transfer.
  * You must wait for the i2cDmaSemaphore after calling it.
  * @param  channel: The channel to select (0-7).
  * @retval HAL_StatusTypeDef: HAL status of starting the I2C transmission.
  */
HAL_StatusTypeDef selectI2CChannel(uint8_t channel) {
    if (channel > 7) {
        return HAL_ERROR; // Invalid channel
    }

    // Prepare the data in our persistent global buffer
    g_i2c_mux_data = 1 << channel;

    // Clear any previous error
    g_i2c_error_status = HAL_OK;

    // Start the DMA transfer using the global buffer
    return HAL_I2C_Master_Transmit_DMA(&hi2c3, MUX_ADDR, &g_i2c_mux_data, 1);
}

/**
  * @brief  Helper function to wait for I2C DMA completion via RTOS semaphore.
  * @retval HAL_StatusTypeDef: HAL_OK, HAL_TIMEOUT, or HAL_ERROR.
  */
static HAL_StatusTypeDef waitForI2C(void) {
    // Wait for the semaphore to be released by an interrupt
    osStatus_t os_status = osSemaphoreAcquire(i2cDmaSemaphore, I2C_TIMEOUT_MS);

    if (os_status == osErrorTimeout) {
        // Abort the I2C transfer on timeout
       // HAL_I2C_Abort_DMA(&hi2c3, MUX_ADDR); // Use any valid address
        return HAL_TIMEOUT;
    }

    // Check if the error callback set a specific error
    if (g_i2c_error_status != HAL_OK) {
        return g_i2c_error_status;
    }

    return HAL_OK;
}
HAL_StatusTypeDef detectI2CMultiplexer(void) {
    // Attempt a zero-length transmit to see if the MUX responds
    g_i2c_error_status = HAL_OK;
    uint8_t dummy = 0; // required by HAL but not actually sent
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit_DMA(&hi2c3, MUX_ADDR, &dummy, 0);
    if (status != HAL_OK) return status;

    // Wait for DMA completion or error
    status = waitForI2C();
    return status;
}


void StartSensorTask(void *argument)
{
    // Wait for ROS to be initialized (as per your original code)
    osSemaphoreAcquire(uartInitSemaphore, osWaitForever);
//    for (int i = 0; i < 9; i++)
//      {
//          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
//          HAL_Delay(1);  // 1 ms high
//          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
//          HAL_Delay(1);  // 1 ms low
//      }
//    HAL_I2C_DeInit(&hi2c3);
//    HAL_I2C_Init(&hi2c3);
//    osDelay(50);  // small delay before detectI2CMultiplexer()
//
    while (detectI2CMultiplexer() != HAL_OK) {
        uart_printf("MUX not detected. Retrying in 1s...\r\n");
        osDelay(1000);
    }
    uart_printf("MUX  detected \n");

    for (;;)
    {
//        HAL_StatusTypeDef status;
//
//        // Always select channel 0
//        status = selectI2CChannel(0);
//        if (status != HAL_OK) continue;
//        status = waitForI2C();
//        if (status != HAL_OK) continue;
//
//        // Point to AS5600 angle register
//        g_i2c_tx_reg_addr = AS5600_ANGLE_REGISTER;
//        g_i2c_error_status = HAL_OK;
//        status = HAL_I2C_Master_Transmit_DMA(&hi2c3, AS5600_ADDR, &g_i2c_tx_reg_addr, 1);
//        if (status != HAL_OK) continue;
//        status = waitForI2C();
//        if (status != HAL_OK) continue;
//
//        // Read 2 bytes of angle data
//        g_i2c_error_status = HAL_OK;
//        status = HAL_I2C_Master_Receive_DMA(&hi2c3, AS5600_ADDR, g_i2c_rx_data, 2);
//        if (status != HAL_OK) continue;
//        status = waitForI2C();
//        if (status != HAL_OK) continue;
//
//        // Convert and store
//        int rawAngle = (g_i2c_rx_data[0] << 8) | g_i2c_rx_data[1];
//        float degrees = rawAngle * (360.0f / 4096.0f);
//
//        motors[0].encoder_position = degrees;   // Store in joint 0
//
//        osDelay(1000);
    }
}
// ======================================================================
// ================== I2C DMA COMPLETION CALLBACKS ======================
// ======================================================================

/**
  * @brief  I2C Master Tx Transfer completed callback.
  * @param  hi2c: I2C handle.
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c3.Instance) {
        // Signal the task that the transfer is complete
        osSemaphoreRelease(i2cDmaSemaphore);
    }
}

/**
  * @brief  I2C Master Rx Transfer completed callback.
  * @param  hi2c: I2C handle.
  * @retval None
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c3.Instance) {
        // Signal the task that the transfer is complete
        osSemaphoreRelease(i2cDmaSemaphore);
    }
}

/**
  * @brief  I2C Error callback.
  * @param  hi2c: I2C handle
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == hi2c3.Instance) {
        // Store the error status
        g_i2c_error_status = HAL_ERROR;

        // IMPORTANT: Also release the semaphore to unblock the waiting task,
        // which will then check the error status.
        osSemaphoreRelease(i2cDmaSemaphore);
    }
}

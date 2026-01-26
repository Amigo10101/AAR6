#include "uart_task.h"
#include "Motor_Driver_control.h"
#include "Motor_control.h"
#include "cmsis_os.h"
#include "constants.h"
#include "joint_config.h"
#include "math.h"
#include "structs.h"
#include "tim.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265f

#define STREAM_BUFFER_SIZE 256
#define MAX_RX_ITER 256
__attribute__((aligned(4))) uint8_t txA[STREAM_BUFFER_SIZE];
__attribute__((aligned(4))) uint8_t txB[STREAM_BUFFER_SIZE];
__attribute__((aligned(4))) uint8_t rxDMA[STREAM_BUFFER_SIZE];
static uint8_t startupTelemetry[STREAM_BUFFER_SIZE];
volatile uint8_t dmaBusy = 0;
volatile uint8_t activeBuf = 0;
volatile uint16_t rxReadIndex = 0;
#define TX_PERIOD_MS 10 // 100 Hz

TickType_t lastTxTick = 0;
extern struct MotorStruct motors[NUMBER_OF_JOINTS];
extern struct Robot robot;
extern UART_HandleTypeDef huart2;

//============================logs----------------------//
#define LOG_FUSE_MAX (STREAM_BUFFER_SIZE - 16)
#define LOG_QUEUE_SIZE 128
#define LOG_MAX_LEN 128

typedef struct {
  uint16_t len;
  uint8_t data[LOG_MAX_LEN];
} LogEntry_t;

static LogEntry_t logQueue[LOG_QUEUE_SIZE];
static volatile uint8_t logHead = 0;
static volatile uint8_t logTail = 0;

// ============================================================
// ===================== TX FRAME ============================
// MCU → PC streaming packet structure:
//
// [0]     0xAA
// [1]     0x55
// [2-3]   payload length (little endian)
//
// Payload:
//   for each joint:
//     4 bytes float position
//     4 bytes float velocity
//     4 bytes float acceleration
//   1 byte robot_state (RobotState_t)
//
// [last] XOR checksum of payload bytes
// ============================================================

static uint16_t build_motor_packet(uint8_t *dst, struct MotorStruct *motors) {
  uint16_t index = 0;
  dst[index++] = 0xAA;
  dst[index++] = 0x55;

  uint16_t payloadStart = index;
  index += 2;                   // Reserve 2 bytes for payload length
  dst[index++] = PKT_TELEMETRY; // 👈 ADD THIS

  // --- Motor data ---
  for (uint8_t i = 0; i < NUMBER_OF_JOINTS; i++) {

    /* Position: steps → radians */
    float pos = motors[i].position * motors[i].radians_per_step;

    /* Velocity: steps/sec → rad/sec
     * The ISR calculates velocity in steps/sec, convert to rad/sec */
    float vel = motors[i].velocity * motors[i].radians_per_step;

    /* Acceleration: steps/sec² → rad/sec²
     * The ISR calculates acceleration in steps/sec², convert to rad/sec² */
    float acc = motors[i].acceleration * motors[i].radians_per_step;

    memcpy(&dst[index], &pos, sizeof(float));
    index += sizeof(float);

    memcpy(&dst[index], &vel, sizeof(float));
    index += sizeof(float);

    memcpy(&dst[index], &acc, sizeof(float));
    index += sizeof(float);
  }

  // --- Robot state ---
  dst[index++] = (uint8_t)robot.state;

  // --- Payload length ---
  uint16_t payloadLen = index - payloadStart - 2;
  dst[payloadStart] = payloadLen & 0xFF;
  dst[payloadStart + 1] = payloadLen >> 8;

  // --- XOR checksum ---
  uint8_t chk = 0;
  for (uint16_t i = payloadStart + 2; i < index; i++)
    chk ^= dst[i];
  dst[index++] = chk;

  return index;
}

static uint16_t build_log_packet(uint8_t *dst, LogLevel_t level,
                                 const char *msg, uint16_t msgLen) {
  uint16_t index = 0;
  dst[index++] = 0xAA;
  dst[index++] = 0x55;

  uint16_t payloadStart = index;
  index += 2; // reserve length

  dst[index++] = PKT_LOG;
  dst[index++] = (uint8_t)level;

  memcpy(&dst[index], msg, msgLen);
  index += msgLen;

  uint16_t payloadLen = index - payloadStart - 2;
  dst[payloadStart] = payloadLen & 0xFF;
  dst[payloadStart + 1] = payloadLen >> 8;

  uint8_t chk = 0;
  for (uint16_t i = payloadStart + 2; i < index; i++)
    chk ^= dst[i];

  dst[index++] = chk;

  return index;
}

void uart_printf(const char *fmt, ...) {
  char tmpBuf[LOG_MAX_LEN];

  // 1. Format the string safely
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(tmpBuf, LOG_MAX_LEN, fmt, args);
  va_end(args);

  if (len <= 0)
    return;
  if (len > LOG_MAX_LEN)
    len = LOG_MAX_LEN;

  // ---------------------------------------------------------
  // CASE A: Blocking Mode (Before Homing)
  // ---------------------------------------------------------
  if (robot.state < ROBOT_STATE_HOMED) {
    // Create a temporary buffer on stack for the FULL PACKET
    // (Must be large enough to hold Header + Msg + Checksum)
    uint8_t packetBuf[STREAM_BUFFER_SIZE];

    // Wrap the formatted string into a valid packet [AA 55 ...]
    uint16_t packetLen = build_log_packet(packetBuf, LOG_INFO, tmpBuf, len);

    // Blocking Transmit of the PACKET (not just the string)
    HAL_UART_Transmit(&huart2, packetBuf, packetLen, 100);

    // Small delay to prevent flooding if called in a tight loop
    vTaskDelay(pdMS_TO_TICKS(5));
    return;
  }

  // ---------------------------------------------------------
  // CASE B: Queue Mode (Normal Operation)
  // ---------------------------------------------------------
  uint8_t next = (logHead + 1) % LOG_QUEUE_SIZE;

  // If full, drop the oldest (overwrite) or drop new?
  // Here we drop new if full to avoid blocking the high-speed loop
  if (next == logTail) {
    // Option: just return to avoid freezing motor loop
    // return;
    // Option: Overwrite old (move tail)
    logTail = (logTail + 1) % LOG_QUEUE_SIZE;
  }

  taskENTER_CRITICAL();
  memcpy(logQueue[logHead].data, tmpBuf, len);
  logQueue[logHead].len = len;
  logHead = next;
  taskEXIT_CRITICAL();
}
// ============================================================
// TX Complete Callback
// ============================================================
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2)
    dmaBusy = 0;
}

// ============================================================
// ===================== RX FRAME ============================
// PC → MCU CONTROL PACKET
//
// Frame format:
// [0]     0xAA
// [1]     0x55
// [2-3]   Payload length (uint16, little endian)
//
// Payload formats:
//
// ------------------------------------------------------------
// 1) STATE / CONTROL COMMAND
// ------------------------------------------------------------
// Payload length: 1
//
// [0]     CONTROL_COMMAND (ControlCommand_t)
//
// CONTROL_COMMAND values:
//   0x00 = CMD_NONE
//   0x01 = CMD_ESTOP
//   0x02 = CMD_ESTOP_RELEASE
//   0x03 = CMD_HOME
//   0x04 = CMD_DEHOME
//   0x05 = CMD_DISABLE_MOTORS
//
// ------------------------------------------------------------
// 2) MOTION COMMAND (Position / Velocity)
// ------------------------------------------------------------
// Payload length: 2 + NUMBER_OF_JOINTS * 8
//
// [0]     MOTION_COMMAND (MotionCommand_t)
// [1]     MOTION_ACTION  (MotionAction_t)
//
// For each joint (repeated NUMBER_OF_JOINTS times):
//   4 bytes  float  position   (radians)
//   4 bytes  float  velocity   (rad/s)
//
// MOTION_COMMAND values:
//   0x20 = CMD_MOTION_POSITION   // use position values, ignore velocity
//   0x21 = CMD_MOTION_VELOCITY   // use velocity values, ignore position
//
// MOTION_ACTION values:
//   0x01 = MOTION_START          // start motion
//   0x02 = MOTION_UPDATE         // streaming update
//   0x03 = MOTION_STOP           // stop motion (velocity → 0)
//   0x04 = MOTION_HOLD           // hold last command
//
// ------------------------------------------------------------
// Checksum:
// ------------------------------------------------------------
// [last] XOR checksum of payload bytes
// ============================================================

static void parse_rx_dma(void) {
  uint16_t rxWriteIndex =
      STREAM_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);

  // Add safety counter to prevent infinite loop
  // Max iterations = buffer size to handle wrap-around
  uint16_t maxIterations = STREAM_BUFFER_SIZE;
  uint16_t iterations = 0;

  while (rxReadIndex != rxWriteIndex && iterations < maxIterations) {
    iterations++;

    uint8_t b = rxDMA[rxReadIndex++];
    if (rxReadIndex >= STREAM_BUFFER_SIZE)
      rxReadIndex = 0;
    static enum {
      WAIT_AA,
      WAIT_55,
      WAIT_LEN1,
      WAIT_LEN2,
      WAIT_PAYLOAD,
      WAIT_CHK
    } state = WAIT_AA;
    static uint16_t payloadLen = 0, payloadPos = 0;
    static uint8_t payload[STREAM_BUFFER_SIZE];
    static uint8_t checksum = 0;
    switch (state) {
    case WAIT_AA:
      if (b == 0xAA)
        state = WAIT_55;
      break;
    case WAIT_55:
      state = (b == 0x55) ? WAIT_LEN1 : WAIT_AA;
      break;
    case WAIT_LEN1:
      payloadLen = b;
      state = WAIT_LEN2;
      break;
    case WAIT_LEN2:
      payloadLen |= (b << 8);
      if (payloadLen > STREAM_BUFFER_SIZE) {
        state = WAIT_AA;
        break;
      }
      payloadPos = 0;
      checksum = 0;
      state = WAIT_PAYLOAD;
      break;
    case WAIT_PAYLOAD:
      payload[payloadPos++] = b;
      checksum ^= b;
      if (payloadPos >= payloadLen)
        state = WAIT_CHK;
      break;
    case WAIT_CHK:
      if (b == checksum) {
        // Process control command
        if (payloadLen == 1) {
          ControlCommand_t cmd = (ControlCommand_t)payload[0];
          robot.command = cmd;
          switch (cmd) {
          case CMD_ACK:
            robot.state = ROBOT_STATE_START;
            while (dmaBusy) {
              osDelay(5);
            }
            uint16_t size = build_motor_packet(startupTelemetry, motors);
            HAL_UART_Transmit_DMA(&huart2, startupTelemetry, size);
            dmaBusy = 1;
            uart_printf("[MCU] ACK received, ready for CMD_START");
            break;
          case CMD_START:
            uart_printf("[MCU] START received, motors initialized");
            tmc5160_init();
            break;
          case CMD_HOME:
            robot.state = ROBOT_STATE_HOMING;
            Home_Motors();
            robot.state = ROBOT_STATE_HOMED;
            break;
          case CMD_ESTOP:
            robot.state = ROBOT_STATE_ESTOP;
            tmc5160_disable_driver();
            break;
          case CMD_ESTOP_RELEASE:
            tmc5160_init();
            robot.state = ROBOT_STATE_STANDSTILL;
            break;
          case CMD_DEHOME:
            robot.state = ROBOT_STATE_WAIT_PC;
            break;
          case CMD_DISABLE:
            tmc5160_disable_driver();
            robot.state = ROBOT_STATE_DISABLED;
            break;
          }
        } else if (payloadLen ==
                   2 + NUMBER_OF_JOINTS * 8) { // position/velocity
          MotionCommand_t mode = (MotionCommand_t)payload[0];
          MotionAction_t action = (MotionAction_t)payload[1];
          uint16_t idx = 2;
          if (robot.state == ROBOT_STATE_ESTOP ||
              robot.state == ROBOT_STATE_DISABLED)
            return;
          for (uint8_t i = 0; i < NUMBER_OF_JOINTS; i++) {

            float pos, vel;
            memcpy(&pos, &payload[idx], 4);
            idx += 4;
            memcpy(&vel, &payload[idx], 4);
            idx += 4;

            switch (mode) {

            case CMD_MOTION_POSITION:
              if (action != MOTION_STOP)
                Set_Motor_Angle(&motors[i], pos);
              break;

            case CMD_MOTION_VELOCITY:
              if (action == MOTION_START || action == MOTION_UPDATE)
            	  Set_Motor_Angular_Velocity(&motors[i], vel);
              else if (action == MOTION_STOP)
            	  Set_Motor_Angular_Velocity(&motors[i], 0.0f);
              break;

            default:
              break;
            }
          }
        }
      }
      state = WAIT_AA; // reset for next packet
      break;
    }
  }
} // ============================================================
// ===================== UART TASK ============================
// Implements startup handshake + homing + TX streaming
// ============================================================
void StartUartTask(void *argument) {
  // --- Startup: wait for PC ---
  robot.state = ROBOT_STATE_WAIT_PC;

  HAL_UART_Receive_DMA(&huart2, rxDMA, STREAM_BUFFER_SIZE); // circular RX

  for (;;) {
    parse_rx_dma();

    if (robot.state < ROBOT_STATE_START) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    TickType_t now = xTaskGetTickCount();

    /* =========================
     * 1️ Telemetry (highest priority)
     * ========================= */
    if (robot.state > ROBOT_STATE_HOMED &&
        (now - lastTxTick) >= pdMS_TO_TICKS(10) && !dmaBusy) {
      uint8_t *buf = (activeBuf == 0) ? txA : txB;
      uint16_t size = build_motor_packet(buf, motors);

      dmaBusy = 1;
      HAL_UART_Transmit_DMA(&huart2, buf, size);
      activeBuf ^= 1;

      lastTxTick = now;
    }

    /* ========================
     * 2️Logs (best-effort)
     * ========================= */
    if (!dmaBusy && logTail != logHead) {
      if (robot.state < ROBOT_STATE_HOMED) {
        while (dmaBusy) {
          osDelay(1);
        }
      }
      uint8_t *buf = (activeBuf == 0) ? txA : txB;
      uint16_t idx = 0;

      buf[idx++] = 0xAA;
      buf[idx++] = 0x55;

      uint16_t lenPos = idx;
      idx += 2;

      buf[idx++] = PKT_LOG;
      buf[idx++] = LOG_INFO;

      while (logTail != logHead) {

        taskENTER_CRITICAL();
        LogEntry_t *e = &logQueue[logTail];
        taskEXIT_CRITICAL();

        if (idx + e->len + 1 >= LOG_FUSE_MAX)
          break;

        memcpy(&buf[idx], e->data, e->len);
        idx += e->len;
        buf[idx++] = '\n';

        logTail = (logTail + 1) % LOG_QUEUE_SIZE;
      }

      uint16_t payloadLen = idx - lenPos - 2;
      buf[lenPos] = payloadLen & 0xFF;
      buf[lenPos + 1] = payloadLen >> 8;

      uint8_t chk = 0;
      for (uint16_t i = lenPos + 2; i < idx; i++)
        chk ^= buf[i];

      buf[idx++] = chk;

      dmaBusy = 1;
      HAL_UART_Transmit_DMA(&huart2, buf, idx);
      activeBuf ^= 1;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

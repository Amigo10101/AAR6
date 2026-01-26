#pragma once
#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <vector>

// ================== Enums ==================

// Robot state received from MCU
typedef enum {
  ROBOT_STATE_ERROR = 0,    // Any fault or error state
  ROBOT_STATE_WAIT_PC = 1,  // Waiting for PC connection
  ROBOT_STATE_START = 2,    // Robot has just powered on, before initialization
  ROBOT_STATE_DISABLED = 3, // Robot is disabled
  ROBOT_STATE_HOMING = 4,   // Robot is performing homing routine
  ROBOT_STATE_HOMED = 5,    // Robot has completed homing
  ROBOT_STATE_ESTOP = 6,    // Emergency stop active
  ROBOT_STATE_STANDSTILL = 7, // Robot is idle, motors not moving, also homed
  ROBOT_STATE_MOVING = 8,     // Robot is executing a motion
  ROBOT_STATE_EXECUTING = 9   // Robot executing a command/trajectory
} RobotState_t;

// Control commands sent from PC → MCU
typedef enum {
  CMD_NONE = 0,          // No command
  CMD_ACK = 1,           // Found by PC
  CMD_START = 2,         // Start the homing process
  CMD_ESTOP = 3,         // Emergency stop
  CMD_ESTOP_RELEASE = 4, // Release E-stop
  CMD_HOME = 5,          // Start homing
  CMD_DEHOME = 6,        // Dehome (reset home)
  CMD_DISABLE = 7        // Disable motors
} ControlCommand_t;

// Motion command types (PC → MCU)
typedef enum {
  CMD_MOTION_POSITION = 0x20, // Use position values, ignore velocity
  CMD_MOTION_VELOCITY = 0x21  // Use velocity values, ignore position
} MotionCommand_t;

// Motion action (used with motion commands)
typedef enum {
  MOTION_START = 0x01,  // Start motion
  MOTION_UPDATE = 0x02, // Streaming update
  MOTION_STOP = 0x03,   // Stop motion (velocity → 0)
  MOTION_HOLD = 0x04    // Hold last command
} MotionAction_t;

// Packet types (MCU → PC)
typedef enum {
  PKT_TELEMETRY = 0x01, // Joint data + robot state
  PKT_LOG = 0xF0        // Log message from MCU
} PacketType_t;

// Log levels (MCU → PC)
typedef enum {
  LOG_INFO = 0,
  LOG_WARN = 1,
  LOG_ERROR = 2,
  LOG_DEBUG = 3
} LogLevel_t;

// Joint structure
struct JointState {
  float position;
  float velocity;
  float acceleration;
};

// ================== Data Frames ==================
//
// MCU → PC (telemetry):
// [0]  0xAA
// [1]  0x55
// [2-3] payload length (little endian)
// [4]  PKT_TELEMETRY (0x01)
// [5..] Motor data: each joint 12 bytes (pos[4], vel[4], acc[4])
// [last payload byte] robot_state (RobotState_t)
// [checksum] XOR of payload bytes
//
// MCU → PC (log):
// [0]  0xAA
// [1]  0x55
// [2-3] payload length (little endian)
// [4]  PKT_LOG (0xF0)
// [5]  LogLevel_t
// [6..] message string (variable length)
// [checksum] XOR of payload bytes
//
// PC → MCU (control):
// [0] 0xAA
// [1] 0x55
// [2-3] payload length (usually 1)
// [4] COMMAND_TYPE (ControlCommand_t)
// [checksum] XOR of payload bytes
//

class RobotSerial : public QObject {
  Q_OBJECT
public:
  RobotSerial(QObject *parent = nullptr);

  // Serial port setup
  bool openPort(const QString &portName, int baudRate);
  void closePort();
  bool isOpen() const;

  // Send command to robot
  void sendCommand(ControlCommand_t cmd);
  void sendHome() { sendCommand(CMD_HOME); }
  void sendEstop() { sendCommand(CMD_ESTOP); }
  void sendReleaseEstop() { sendCommand(CMD_ESTOP_RELEASE); }
  void sendDehome() { sendCommand(CMD_DEHOME); }
  void sendDisable() { sendCommand(CMD_DISABLE); }

  // Send motion data with new protocol
  void sendMotionData(MotionCommand_t mode, MotionAction_t action,
                      const QByteArray &jointData);

  // Get latest robot state
  RobotState_t getRobotState() const { return robotState; }
  std::vector<JointState> getJointStates() const { return jointStates; }

signals:
  void newJointData();
  void errorOccurred(const QString &error);
  void logReceived(const QString &level, const QString &message);
  void
  firstPacketReceived(); // Emitted on first valid packet (telemetry or log)

private slots:
  void readSerial();

private:
  QSerialPort serial;
  std::vector<JointState> jointStates;
  RobotState_t robotState;

  // Parser state (must be reset on disconnect)
  QByteArray rxBuffer;
  bool firstPacketEmitted;

  // Helpers
  void parseRxData(const QByteArray &data);
  void resetParserState();
};

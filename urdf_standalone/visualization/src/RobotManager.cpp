#include "RobotManager.h"
#include <QDebug>
#include <QtSerialPort/QSerialPortInfo>

RobotManager::RobotManager(QObject *parent)
    : QObject(parent), m_serial(new RobotSerial(this)),
      m_scanTimer(new QTimer(this)), m_connected(false),
      m_robotAcknowledged(false), m_lastState(ROBOT_STATE_WAIT_PC) {
  connect(m_serial, &RobotSerial::newJointData, this,
          &RobotManager::onNewJointData);
  connect(m_serial, &RobotSerial::errorOccurred, this,
          &RobotManager::onSerialError);
  connect(m_serial, &RobotSerial::firstPacketReceived, this,
          &RobotManager::onFirstPacket);
  connect(m_serial, &RobotSerial::logReceived, this,
          &RobotManager::logReceived); // Forward MCU logs to QML

  // Auto-scan for robot every 2 seconds
  connect(m_scanTimer, &QTimer::timeout, this, &RobotManager::checkForRobot);
  m_scanTimer->start(2000);
}

RobotManager::~RobotManager() {
  if (m_connected) {
    m_serial->closePort();
  }
}

QString RobotManager::getRobotStateText() const {
  if (!m_connected)
    return "Not Connected";
  return stateToString(m_serial->getRobotState());
}

bool RobotManager::isRobotReady() const {
  if (!m_connected)
    return false;
  RobotState_t state = m_serial->getRobotState();
  // Robot is ready if state >= ROBOT_STATE_STANDSTILL (7)
  return (state >= ROBOT_STATE_STANDSTILL);
}

void RobotManager::scanForRobot() { checkForRobot(); }

void RobotManager::connectRobot(const QString &port, int baudRate) {
  if (m_connected) {
    m_serial->closePort();
    m_connected = false;
    emit connectedChanged();
  }

  if (m_serial->openPort(port, baudRate)) {
    m_connected = true;
    m_portName = port;
    qDebug() << "Connected to robot at" << port << "baud:" << baudRate;
    emit connectedChanged();
    emit portNameChanged();
  } else {
    emit errorOccurred("Failed to open port: " + port);
  }
}

void RobotManager::disconnectRobot() {
  if (m_serial->isOpen() || m_connected) {
    m_serial->closePort();
    m_connected = false;
    m_robotAcknowledged = false;
    m_portName.clear();
    m_lastState = ROBOT_STATE_WAIT_PC; // Reset last state

    emit connectedChanged();
    emit portNameChanged();
    emit robotStateChanged(); // Update state text to "Not Connected"
    emit robotReadyChanged(); // Update ready status
  }

  // Restart scan timer if we were auto-detecting
  if (!m_scanTimer->isActive()) {
    m_scanTimer->start(2000);
  }
}

void RobotManager::sendCommand(int cmd) {
  if (!m_serial || !m_serial->isOpen()) {
    qDebug() << "Cannot send command - serial not open";
    return;
  }

  qDebug() << "Sending command:" << cmd;
  m_serial->sendCommand(static_cast<ControlCommand_t>(cmd));
}

void RobotManager::sendHome() {
  qDebug() << "Sending HOME command";
  if (m_serial && m_serial->isOpen()) {
    m_serial->sendHome();
  }
}

void RobotManager::sendEstop() {
  if (m_connected) {
    qDebug() << "Sending ESTOP command";
    m_serial->sendEstop();
  }
}

void RobotManager::sendReleaseEstop() {
  if (m_connected) {
    qDebug() << "Sending RELEASE ESTOP command";
    m_serial->sendReleaseEstop();
  }
}

void RobotManager::sendDehome() {
  if (m_connected) {
    qDebug() << "Sending DEHOME command";
    m_serial->sendDehome();
  }
}

void RobotManager::sendDisable() {
  if (m_connected) {
    qDebug() << "Sending DISABLE command";
    m_serial->sendDisable();
  }
}

QStringList RobotManager::getAvailablePorts() {
  QStringList ports;
  const auto infos = QSerialPortInfo::availablePorts();
  for (const QSerialPortInfo &info : infos) {
    if (info.portName().startsWith("ttyACM")) {
      ports << "/dev/" + info.portName();
    }
  }
  return ports;
}

void RobotManager::onNewJointData() {
  // Get joint states from serial - not used in UI yet
  // Will be used later to update graph data

  RobotState_t currentState = m_serial->getRobotState();
  // If this is the first data packet with valid state, show popup/acknowledge
  if (!m_robotAcknowledged && currentState != ROBOT_STATE_ERROR) {
    m_robotAcknowledged = true;
    m_connected = true;
    qDebug() << "Robot acknowledged! State:" << stateToString(currentState);
    emit connectedChanged();
    emit robotFound(m_portName);
  }
  // Check if state changed
  if (currentState != m_lastState) {
    m_lastState = currentState;
    qDebug() << "Robot state:" << stateToString(currentState) << "("
             << currentState << ")";
    emit robotStateChanged();
    emit robotReadyChanged();
  }

  // Emit signal for graph updates
  emit jointDataUpdated();
}

QVariantList RobotManager::getJointStates() {
  if (!m_serial || !m_serial->isOpen()) {
    return QVariantList();
  }

  auto jointStates = m_serial->getJointStates();
  QVariantList result;

  for (const auto &joint : jointStates) {
    QVariantMap map;
    map.insert("position", joint.position);
    map.insert("velocity", joint.velocity);
    map.insert("acceleration", joint.acceleration);
    result.append(map);
  }

  return result;
}

void RobotManager::sendJointCommand(int jointIndex, double position,
                                    double velocity) {
  if (!m_serial || jointIndex < 0 || jointIndex >= 7) {
    return;
  }

  // Get current joint states
  auto states = getJointStates();

  // Construct payload with current positions for all joints
  QByteArray payload;
  payload.resize(56); // 7 joints * 8 bytes (4 for position + 4 for velocity)

  for (int i = 0; i < 7; i++) {
    float pos, vel;

    if (i == jointIndex) {
      // Use the commanded position/velocity for this joint
      pos = static_cast<float>(position);
      vel = static_cast<float>(velocity);
    } else {
      // Preserve current position/velocity for other joints
      if (i < states.length()) {
        pos = static_cast<float>(states[i].toMap()["position"].toDouble());
        vel = static_cast<float>(states[i].toMap()["velocity"].toDouble());
      } else {
        pos = 0.0f;
        vel = 0.0f;
      }
    }

    memcpy(&payload.data()[i * 8], &pos, 4);
    memcpy(&payload.data()[i * 8 + 4], &vel, 4);
  }

  m_serial->sendMotionData(CMD_MOTION_POSITION, MOTION_UPDATE, payload);
}

void RobotManager::resetAllJoints() {
  if (!m_serial) {
    return;
  }

  // Construct payload with all zeros
  QByteArray payload;
  payload.resize(56); // 7 joints * 8 bytes (4 for position + 4 for velocity)

  for (int i = 0; i < 7; i++) {
    float pos = 0.0f;
    float vel = 0.0f;

    memcpy(&payload.data()[i * 8], &pos, 4);
    memcpy(&payload.data()[i * 8 + 4], &vel, 4);
  }

  m_serial->sendMotionData(CMD_MOTION_POSITION, MOTION_UPDATE, payload);
  qDebug() << "Sent reset command: all joints to 0.0";
}

void RobotManager::sendAllJoints(const QVariantList &positions) {
  if (!m_serial || !m_serial->isOpen()) {
    qDebug() << "sendAllJoints: Serial not open";
    return;
  }

  // Get current joint states to preserve gripper
  auto states = getJointStates();

  // Construct payload with all joint positions
  QByteArray payload;
  payload.resize(56); // 7 joints * 8 bytes (4 for position + 4 for velocity)

  for (int i = 0; i < 7; i++) {
    float pos = 0.0f;
    float vel = 0.0f;

    if (i < positions.length()) {
      // Use IK positions for arm joints (0-5)
      pos = static_cast<float>(positions[i].toDouble());
    } else if (i < states.length()) {
      // Preserve current position for gripper (joint 6)
      pos = static_cast<float>(states[i].toMap()["position"].toDouble());
    }

    memcpy(&payload.data()[i * 8], &pos, 4);
    memcpy(&payload.data()[i * 8 + 4], &vel, 4);
  }

  m_serial->sendMotionData(CMD_MOTION_POSITION, MOTION_UPDATE, payload);
  qDebug() << "Sent all joints:" << positions;
}

void RobotManager::sendJointVelocities(const QVariantList &velocities) {
  if (!m_serial || !m_serial->isOpen()) {
    qDebug() << "sendJointVelocities: Serial not open";
    return;
  }

  // Construct payload: [pos, vel] for each joint
  // In velocity mode, pos is ignored, vel is used
  QByteArray payload;
  payload.resize(56); // 7 joints * 8 bytes

  for (int i = 0; i < 7; i++) {
    float pos = 0.0f; // Position ignored in velocity mode
    float vel = 0.0f;

    if (i < velocities.length()) {
      vel = static_cast<float>(velocities[i].toDouble());
    }

    memcpy(&payload.data()[i * 8], &pos, 4);
    memcpy(&payload.data()[i * 8 + 4], &vel, 4);
  }

  m_serial->sendMotionData(CMD_MOTION_VELOCITY, MOTION_UPDATE, payload);
}

void RobotManager::stopMotion() {
  if (!m_serial || !m_serial->isOpen()) {
    return;
  }

  // Send zero velocities with MOTION_STOP action
  QByteArray payload(56, '\0'); // All zeros
  m_serial->sendMotionData(CMD_MOTION_VELOCITY, MOTION_STOP, payload);
  qDebug() << "Sent MOTION_STOP";
}

void RobotManager::onSerialError(const QString &err) {
  emit errorOccurred(err);
}

void RobotManager::onFirstPacket() {
  // Called when first valid packet (telemetry or log) is received from MCU
  qDebug() << "First packet received from robot";
  if (!m_connected) {
    m_connected = true;
    emit connectedChanged();
    emit robotStateChanged();
    emit robotReadyChanged();
  }
}

void RobotManager::checkForRobot() {
  if (m_connected)
    return; // Already connected

  QStringList ports = getAvailablePorts();
  if (!ports.isEmpty()) {
    QString port = ports.first();
    qDebug() << "Device found at:" << port;

    // Open port but don't mark as connected yet
    if (m_serial->openPort(port, 1500000)) {
      m_portName = port;
      m_scanTimer->stop(); // Stop scanning

      // Send CMD_ACK to acknowledge we found the robot
      qDebug() << "Sending CMD_ACK to robot...";
      m_serial->sendCommand(CMD_ACK);

      // Wait for robot to send data with robot_state = 0
      // Popup will show in onNewJointData when state = ROBOT_STATE_WAIT_PC
    }
  }
}

QString RobotManager::stateToString(RobotState_t state) const {
  switch (state) {
  case ROBOT_STATE_ERROR:
    return "Error";
  case ROBOT_STATE_WAIT_PC:
    return "Wait PC";
  case ROBOT_STATE_START:
    return "Start";
  case ROBOT_STATE_ESTOP:
    return "E-Stop";
  case ROBOT_STATE_DISABLED:
    return "Disabled";
  case ROBOT_STATE_HOMING:
    return "Homing";
  case ROBOT_STATE_HOMED:
    return "Homed";
  case ROBOT_STATE_STANDSTILL:
    return "Standstill";
  case ROBOT_STATE_MOVING:
    return "Moving";
  case ROBOT_STATE_EXECUTING:
    return "Executing";
  default:
    return "Unknown";
  }
}

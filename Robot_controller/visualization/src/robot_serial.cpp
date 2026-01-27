#include "robot_serial.h"
#include <QDebug>
#include <cstring>

RobotSerial::RobotSerial(QObject *parent)
    : QObject(parent), robotState(ROBOT_STATE_WAIT_PC),
      firstPacketEmitted(false) {
  connect(&serial, &QSerialPort::readyRead, this, &RobotSerial::readSerial);
}

// ---------------- Serial Port ----------------
bool RobotSerial::openPort(const QString &portName, qint32 baudRate) {
  serial.setPortName(portName);
  serial.setBaudRate(baudRate);
  serial.setDataBits(QSerialPort::Data8);
  serial.setParity(QSerialPort::NoParity);
  serial.setStopBits(QSerialPort::OneStop);
  serial.setFlowControl(QSerialPort::NoFlowControl);

  if (!serial.open(QIODevice::ReadWrite))
    return false;

  return true;
}

void RobotSerial::closePort() {
  if (serial.isOpen())
    serial.close();
  resetParserState();
}

void RobotSerial::resetParserState() {
  rxBuffer.clear();
  firstPacketEmitted = false;
  robotState = ROBOT_STATE_WAIT_PC;
  qDebug() << "[Serial] Parser state reset";
}

bool RobotSerial::isOpen() const { return serial.isOpen(); }

// ---------------- Commands ----------------
void RobotSerial::sendCommand(ControlCommand_t cmd) {
  if (!serial.isOpen())
    return;

  QByteArray packet;
  packet.append(static_cast<char>(0xAA));
  packet.append(static_cast<char>(0x55));

  uint16_t payloadLen = 1;
  packet.append(static_cast<char>(payloadLen & 0xFF)); // LSB
  packet.append(static_cast<char>(payloadLen >> 8));   // MSB

  // payload
  packet.append(static_cast<char>(cmd));

  // checksum = XOR of payload bytes ONLY (not length)
  uint8_t chk = static_cast<uint8_t>(cmd);
  packet.append(static_cast<char>(chk));

  serial.write(packet);
  serial.flush();
}

void RobotSerial::sendMotionData(MotionCommand_t mode, MotionAction_t action,
                                 const QByteArray &jointData) {
  if (!serial.isOpen()) {
    qDebug() << "Serial port not open";
    return;
  }

  // New protocol: payload = [mode][action][jointData...]
  // Payload length = 2 + jointData.size()
  QByteArray packet;
  packet.append(static_cast<char>(0xAA));
  packet.append(static_cast<char>(0x55));

  // Length (little endian, 2 bytes): 2 header bytes + joint data
  uint16_t len = 2 + jointData.size();
  packet.append(static_cast<char>(len & 0xFF));
  packet.append(static_cast<char>((len >> 8) & 0xFF));

  // Payload: [mode][action][jointData...]
  packet.append(static_cast<char>(mode));
  packet.append(static_cast<char>(action));
  packet.append(jointData);

  // Checksum (XOR of all payload bytes)
  uint8_t checksum = static_cast<uint8_t>(mode) ^ static_cast<uint8_t>(action);
  for (int i = 0; i < jointData.size(); i++) {
    checksum ^= static_cast<uint8_t>(jointData[i]);
  }
  packet.append(static_cast<char>(checksum));

  serial.write(packet);
  serial.flush();
}

// ---------------- Receive streaming ----------------
void RobotSerial::readSerial() {
  QByteArray data = serial.readAll();
  parseRxData(data);
}

// Parses MCU → PC streaming frames (telemetry and log packets)
void RobotSerial::parseRxData(const QByteArray &data) {
  rxBuffer.append(data);

  while (rxBuffer.size() >= 4) // minimum header+length
  {
    // Find header
    int idx = rxBuffer.indexOf(0xAA);
    if (idx < 0) {
      rxBuffer.clear();
      return;
    }
    if (rxBuffer.size() < idx + 2)
      return; // wait for 0x55
    if (rxBuffer[idx + 1] != 0x55) {
      rxBuffer.remove(0, idx + 2);
      continue;
    }

    if (rxBuffer.size() < idx + 4)
      return; // wait for length bytes

    uint16_t len = static_cast<uint8_t>(rxBuffer[idx + 2]) |
                   (static_cast<uint8_t>(rxBuffer[idx + 3]) << 8);
    if (rxBuffer.size() < idx + 4 + len + 1)
      return; // wait for full packet including checksum

    QByteArray packet = rxBuffer.mid(idx, 4 + len + 1);
    rxBuffer.remove(0, idx + 4 + len + 1);

    // Check checksum
    uint8_t chk = 0;
    for (int i = 4; i < 4 + len; i++)
      chk ^= static_cast<uint8_t>(packet[i]);
    if (chk != static_cast<uint8_t>(packet[4 + len]))
      continue; // invalid packet

    // Get packet type (first byte of payload)
    // NOTE: Must cast to uint8_t first! QByteArray returns signed char,
    // which would make 0xF0 become -16 and fail comparison with PKT_LOG (240)
    if (len < 1)
      continue;
    uint8_t pktTypeByte = static_cast<uint8_t>(packet[4]);
    PacketType_t pktType = static_cast<PacketType_t>(pktTypeByte);

    // Packet type parsed - no debug log to avoid flooding

    // Emit firstPacketReceived on first valid packet (for connection detection)
    if (!firstPacketEmitted) {
      firstPacketEmitted = true;
      qDebug() << "[Serial] First valid packet received - emitting "
                  "firstPacketReceived";
      emit firstPacketReceived();
    }

    if (pktType == PKT_TELEMETRY) {
      // Telemetry packet: [PKT_TYPE][joint data...][robot_state]
      // Joint data starts at offset 5 (after packet type byte)
      int dataLen = len - 1;               // exclude packet type byte
      int jointCount = (dataLen - 1) / 12; // last byte = robot state
      jointStates.resize(jointCount);
      int payloadIdx = 5; // start after header(4) + packet type(1)
      for (int i = 0; i < jointCount; i++) {
        memcpy(&jointStates[i].position, &packet[payloadIdx], 4);
        payloadIdx += 4;
        memcpy(&jointStates[i].velocity, &packet[payloadIdx], 4);
        payloadIdx += 4;
        memcpy(&jointStates[i].acceleration, &packet[payloadIdx], 4);
        payloadIdx += 4;
      }
      robotState = static_cast<RobotState_t>(packet[payloadIdx]);

      emit newJointData();
    } else if (pktType == PKT_LOG) {
      // Log packet: [PKT_TYPE][LogLevel][message...]
      if (len < 2)
        continue;
      LogLevel_t logLevel = static_cast<LogLevel_t>(packet[5]);
      int msgLen = len - 2; // exclude packet type and log level
      QString message = QString::fromUtf8(packet.mid(6, msgLen));

      // Convert log level to string
      QString levelStr;
      switch (logLevel) {
      case LOG_INFO:
        levelStr = "INFO";
        break;
      case LOG_WARN:
        levelStr = "WARN";
        break;
      case LOG_ERROR:
        levelStr = "ERROR";
        break;
      case LOG_DEBUG:
        levelStr = "DEBUG";
        break;
      default:
        levelStr = "INFO";
        break;
      }

      emit logReceived(levelStr, message);
    } else {
      qDebug() << "[Serial] Unknown packet type:" << Qt::hex << (int)pktType;
    }
    // Unknown packet types are silently ignored
  }
}

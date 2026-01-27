#pragma once
#include "robot_serial.h"
#include <QObject>
#include <QStringList>
#include <QTimer>
#include <QVariant>
#include <QVariantList>
#include <QVariantMap>

class RobotManager : public QObject {
  Q_OBJECT
  Q_PROPERTY(bool connected READ isConnected NOTIFY connectedChanged)
  Q_PROPERTY(
      QString robotStateText READ getRobotStateText NOTIFY robotStateChanged)
  Q_PROPERTY(QString portName READ getPortName NOTIFY portNameChanged)
  Q_PROPERTY(bool robotReady READ isRobotReady NOTIFY robotReadyChanged)

public:
  explicit RobotManager(QObject *parent = nullptr);
  ~RobotManager();

  // Properties
  bool isConnected() const { return m_connected; }
  QString getRobotStateText() const;
  QString getPortName() const { return m_portName; }
  bool isRobotReady() const;

  // Invokable methods
  Q_INVOKABLE void scanForRobot();
  Q_INVOKABLE void connectRobot(const QString &port, int baudRate);
  Q_INVOKABLE void disconnectRobot();
  Q_INVOKABLE void sendCommand(int cmd); // Generic command sender
  Q_INVOKABLE void sendHome();
  Q_INVOKABLE void sendJointCommand(int jointIndex, double position,
                                    double velocity);
  Q_INVOKABLE void resetAllJoints(); // Reset all joints to zero
  Q_INVOKABLE void sendEstop();
  Q_INVOKABLE void sendReleaseEstop();
  Q_INVOKABLE void sendDehome();
  Q_INVOKABLE void sendDisable();
  Q_INVOKABLE QStringList getAvailablePorts();
  Q_INVOKABLE QVariantList
  getJointStates(); // Get current joint data for graphing
  Q_INVOKABLE void
  sendAllJoints(const QVariantList &positions); // Send all 6 joints at once
  Q_INVOKABLE void
  sendJointVelocities(const QVariantList &velocities); // Send joint velocities
  Q_INVOKABLE void stopMotion(); // Stop robot motion (send zero velocities)

signals:
  void connectedChanged();
  void robotStateChanged();
  void portNameChanged();
  void robotReadyChanged();
  void robotFound(QString port);
  void errorOccurred(QString message);
  void jointDataUpdated(); // Emitted when new joint data is available
  void logReceived(QString level, QString message); // Forward MCU logs to QML

private slots:
  void onNewJointData();
  void onSerialError(const QString &err);
  void checkForRobot();
  void onFirstPacket();

private:
  RobotSerial *m_serial;
  QTimer *m_scanTimer;
  bool m_connected;
  bool m_robotAcknowledged; // True after first valid packet received
  QString m_portName;
  RobotState_t m_lastState;

  QString stateToString(RobotState_t state) const;
};

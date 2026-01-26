#ifndef APPSTATE_H
#define APPSTATE_H

#include <QObject>
#include <QVariantList>
#include <QVariantMap>
#include <QVector3D>
#include <QtQml/qqmlregistration.h>

/**
 * AppState - Consolidated application state for the robot visualization
 *
 * This QObject centralizes scattered window properties to simplify
 * passing state to QML components via a single appState property.
 */
class AppState : public QObject {
  Q_OBJECT
  QML_ELEMENT

  // ========================================
  // MODE CONTROL
  // ========================================
  Q_PROPERTY(bool isSimulationMode READ isSimulationMode WRITE
                 setIsSimulationMode NOTIFY isSimulationModeChanged)
  Q_PROPERTY(
      bool showGizmo READ showGizmo WRITE setShowGizmo NOTIFY showGizmoChanged)

  // ========================================
  // IK CONTROL
  // ========================================
  Q_PROPERTY(
      bool ikEnabled READ ikEnabled WRITE setIkEnabled NOTIFY ikEnabledChanged)
  Q_PROPERTY(
      bool ikSolving READ ikSolving WRITE setIkSolving NOTIFY ikSolvingChanged)
  Q_PROPERTY(QString ikStatusText READ ikStatusText WRITE setIkStatusText NOTIFY
                 ikStatusTextChanged)
  Q_PROPERTY(QVector3D lastValidGizmoPosition READ lastValidGizmoPosition WRITE
                 setLastValidGizmoPosition NOTIFY lastValidGizmoPositionChanged)
  Q_PROPERTY(QVariantList ikJointNames READ ikJointNames CONSTANT)

  // ========================================
  // CARTESIAN CONTROL
  // ========================================
  Q_PROPERTY(QVariantList cartesianStepSizes READ cartesianStepSizes CONSTANT)
  Q_PROPERTY(int cartesianStepIndex READ cartesianStepIndex WRITE
                 setCartesianStepIndex NOTIFY cartesianStepIndexChanged)
  Q_PROPERTY(double cartesianStepSize READ cartesianStepSize NOTIFY
                 cartesianStepIndexChanged)

  Q_PROPERTY(QVariantList rotationStepSizes READ rotationStepSizes CONSTANT)
  Q_PROPERTY(int rotationStepIndex READ rotationStepIndex WRITE
                 setRotationStepIndex NOTIFY rotationStepIndexChanged)
  Q_PROPERTY(double rotationStepSize READ rotationStepSize NOTIFY
                 rotationStepIndexChanged)

  Q_PROPERTY(bool useLocalFrame READ useLocalFrame WRITE setUseLocalFrame NOTIFY
                 useLocalFrameChanged)

  // ========================================
  // JOINT STATE
  // ========================================
  Q_PROPERTY(QVariantMap jointAngles READ jointAngles WRITE setJointAngles
                 NOTIFY jointAnglesChanged)
  Q_PROPERTY(QVariantList currentJointPositions READ currentJointPositions WRITE
                 setCurrentJointPositions NOTIFY currentJointPositionsChanged)
  Q_PROPERTY(
      QVariantList simulationJointPositions READ simulationJointPositions WRITE
          setSimulationJointPositions NOTIFY simulationJointPositionsChanged)
  Q_PROPERTY(QVariantList jointColors READ jointColors CONSTANT)

  // ========================================
  // GRAPH STATE
  // ========================================
  Q_PROPERTY(int maxDataPoints READ maxDataPoints CONSTANT)
  Q_PROPERTY(int dataUpdateTrigger READ dataUpdateTrigger NOTIFY
                 dataUpdateTriggerChanged)
  Q_PROPERTY(double graphTimeZoom READ graphTimeZoom WRITE setGraphTimeZoom
                 NOTIFY graphTimeZoomChanged)
  Q_PROPERTY(double graphTimeOffset READ graphTimeOffset WRITE
                 setGraphTimeOffset NOTIFY graphTimeOffsetChanged)
  Q_PROPERTY(QVariantList selectedJoints READ selectedJoints WRITE
                 setSelectedJoints NOTIFY selectedJointsChanged)
  Q_PROPERTY(bool useRadians READ useRadians WRITE setUseRadians NOTIFY
                 useRadiansChanged)

  // ========================================
  // COLLISION STATE
  // ========================================
  Q_PROPERTY(QVariantList collidingPairs READ collidingPairs WRITE
                 setCollidingPairs NOTIFY collidingPairsChanged)
  Q_PROPERTY(QVariantMap collidingLinks READ collidingLinks WRITE
                 setCollidingLinks NOTIFY collidingLinksChanged)
  Q_PROPERTY(QVariantList collisionColors READ collisionColors CONSTANT)

  // ========================================
  // CAMERA STATE
  // ========================================
  Q_PROPERTY(double cameraDistance READ cameraDistance WRITE setCameraDistance
                 NOTIFY cameraDistanceChanged)
  Q_PROPERTY(
      double panSpeed READ panSpeed WRITE setPanSpeed NOTIFY panSpeedChanged)
  Q_PROPERTY(double orbitSpeed READ orbitSpeed WRITE setOrbitSpeed NOTIFY
                 orbitSpeedChanged)
  Q_PROPERTY(double zoomSpeed READ zoomSpeed WRITE setZoomSpeed NOTIFY
                 zoomSpeedChanged)

  // ========================================
  // SETTINGS
  // ========================================
  Q_PROPERTY(QString selectedDevice READ selectedDevice WRITE setSelectedDevice
                 NOTIFY selectedDeviceChanged)
  Q_PROPERTY(int selectedBaudrate READ selectedBaudrate WRITE
                 setSelectedBaudrate NOTIFY selectedBaudrateChanged)
  Q_PROPERTY(QVariantList availableBaudrates READ availableBaudrates CONSTANT)

public:
  explicit AppState(QObject *parent = nullptr);

  // Mode control
  bool isSimulationMode() const { return m_isSimulationMode; }
  void setIsSimulationMode(bool value);
  bool showGizmo() const { return m_showGizmo; }
  void setShowGizmo(bool value);

  // IK control
  bool ikEnabled() const { return m_ikEnabled; }
  void setIkEnabled(bool value);
  bool ikSolving() const { return m_ikSolving; }
  void setIkSolving(bool value);
  QString ikStatusText() const { return m_ikStatusText; }
  void setIkStatusText(const QString &value);
  QVector3D lastValidGizmoPosition() const { return m_lastValidGizmoPosition; }
  void setLastValidGizmoPosition(const QVector3D &value);
  QVariantList ikJointNames() const { return m_ikJointNames; }

  // Cartesian control
  QVariantList cartesianStepSizes() const { return m_cartesianStepSizes; }
  int cartesianStepIndex() const { return m_cartesianStepIndex; }
  void setCartesianStepIndex(int value);
  double cartesianStepSize() const;
  QVariantList rotationStepSizes() const { return m_rotationStepSizes; }
  int rotationStepIndex() const { return m_rotationStepIndex; }
  void setRotationStepIndex(int value);
  double rotationStepSize() const;
  bool useLocalFrame() const { return m_useLocalFrame; }
  void setUseLocalFrame(bool value);

  // Joint state
  QVariantMap jointAngles() const { return m_jointAngles; }
  void setJointAngles(const QVariantMap &value);
  QVariantList currentJointPositions() const { return m_currentJointPositions; }
  void setCurrentJointPositions(const QVariantList &value);
  QVariantList simulationJointPositions() const {
    return m_simulationJointPositions;
  }
  void setSimulationJointPositions(const QVariantList &value);
  QVariantList jointColors() const { return m_jointColors; }

  // Graph state
  int maxDataPoints() const { return m_maxDataPoints; }
  int dataUpdateTrigger() const { return m_dataUpdateTrigger; }
  double graphTimeZoom() const { return m_graphTimeZoom; }
  void setGraphTimeZoom(double value);
  double graphTimeOffset() const { return m_graphTimeOffset; }
  void setGraphTimeOffset(double value);
  QVariantList selectedJoints() const { return m_selectedJoints; }
  void setSelectedJoints(const QVariantList &value);
  bool useRadians() const { return m_useRadians; }
  void setUseRadians(bool value);

  // Collision state
  QVariantList collidingPairs() const { return m_collidingPairs; }
  void setCollidingPairs(const QVariantList &value);
  QVariantMap collidingLinks() const { return m_collidingLinks; }
  void setCollidingLinks(const QVariantMap &value);
  QVariantList collisionColors() const { return m_collisionColors; }

  // Camera state
  double cameraDistance() const { return m_cameraDistance; }
  void setCameraDistance(double value);
  double panSpeed() const { return m_panSpeed; }
  void setPanSpeed(double value);
  double orbitSpeed() const { return m_orbitSpeed; }
  void setOrbitSpeed(double value);
  double zoomSpeed() const { return m_zoomSpeed; }
  void setZoomSpeed(double value);

  // Settings
  QString selectedDevice() const { return m_selectedDevice; }
  void setSelectedDevice(const QString &value);
  int selectedBaudrate() const { return m_selectedBaudrate; }
  void setSelectedBaudrate(int value);
  QVariantList availableBaudrates() const { return m_availableBaudrates; }

  // Helper functions
  Q_INVOKABLE double getJointValueByIndex(int index) const;
  Q_INVOKABLE QString getCollisionColor(const QString &linkName) const;
  Q_INVOKABLE void triggerDataUpdate();

signals:
  void isSimulationModeChanged();
  void showGizmoChanged();
  void ikEnabledChanged();
  void ikSolvingChanged();
  void ikStatusTextChanged();
  void lastValidGizmoPositionChanged();
  void cartesianStepIndexChanged();
  void rotationStepIndexChanged();
  void useLocalFrameChanged();
  void jointAnglesChanged();
  void currentJointPositionsChanged();
  void simulationJointPositionsChanged();
  void dataUpdateTriggerChanged();
  void graphTimeZoomChanged();
  void graphTimeOffsetChanged();
  void selectedJointsChanged();
  void useRadiansChanged();
  void collidingPairsChanged();
  void collidingLinksChanged();
  void cameraDistanceChanged();
  void panSpeedChanged();
  void orbitSpeedChanged();
  void zoomSpeedChanged();
  void selectedDeviceChanged();
  void selectedBaudrateChanged();

private:
  // Mode control
  bool m_isSimulationMode = false;
  bool m_showGizmo = true;

  // IK control
  bool m_ikEnabled = true;
  bool m_ikSolving = false;
  QString m_ikStatusText;
  QVector3D m_lastValidGizmoPosition;
  QVariantList m_ikJointNames;

  // Cartesian control
  QVariantList m_cartesianStepSizes;
  int m_cartesianStepIndex = 0;
  QVariantList m_rotationStepSizes;
  int m_rotationStepIndex = 0;
  bool m_useLocalFrame = false;

  // Joint state
  QVariantMap m_jointAngles;
  QVariantList m_currentJointPositions;
  QVariantList m_simulationJointPositions;
  QVariantList m_jointColors;

  // Graph state
  int m_maxDataPoints = 2000;
  int m_dataUpdateTrigger = 0;
  double m_graphTimeZoom = 1.0;
  double m_graphTimeOffset = 1.0;
  QVariantList m_selectedJoints;
  bool m_useRadians = true;

  // Collision state
  QVariantList m_collidingPairs;
  QVariantMap m_collidingLinks;
  QVariantList m_collisionColors;

  // Camera state
  double m_cameraDistance = 200.0;
  double m_panSpeed = 0.2;
  double m_orbitSpeed = 1.0;
  double m_zoomSpeed = 20.0;

  // Settings
  QString m_selectedDevice;
  int m_selectedBaudrate = 115200;
  QVariantList m_availableBaudrates;
};

#endif // APPSTATE_H

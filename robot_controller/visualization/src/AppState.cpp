#include "AppState.h"

AppState::AppState(QObject *parent) : QObject(parent) {
  // Initialize IK joint names
  m_ikJointNames = QVariantList{"L1", "L2", "L3", "L4", "L5", "L6"};

  // Initialize step sizes
  m_cartesianStepSizes = QVariantList{0.0005, 0.001, 0.005, 0.010}; // meters
  m_rotationStepSizes = QVariantList{0.5, 1.0, 5.0, 10.0};          // degrees

  // Initialize joint positions
  m_currentJointPositions = QVariantList{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  m_simulationJointPositions = QVariantList{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Initialize joint colors
  m_jointColors = QVariantList{"#FF5252", "#4CAF50", "#2196F3", "#FF9800",
                               "#9C27B0", "#00BCD4", "#FFEB3B"};

  // Initialize joint selection (all off)
  m_selectedJoints =
      QVariantList{false, false, false, false, false, false, false};

  // Initialize collision colors
  m_collisionColors = QVariantList{"#FF0000", "#FF8800", "#FFFF00",
                                   "#FF00FF", "#00FFFF", "#8800FF"};

  // Initialize baudrates
  m_availableBaudrates =
      QVariantList{9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
}

// Mode control
void AppState::setIsSimulationMode(bool value) {
  if (m_isSimulationMode != value) {
    m_isSimulationMode = value;
    emit isSimulationModeChanged();
  }
}

void AppState::setShowGizmo(bool value) {
  if (m_showGizmo != value) {
    m_showGizmo = value;
    emit showGizmoChanged();
  }
}

// IK control
void AppState::setIkEnabled(bool value) {
  if (m_ikEnabled != value) {
    m_ikEnabled = value;
    emit ikEnabledChanged();
  }
}

void AppState::setIkSolving(bool value) {
  if (m_ikSolving != value) {
    m_ikSolving = value;
    emit ikSolvingChanged();
  }
}

void AppState::setIkStatusText(const QString &value) {
  if (m_ikStatusText != value) {
    m_ikStatusText = value;
    emit ikStatusTextChanged();
  }
}

void AppState::setLastValidGizmoPosition(const QVector3D &value) {
  if (m_lastValidGizmoPosition != value) {
    m_lastValidGizmoPosition = value;
    emit lastValidGizmoPositionChanged();
  }
}

// Cartesian control
void AppState::setCartesianStepIndex(int value) {
  if (value >= 0 && value < m_cartesianStepSizes.size() &&
      m_cartesianStepIndex != value) {
    m_cartesianStepIndex = value;
    emit cartesianStepIndexChanged();
  }
}

double AppState::cartesianStepSize() const {
  if (m_cartesianStepIndex >= 0 &&
      m_cartesianStepIndex < m_cartesianStepSizes.size()) {
    return m_cartesianStepSizes[m_cartesianStepIndex].toDouble();
  }
  return 0.001; // default 1mm
}

void AppState::setRotationStepIndex(int value) {
  if (value >= 0 && value < m_rotationStepSizes.size() &&
      m_rotationStepIndex != value) {
    m_rotationStepIndex = value;
    emit rotationStepIndexChanged();
  }
}

double AppState::rotationStepSize() const {
  if (m_rotationStepIndex >= 0 &&
      m_rotationStepIndex < m_rotationStepSizes.size()) {
    return m_rotationStepSizes[m_rotationStepIndex].toDouble();
  }
  return 1.0; // default 1 degree
}

void AppState::setUseLocalFrame(bool value) {
  if (m_useLocalFrame != value) {
    m_useLocalFrame = value;
    emit useLocalFrameChanged();
  }
}

// Joint state
void AppState::setJointAngles(const QVariantMap &value) {
  m_jointAngles = value;
  emit jointAnglesChanged();
}

void AppState::setCurrentJointPositions(const QVariantList &value) {
  m_currentJointPositions = value;
  emit currentJointPositionsChanged();
}

void AppState::setSimulationJointPositions(const QVariantList &value) {
  m_simulationJointPositions = value;
  emit simulationJointPositionsChanged();
}

// Graph state
void AppState::setGraphTimeZoom(double value) {
  if (m_graphTimeZoom != value) {
    m_graphTimeZoom = value;
    emit graphTimeZoomChanged();
  }
}

void AppState::setGraphTimeOffset(double value) {
  if (m_graphTimeOffset != value) {
    m_graphTimeOffset = value;
    emit graphTimeOffsetChanged();
  }
}

void AppState::setSelectedJoints(const QVariantList &value) {
  m_selectedJoints = value;
  emit selectedJointsChanged();
}

void AppState::setUseRadians(bool value) {
  if (m_useRadians != value) {
    m_useRadians = value;
    emit useRadiansChanged();
  }
}

// Collision state
void AppState::setCollidingPairs(const QVariantList &value) {
  m_collidingPairs = value;
  emit collidingPairsChanged();
}

void AppState::setCollidingLinks(const QVariantMap &value) {
  m_collidingLinks = value;
  emit collidingLinksChanged();
}

// Camera state
void AppState::setCameraDistance(double value) {
  if (m_cameraDistance != value) {
    m_cameraDistance = value;
    emit cameraDistanceChanged();
  }
}

void AppState::setPanSpeed(double value) {
  if (m_panSpeed != value) {
    m_panSpeed = value;
    emit panSpeedChanged();
  }
}

void AppState::setOrbitSpeed(double value) {
  if (m_orbitSpeed != value) {
    m_orbitSpeed = value;
    emit orbitSpeedChanged();
  }
}

void AppState::setZoomSpeed(double value) {
  if (m_zoomSpeed != value) {
    m_zoomSpeed = value;
    emit zoomSpeedChanged();
  }
}

// Settings
void AppState::setSelectedDevice(const QString &value) {
  if (m_selectedDevice != value) {
    m_selectedDevice = value;
    emit selectedDeviceChanged();
  }
}

void AppState::setSelectedBaudrate(int value) {
  if (m_selectedBaudrate != value) {
    m_selectedBaudrate = value;
    emit selectedBaudrateChanged();
  }
}

// Helper functions
double AppState::getJointValueByIndex(int index) const {
  if (m_isSimulationMode) {
    // In simulation mode, use jointAngles map
    if (index < m_ikJointNames.size()) {
      QString jointName = m_ikJointNames[index].toString();
      return m_jointAngles.value(jointName, 0.0).toDouble();
    } else if (index == 6) {
      return m_jointAngles.value("left_finger_joint", 0.0).toDouble();
    }
    return 0.0;
  } else {
    // In robot mode, use currentJointPositions from robot
    return (index < m_currentJointPositions.size())
               ? m_currentJointPositions[index].toDouble()
               : 0.0;
  }
}

QString AppState::getCollisionColor(const QString &linkName) const {
  if (m_collidingLinks.contains(linkName)) {
    int colorIdx = m_collidingLinks.value(linkName).toInt();
    int idx = colorIdx % m_collisionColors.size();
    return m_collisionColors[idx].toString();
  }
  return QString(); // empty string = not colliding
}

void AppState::triggerDataUpdate() {
  m_dataUpdateTrigger++;
  emit dataUpdateTriggerChanged();
}

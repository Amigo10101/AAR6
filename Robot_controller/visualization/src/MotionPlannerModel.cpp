#include "MotionPlannerModel.h"
#include <QDebug>

WaypointListModel::WaypointListModel(QObject *parent)
    : QAbstractListModel(parent) {}

int WaypointListModel::rowCount(const QModelIndex &parent) const {
  if (parent.isValid())
    return 0;
  return static_cast<int>(m_waypoints.size());
}

QVariant WaypointListModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid() || index.row() < 0 || index.row() >= m_waypoints.size())
    return QVariant();

  const Waypoint &wp = m_waypoints[index.row()];

  switch (role) {
  case NameRole:
    return wp.name;
  case PositionXRole:
    return wp.position.x();
  case PositionYRole:
    return wp.position.y();
  case PositionZRole:
    return wp.position.z();
  case OrientationWRole:
    return wp.orientation.scalar();
  case OrientationXRole:
    return wp.orientation.x();
  case OrientationYRole:
    return wp.orientation.y();
  case OrientationZRole:
    return wp.orientation.z();
  case RollRole: {
    // Convert quaternion to Euler angles (Roll around X)
    QVector3D euler = wp.orientation.toEulerAngles();
    return euler.x(); // Roll in degrees
  }
  case PitchRole: {
    QVector3D euler = wp.orientation.toEulerAngles();
    return euler.y(); // Pitch in degrees
  }
  case YawRole: {
    QVector3D euler = wp.orientation.toEulerAngles();
    return euler.z(); // Yaw in degrees
  }
  case MotionTypeRole:
    return static_cast<int>(wp.motionType);
  case VelocityRole:
    return wp.velocity;
  case BlendRadiusRole:
    return wp.blendRadius;
  case IndexRole:
    return index.row();
  default:
    return QVariant();
  }
}

bool WaypointListModel::setData(const QModelIndex &index, const QVariant &value,
                                int role) {
  if (!index.isValid() || index.row() < 0 || index.row() >= m_waypoints.size())
    return false;

  Waypoint &wp = m_waypoints[index.row()];
  bool changed = false;

  switch (role) {
  case NameRole:
    if (wp.name != value.toString()) {
      wp.name = value.toString();
      changed = true;
    }
    break;
  case MotionTypeRole:
    if (static_cast<int>(wp.motionType) != value.toInt()) {
      wp.motionType = static_cast<MotionType>(value.toInt());
      changed = true;
    }
    break;
  case VelocityRole:
    if (wp.velocity != value.toDouble()) {
      wp.velocity = value.toDouble();
      changed = true;
    }
    break;
  default:
    return false;
  }

  if (changed) {
    emit dataChanged(index, index, {role});
    emit waypointsChanged();
  }
  return changed;
}

QHash<int, QByteArray> WaypointListModel::roleNames() const {
  return {{NameRole, "name"},
          {PositionXRole, "posX"},
          {PositionYRole, "posY"},
          {PositionZRole, "posZ"},
          {OrientationWRole, "orientW"},
          {OrientationXRole, "orientX"},
          {OrientationYRole, "orientY"},
          {OrientationZRole, "orientZ"},
          {RollRole, "roll"},
          {PitchRole, "pitch"},
          {YawRole, "yaw"},
          {MotionTypeRole, "motionType"},
          {VelocityRole, "velocity"},
          {BlendRadiusRole, "blendRadius"},
          {IndexRole, "waypointIndex"}};
}

Qt::ItemFlags WaypointListModel::flags(const QModelIndex &index) const {
  if (!index.isValid())
    return Qt::NoItemFlags;
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsEditable;
}

void WaypointListModel::addWaypoint(const QString &name, double x, double y,
                                    double z, double qw, double qx, double qy,
                                    double qz, int motionType) {
  const int newRow = static_cast<int>(m_waypoints.size());
  beginInsertRows(QModelIndex(), newRow, newRow);

  Waypoint wp;
  wp.name = name;
  wp.position = QVector3D(static_cast<float>(x), static_cast<float>(y),
                          static_cast<float>(z));
  wp.orientation = QQuaternion(static_cast<float>(qw), static_cast<float>(qx),
                               static_cast<float>(qy), static_cast<float>(qz));
  wp.motionType = static_cast<MotionType>(motionType);
  wp.velocity = 1.0;
  wp.blendRadius = 0.0;
  wp.hasAuxPoint = false;

  m_waypoints.append(wp);

  endInsertRows();

  qDebug() << "Added waypoint:" << name << "at" << wp.position
           << "type:" << motionType;

  emit countChanged();
  emit waypointsChanged();
}

void WaypointListModel::removeWaypoint(int index) {
  if (index < 0 || index >= m_waypoints.size())
    return;

  beginRemoveRows(QModelIndex(), index, index);
  m_waypoints.removeAt(index);
  endRemoveRows();

  // Adjust selection if needed
  if (m_selectedIndex >= m_waypoints.size()) {
    setSelectedIndex(m_waypoints.isEmpty() ? -1 : m_waypoints.size() - 1);
  } else if (m_selectedIndex == index) {
    // Selected item was removed
    setSelectedIndex(
        m_waypoints.isEmpty() ? -1 : qMin(index, m_waypoints.size() - 1));
  }

  emit countChanged();
  emit waypointsChanged();
}

void WaypointListModel::moveWaypoint(int from, int to) {
  if (from < 0 || from >= m_waypoints.size() || to < 0 ||
      to >= m_waypoints.size() || from == to)
    return;

  // Qt's beginMoveRows has specific requirements
  int destRow = to > from ? to + 1 : to;
  if (!beginMoveRows(QModelIndex(), from, from, QModelIndex(), destRow))
    return;

  m_waypoints.move(from, to);
  endMoveRows();

  // Update selection to follow the moved item
  if (m_selectedIndex == from) {
    m_selectedIndex = to;
    emit selectedIndexChanged();
  }

  emit waypointsChanged();
}

void WaypointListModel::updateWaypointPosition(int index, double x, double y,
                                               double z) {
  if (index < 0 || index >= m_waypoints.size())
    return;

  m_waypoints[index].position = QVector3D(
      static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));

  QModelIndex modelIndex = createIndex(index, 0);
  emit dataChanged(modelIndex, modelIndex,
                   {PositionXRole, PositionYRole, PositionZRole});
  emit waypointsChanged();
}

void WaypointListModel::updateWaypointOrientation(int index, double qw,
                                                  double qx, double qy,
                                                  double qz) {
  if (index < 0 || index >= m_waypoints.size())
    return;

  m_waypoints[index].orientation =
      QQuaternion(static_cast<float>(qw), static_cast<float>(qx),
                  static_cast<float>(qy), static_cast<float>(qz));

  QModelIndex modelIndex = createIndex(index, 0);
  emit dataChanged(
      modelIndex, modelIndex,
      {OrientationWRole, OrientationXRole, OrientationYRole, OrientationZRole});
  emit waypointsChanged();
}

void WaypointListModel::updateWaypointMotionType(int index, int motionType) {
  if (index < 0 || index >= m_waypoints.size())
    return;

  if (motionType < 0 || motionType > 2)
    return;

  m_waypoints[index].motionType = static_cast<MotionType>(motionType);

  QModelIndex modelIndex = createIndex(index, 0);
  emit dataChanged(modelIndex, modelIndex, {MotionTypeRole});
  emit waypointsChanged();
}

void WaypointListModel::updateWaypointName(int index, const QString &name) {
  if (index < 0 || index >= m_waypoints.size())
    return;

  m_waypoints[index].name = name;

  QModelIndex modelIndex = createIndex(index, 0);
  emit dataChanged(modelIndex, modelIndex, {NameRole});
  emit waypointsChanged();
}

void WaypointListModel::clearAll() {
  if (m_waypoints.isEmpty())
    return;

  beginResetModel();
  m_waypoints.clear();
  endResetModel();

  setSelectedIndex(-1);
  emit countChanged();
  emit waypointsChanged();
}

QVariantMap WaypointListModel::getWaypoint(int index) const {
  QVariantMap result;
  if (index < 0 || index >= m_waypoints.size())
    return result;

  const Waypoint &wp = m_waypoints[index];
  result["name"] = wp.name;
  result["x"] = wp.position.x();
  result["y"] = wp.position.y();
  result["z"] = wp.position.z();
  result["qw"] = wp.orientation.scalar();
  result["qx"] = wp.orientation.x();
  result["qy"] = wp.orientation.y();
  result["qz"] = wp.orientation.z();
  result["motionType"] = static_cast<int>(wp.motionType);
  result["velocity"] = wp.velocity;
  result["blendRadius"] = wp.blendRadius;

  return result;
}

QVariantList WaypointListModel::getWaypointsForVisualization() const {
  QVariantList result;
  for (int i = 0; i < m_waypoints.size(); ++i) {
    const Waypoint &wp = m_waypoints[i];
    QVariantMap map;
    map["index"] = i;
    map["name"] = wp.name;
    map["x"] = wp.position.x();
    map["y"] = wp.position.y();
    map["z"] = wp.position.z();
    map["motionType"] = static_cast<int>(wp.motionType);
    map["isSelected"] = (i == m_selectedIndex);
    result.append(map);
  }
  return result;
}

void WaypointListModel::setSelectedIndex(int index) {
  if (index < -1)
    index = -1;
  if (index >= m_waypoints.size())
    index = m_waypoints.size() - 1;

  if (m_selectedIndex != index) {
    m_selectedIndex = index;
    emit selectedIndexChanged();
    emit
    waypointsChanged(); // Trigger visualization update for selection highlight
  }
}

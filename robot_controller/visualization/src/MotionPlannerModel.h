#ifndef MOTIONPLANNERMODEL_H
#define MOTIONPLANNERMODEL_H

#include <QAbstractListModel>
#include <QObject>
#include <QQuaternion>
#include <QVariantList>
#include <QVector3D>

// Motion types for robot movement
enum class MotionType {
  PTP = 0, // Point-to-Point (joint interpolation)
  LIN = 1, // Linear (Cartesian interpolation)
  CIRC = 2 // Circular (arc motion)
};

// Single waypoint data structure
struct Waypoint {
  QString name;             // User-friendly name
  QVector3D position;       // XYZ position in meters (URDF frame)
  QQuaternion orientation;  // Orientation quaternion
  MotionType motionType;    // How to move TO this point
  double velocity = 1.0;    // Velocity scale (0.0 - 1.0)
  double blendRadius = 0.0; // Blend radius for smooth transitions (meters)

  // For CIRC motion, optional auxiliary point
  bool hasAuxPoint = false;
  QVector3D auxPoint; // Via-point for circular motion
};

/**
 * @brief WaypointListModel - Single source of truth for motion planner
 * waypoints
 *
 * This model uses QAbstractListModel for efficient ListView integration in QML.
 * All waypoint data is stored here and accessed via role-based data binding.
 */
class WaypointListModel : public QAbstractListModel {
  Q_OBJECT
  Q_PROPERTY(int count READ count NOTIFY countChanged)
  Q_PROPERTY(int selectedIndex READ selectedIndex WRITE setSelectedIndex NOTIFY
                 selectedIndexChanged)

public:
  // Roles for QML data binding
  enum WaypointRoles {
    NameRole = Qt::UserRole + 1,
    PositionXRole,
    PositionYRole,
    PositionZRole,
    OrientationWRole,
    OrientationXRole,
    OrientationYRole,
    OrientationZRole,
    RollRole,
    PitchRole,
    YawRole,
    MotionTypeRole,
    VelocityRole,
    BlendRadiusRole,
    IndexRole
  };

  explicit WaypointListModel(QObject *parent = nullptr);
  ~WaypointListModel() override = default;

  // QAbstractListModel interface
  int rowCount(const QModelIndex &parent = QModelIndex()) const override;
  QVariant data(const QModelIndex &index,
                int role = Qt::DisplayRole) const override;
  bool setData(const QModelIndex &index, const QVariant &value,
               int role) override;
  QHash<int, QByteArray> roleNames() const override;
  Qt::ItemFlags flags(const QModelIndex &index) const override;

  // Q_INVOKABLE methods for QML
  Q_INVOKABLE void addWaypoint(const QString &name, double x, double y,
                               double z, double qw, double qx, double qy,
                               double qz, int motionType);
  Q_INVOKABLE void removeWaypoint(int index);
  Q_INVOKABLE void moveWaypoint(int from, int to);
  Q_INVOKABLE void updateWaypointPosition(int index, double x, double y,
                                          double z);
  Q_INVOKABLE void updateWaypointOrientation(int index, double qw, double qx,
                                             double qy, double qz);
  Q_INVOKABLE void updateWaypointMotionType(int index, int motionType);
  Q_INVOKABLE void updateWaypointName(int index, const QString &name);
  Q_INVOKABLE void clearAll();
  Q_INVOKABLE QVariantMap getWaypoint(int index) const;

  // For 3D visualization - returns list suitable for Repeater3D
  Q_INVOKABLE QVariantList getWaypointsForVisualization() const;

  // Property accessors
  int count() const { return static_cast<int>(m_waypoints.size()); }
  int selectedIndex() const { return m_selectedIndex; }
  void setSelectedIndex(int index);

signals:
  void countChanged();
  void selectedIndexChanged();
  void waypointsChanged(); // Emitted when any waypoint data changes

private:
  QVector<Waypoint> m_waypoints;
  int m_selectedIndex = -1;
};

#endif // MOTIONPLANNERMODEL_H

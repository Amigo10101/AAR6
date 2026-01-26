find_package(Qt6 REQUIRED COMPONENTS Core Gui Quick Quick3D SerialPort Qml)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FCL REQUIRED fcl)
set(QT_QML_GENERATE_QMLLS_INI ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

qt_add_executable(visualization
    src/main.cpp
    src/RobotNode.cpp
    src/RobotNode.h
    src/robot_serial.cpp
    src/robot_serial.h
    src/RobotManager.cpp
    src/RobotManager.h
    src/CollisionWrapper.cpp
    src/CollisionWrapper.h
    src/MotionPlannerModel.cpp
    src/MotionPlannerModel.h
    src/MotionController.cpp
    src/MotionController.h
    src/AppState.cpp
    src/AppState.h
)
target_include_directories(visualization PRIVATE
    src
    ${CMAKE_SOURCE_DIR}/collision_checker/include
    ${CMAKE_SOURCE_DIR}/kinematics/include
    ${CMAKE_SOURCE_DIR}/robot_model/include
    ${EIGEN3_INCLUDE_DIRS}
    ${FCL_INCLUDE_DIRS}
)

target_link_libraries(visualization PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Quick
    Qt6::Quick3D
    Qt6::SerialPort
    urdfdom::urdfdom_model
    Qt6::Qml
    collision_checker
    kinematics
    robot_model
)

qt_add_qml_module(visualization
    URI Visualization
    VERSION 1.0
    QML_FILES
        qml/main.qml
        qml/panels/JointControlPanel.qml
        qml/panels/MotionPlannerPanel.qml
        qml/tabs/SettingsTab.qml
        qml/tabs/DriverSettingsTab.qml
        qml/components/JointGraph.qml
        qml/components/GlassPanel.qml
        qml/components/PanelHeader.qml
        qml/components/StyledButton.qml
        qml/components/ValueDisplay.qml
        qml/view3d/WaypointMarkers.qml
)


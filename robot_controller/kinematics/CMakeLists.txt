project(kinematics)

# Find dependencies
find_package(urdfdom REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# Include directories
include_directories(
    include
    ${CMAKE_SOURCE_DIR}/robot_model/include
    ${CMAKE_SOURCE_DIR}/urdfdom/urdf_parser/include
    ${CMAKE_SOURCE_DIR}/urdfdom_headers/urdf_model/include
    ${CMAKE_SOURCE_DIR}/urdfdom_headers/urdf_exception/include
    ${CMAKE_SOURCE_DIR}/external/EAIK/CPP/src
    ${EIGEN3_INCLUDE_DIRS}
)

# Add EAIK library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(${CMAKE_SOURCE_DIR}/external/EAIK/CPP/src build_EAIK)

add_library(kinematics SHARED
    src/ForwardKinematics.cpp
    src/InverseKinematics.cpp
    src/PositionIK.cpp
    src/SafetyLimiter.cpp
    src/SqpIK.cpp
    src/TrajectoryGenerator.cpp
    src/VelocityIK.cpp
)

target_include_directories(kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(kinematics PUBLIC
    robot_model
    urdfdom::urdfdom_sensor
    urdfdom::urdfdom_model
    urdfdom::urdfdom_world
    EAIK
    OsqpEigen::OsqpEigen
)

cmake_minimum_required(VERSION 3.5)
project(urdf_standalone_project)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(QT_QML_GENERATE_QMLLS_INI ON)

# Find dependencies
# Assumes urdfdom is installed in ./install
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/install")
find_package(urdfdom REQUIRED)

# Add subdirectories
add_subdirectory(robot_model)
add_subdirectory(kinematics)
add_subdirectory(collision_checker)
add_subdirectory(control)
add_subdirectory(visualization)
add_subdirectory(examples)

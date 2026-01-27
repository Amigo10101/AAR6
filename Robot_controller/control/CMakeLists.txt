cmake_minimum_required(VERSION 3.10)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Control module for real-time velocity control
add_library(control STATIC
    src/RealTimeController.cpp
)

target_include_directories(control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(control
    PUBLIC
        kinematics
        collision_checker
        robot_model
        pthread
)

# Set C++ standard
set_target_properties(control PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)


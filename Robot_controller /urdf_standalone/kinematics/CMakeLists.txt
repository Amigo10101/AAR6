project(kinematics)

# Find dependencies
find_package(urdfdom REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FCL REQUIRED fcl)

# Include directories
include_directories(
    include
    ${CMAKE_SOURCE_DIR}/urdfdom/urdf_parser/include
    ${CMAKE_SOURCE_DIR}/urdfdom_headers/urdf_model/include
    ${CMAKE_SOURCE_DIR}/urdfdom_headers/urdf_exception/include
    ${EIGEN3_INCLUDE_DIRS}
    ${FCL_INCLUDE_DIRS}
)

add_library(kinematics SHARED
    src/ForwardKinematics.cpp
)

target_include_directories(kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(kinematics PUBLIC
    urdfdom::urdfdom_sensor
    urdfdom::urdfdom_model
    urdfdom::urdfdom_world
    ${FCL_LIBRARIES}
)

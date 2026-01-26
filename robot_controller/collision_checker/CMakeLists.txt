cmake_minimum_required(VERSION 3.10)

project(collision_checker)

# Find required packages
find_package(urdfdom REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FCL REQUIRED fcl)
find_package(assimp REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    include
    ${CMAKE_SOURCE_DIR}/urdfdom/urdf_parser/include
    ${CMAKE_SOURCE_DIR}/urdfdom_headers/urdf_model/include
    ${CMAKE_SOURCE_DIR}/urdfdom_headers/urdf_exception/include
    ${CMAKE_SOURCE_DIR}/kinematics/include
    ${FCL_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Collision checker library
add_library(collision_checker
    src/CollisionChecker.cpp
    src/GroundTruthGenerator.cpp
)

target_link_libraries(collision_checker
    urdfdom_model
    kinematics
    ${FCL_LIBRARIES}
    ${ASSIMP_LIBRARIES}
)

target_include_directories(collision_checker PUBLIC
    include
    ${CMAKE_SOURCE_DIR}/kinematics/include
)

target_include_directories(collision_checker PRIVATE
    ${FCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${ASSIMP_INCLUDE_DIRS}
)

target_compile_options(collision_checker PRIVATE
    ${FCL_CFLAGS_OTHER}
)

# Example executable
add_executable(test_collision_checker
    examples/test_collision_checker.cpp
)

target_link_libraries(test_collision_checker
    collision_checker
    urdfdom_model
    ${FCL_LIBRARIES}
)

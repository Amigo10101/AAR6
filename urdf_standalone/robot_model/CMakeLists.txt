add_library(robot_model SHARED
    src/RobotModel.cpp
)

target_include_directories(robot_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(robot_model PUBLIC
    urdfdom::urdfdom_sensor
    urdfdom::urdfdom_model
    urdfdom::urdfdom_world
)

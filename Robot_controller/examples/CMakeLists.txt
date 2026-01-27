add_executable(urdf_example main.cpp)

target_link_libraries(urdf_example PRIVATE
    robot_model
)

find_package(Eigen3 REQUIRED)

add_executable(fk_example fk_example.cpp)
target_include_directories(fk_example PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(fk_example PRIVATE
    robot_model
    kinematics
)

add_executable(ik_example ik_example.cpp)
target_include_directories(ik_example PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(ik_example PRIVATE
    robot_model
    kinematics
)

add_executable(ik_convergence_test ik_convergence_test.cpp)
target_include_directories(ik_convergence_test PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(ik_convergence_test PRIVATE
    robot_model
    kinematics
)

add_executable(fk_comparison_test fk_comparison_test.cpp)
target_include_directories(fk_comparison_test PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(fk_comparison_test PRIVATE
    robot_model
    kinematics
)

add_executable(dh_visual_test dh_visual_test.cpp)
target_include_directories(dh_visual_test PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(dh_visual_test PRIVATE
    robot_model
    kinematics
)

add_executable(extract_link_positions extract_link_positions.cpp)
target_include_directories(extract_link_positions PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(extract_link_positions PRIVATE
    robot_model
    kinematics
)



add_executable(test_eaik_solver test_eaik_solver.cpp)
target_include_directories(test_eaik_solver PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(test_eaik_solver PRIVATE
    robot_model
    kinematics
)

add_executable(test_velocity_controller test_velocity_controller.cpp)
target_include_directories(test_velocity_controller PRIVATE 
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(test_velocity_controller PRIVATE
    robot_model
    kinematics
    collision_checker
    control
    pthread
)

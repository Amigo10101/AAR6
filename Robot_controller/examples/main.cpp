#include "robot_model/RobotModel.h"
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: urdf_example <urdf_file>" << std::endl;
        return 1;
    }

    std::string urdf_file = argv[1];
    robot_model::RobotModel robot;

    if (robot.loadURDF(urdf_file)) {
        std::cout << "Successfully loaded URDF!" << std::endl;
        robot.printInfo();
    } else {
        return -1;
    }

    return 0;
}

#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>

namespace kinematics {

enum class MovementType {
    PTP,  // Point-to-Point (Joint Space)
    LIN,  // Linear (Cartesian Space)
    CIRC  // Circular (Cartesian Space)
};

struct TrajectoryPoint {
    double time;
    Eigen::VectorXd position; // Joint positions or Cartesian pose (vectorized)
    Eigen::VectorXd velocity; // Joint velocities or Cartesian twist
    Eigen::VectorXd acceleration; 
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();

    /**
     * @brief Generate a trajectory between two Cartesin poses.
     * @param start_pose Start pose in Cartesian space
     * @param end_pose End pose in Cartesian space
     * @param type Movement type (LIN, CIRC, PTP)
     * @param max_velocity Maximum velocity scaling factor (0.0 to 1.0)
     * @param dt Time step for generation
     * @return Vector of trajectory points containing velocity commands
     */
    std::vector<TrajectoryPoint> generateTrajectory(
        const Eigen::Isometry3d& start_pose,
        const Eigen::Isometry3d& end_pose,
        MovementType type,
        double max_velocity,
        double dt,
        const Eigen::VectorXd& start_joints, // Optional: helpful for PTP if we have IK
        const Eigen::VectorXd& end_joints    // Optional: helpful for PTP
    );

private:
    std::vector<TrajectoryPoint> generatePTP(
        const Eigen::VectorXd& start_joints,
        const Eigen::VectorXd& end_joints,
        double max_velocity,
        double dt
    );

    std::vector<TrajectoryPoint> generateLIN(
        const Eigen::Isometry3d& start_pose,
        const Eigen::Isometry3d& end_pose,
        double max_velocity,
        double dt
    );

    std::vector<TrajectoryPoint> generateCIRC(
        const Eigen::Isometry3d& start_pose,
        const Eigen::Isometry3d& end_pose,
        double max_velocity,
        double dt
    );
};

} // namespace kinematics

#endif // TRAJECTORY_GENERATOR_H

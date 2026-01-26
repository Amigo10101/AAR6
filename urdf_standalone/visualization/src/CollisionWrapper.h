// CollisionWrapper.h - Qt-free header for collision checking
#ifndef COLLISION_WRAPPER_H
#define COLLISION_WRAPPER_H

#include <string>
#include <vector>
#include <memory>

// Forward declaration - no FCL headers here
namespace collision_checker {
    class GroundTruthGenerator;
}

class CollisionWrapper {
public:
    CollisionWrapper();
    ~CollisionWrapper();
    
    bool initialize(const std::string& urdfPath);
    bool checkCollision(const std::vector<double>& jointAngles);
    std::vector<std::pair<std::string, std::string>> getCollidingPairs(const std::vector<double>& jointAngles);
    bool isInitialized() const { return initialized_; }

private:
    std::unique_ptr<collision_checker::GroundTruthGenerator> checker_;
    bool initialized_ = false;
};

#endif // COLLISION_WRAPPER_H

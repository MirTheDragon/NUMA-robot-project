#pragma once

#include <array>
#include <string>
#include <cstdint>
#include "RobotBody.hpp"
#include "ThreePointLeg.hpp"
#include "ServoController.hpp"

struct ServoMapping {
    uint8_t controllerIndex;
    uint8_t channelIndex;
};

class RobotController {
public:
    RobotController();

    int initialize();
    void updateAllIK();
    void updateServoAnglesFromIK();
    int applyAll();
    void updateKinematicsAndApply();

    void setFootTarget(size_t legIndex, const Vec3& target);
    ThreePointLeg& getLeg(size_t legIndex);
    ServoController& getServoController(size_t controllerIndex);



private:
    static constexpr size_t kNumLegs = 6;
    static constexpr size_t kJointsPerLeg = 3;
    static constexpr size_t kNumControllers = 2;
    static constexpr size_t kChannelsPerController = 16;

    
    RobotBody Body_;  // Added robot body instance

    std::array<ThreePointLeg, kNumLegs> legs_; // Array of legs, each leg is a ThreePointLeg instance
    std::array<ServoController, kNumControllers> servoControllers_; // Array of servo controllers
    std::array<std::array<ServoMapping, kJointsPerLeg>, kNumLegs> servoMappings_; // Mappings from legs to servo controllers and channels

    
    // New: store current desired foot positions in world coordinates
    std::array<Vec3, kNumLegs> footTargetsWorld_;

    // New: store last pushed foot positions (for direction/speed calculation)
    std::array<Vec3, kNumLegs> footTargetsLastPushedWorld_;

    void initServoMappingsAndConfigs(int LegsCount);
};

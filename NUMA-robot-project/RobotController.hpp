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

    static constexpr size_t kNumLegs = 6;
    size_t legCount_ = 6; // Number of legs, default to 6
    RobotController(int legCount);

    int initialize();
    void updateAllIK();
    void updateServoAnglesFromIK();
    int applyAll();
    void updateKinematicsAndApply();

    void setFootTarget(size_t legIndex, const Vec3& target);
    const ServoMapping& getServoMapping(size_t legIndex, size_t jointIndex);

    
    RobotBody Body;  // Added robot body instance
    std::array<ThreePointLeg, kNumLegs> legs; // Array of legs, each leg is a ThreePointLeg instance

    ThreePointLeg& getLeg(size_t legIndex);
    const ServoMapping& getServoMapping(size_t legIndex, size_t jointIndex) const;




private:
    static constexpr size_t kJointsPerLeg = 3;
    static constexpr size_t kNumControllers = 3;
    static constexpr size_t kChannelsPerController = 16;

    std::array<ServoController, kNumControllers> servoControllers_; // Array of servo controllers
    std::array<std::array<ServoMapping, kJointsPerLeg>, kNumLegs> servoMappings_; // Mappings from legs to servo controllers and channels

    
    // New: store current desired foot positions in world coordinates
    std::array<Vec3, kNumLegs> footTargetsWorld_;

    // New: store last pushed foot positions (for direction/speed calculation)
    std::array<Vec3, kNumLegs> footTargetsLastPushedWorld_;

    void initServoMappingsAndConfigs(int LegsCount);
};

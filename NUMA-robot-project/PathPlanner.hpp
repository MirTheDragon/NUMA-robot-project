#pragma once

#include <array>
#include <cmath>
#include "RobotController.hpp"

// Simple 2D vector for planar foot movement
struct Vec2 {
    float x = 0.f, y = 0.f;

    Vec2() = default;
    Vec2(float x_, float y_) : x(x_), y(y_) {}

    float length() const {
        return std::sqrt(x * x + y * y);
    }

    Vec2 normalized() const {
        float l = length();
        return l > 0 ? Vec2(x / l, y / l) : Vec2(0, 0);
    }

    Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    Vec2 operator*(float s) const {
        return Vec2(x * s, y * s);
    }

    void operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
    }
};

// Rotates a Vec2 vector by angleDegrees CCW around origin (0,0)
inline Vec2 rotateZ(const Vec2& vec, float angleDegrees) {
    const float rad = angleDegrees * 3.14159265358979323846f / 180.f;
    float cosA = std::cos(rad);
    float sinA = std::sin(rad);

    float xNew = vec.x * cosA - vec.y * sinA;
    float yNew = vec.x * sinA + vec.y * cosA;

    return Vec2{xNew, yNew};
}

// Foot states simplified: either on ground or lifting (in air)
enum class FootState {
    Grounded,
    Lifted
};

class WalkCycle;  // Forward declaration before PathPlanner class definition


class PathPlanner {
    struct FootStatusInternal {
        FootState state = FootState::Grounded;
        Vec3 currentPosition;   // Current position in 3D (world coords)
        Vec3 desiredTarget;   // Desired target in 3D (world coords)
        Vec2 stepAreaCenter;     // Center of allowed step area circle
        Vec2 stepAreaVector;   // Vector from step area center to foot target
        Vec2 stepAreaTarget;     // Target position in 2D (XY plane)
        Vec2 stepAreaSyncronizedTarget;     // Target position in 2D (XY plane)
        float stepProgress = 0.f;   // Progress along step path [0..1]
    };

public:
    static constexpr size_t kNumLegs = 6;  // fixed leg count
    
    // Pass robot controller reference in constructor
    PathPlanner(RobotController& robot);
    float stepAreaPlacementDistance = 34.f; // cm step center from robot body center
    

    // Max robot speed in cm/s (grounded feet move this fast)
    float maxRobotSpeedCmPerSec = 10.0f;
    float RobotSpeedCmPerSec = 0.0f; // Current speed based on joystick input

    float stepHeight = 4.0f; // Default step height in cm, adjustable on the fly
    bool clampFootTargets = true;  // Toggle clamping on/off

    // Update planner state with joystick input and elapsed time (seconds)
    void update(const Vec2& joystickInput, float deltaTimeSeconds);
    void requestWalkCycleSwitch(WalkCycle* newCycle) {
        nextWalkCycle_ = newCycle;
    }

    WalkCycle* currentWalkCycle_ = nullptr;  // Pointer to active walk cycle pattern
    WalkCycle* nextWalkCycle_ = nullptr;  // Pointer to active walk cycle pattern


    
    // Getter for foot status by leg index
    const FootStatusInternal& getFootStatus(size_t legIndex) const {
        return footStatuses_[legIndex];
    }

private:
    RobotController& robot_; // Robot controller must have 6 legs

    // Configurable parameters (can expose setters if needed)
    float stepAreaRadius_ = 10.f;    // cm max foot move radius from step center
    float stepSpeed_ = 5.f;      // cm/s speed foot moves on ground plane
    float stepDuration_ = 0.5f;  // seconds for full step cycle (lift + lower)

    std::array<FootStatusInternal, kNumLegs> footStatuses_;


    // Setter to change walk cycle at runtime
    void setCurrentWalkCycle(WalkCycle* walkCycle) {
        currentWalkCycle_ = walkCycle;
    }

    // Getter for convenience
    WalkCycle* getCurrentWalkCycle() const {
        return currentWalkCycle_;
    }


    // Calculate step area centers for each leg based on leg angle and robot heading
    void updateStepAreaCenter(size_t legIndex, float headingRad);

    // Update desired foot targets constrained within step area, driven by joystick input
    void updateStepAreaTargets(const Vec2& joystickInput, float headingRad);
    void updateFootTargets(const Vec2& joystickInput, float dt);

    // Update foot states and compute stepProgress and foot height smoothly:
    void updateFootStateTransitionsByGroup();
    void stepPathLogic(const Vec2& joystickInput, float dt);


    // Helper to compute vertical offset (height) based on stepProgress
    void computeFootHeights();

    // These functions operate on a WalkCycle instance
    void computeTimeToEdgePerGroup(WalkCycle& walkCycle, const Vec2& moveDir);
    void computeDistanceToBackEdgePerGroup(WalkCycle& walkCycle);
    void selectNextGroupToLift();
    float computeMaxDistanceToTargetInGroup(size_t groupIndex) const;
    float computeMaxForwardDistanceToStepAreaTarget() const;
    Vec2 getSynchronizedStepAreaVector(const WalkCycle& walkCycle) const;
    void updateSynchronizedStepAreaTargets();
    void selectNextGroupToLift(WalkCycle& walkCycle);
    void adjustSpeedToLegCapabilities(WalkCycle& walkCycle, const Vec2& moveDir);

    
    // Convert 3D foot target into robot controller foot targets (Vec3)
    void pushTargetsToRobot();
    
};


class WalkCycle {
public:
    WalkCycle(std::vector<std::vector<size_t>> groups, float liftedSpeedMultiplier,  float fractionAhead = 0.5f, float earlyLiftFraction = 0.20f)
        : legGroups_(std::move(groups)),
          liftedSpeedMultiplier_(liftedSpeedMultiplier),
          fractionAhead_(fractionAhead),
          earlyLiftFraction_(earlyLiftFraction)
    {
        timeToEdgePerGroup.resize(legGroups_.size(), 0.f);
        distToEdgePerGroup.resize(legGroups_.size(), 0.f);
    }

    // Array of leg groups, each group contains indices of legs
    std::vector<std::vector<size_t>> legGroups_;
    float liftedSpeedMultiplier_;
    float maxRobotSpeedCmPerSec = 10.f;  // max desired speed of grounded legs
    float fractionAhead_;  // fraction of step area radius for spacing
    float earlyLiftFraction_; // fraction of step duration to lift early
    float positionThreshold_ = 0.2f;  // threshold distance to switch foot states (cm)

    int walkDirection_ = 1;  // +1 or -1 to cycle through legGroups_
    size_t minDistToBackGroupIndex_ = 0;  // index of group with minimum distance to edge

    // Getters
    const std::vector<std::vector<size_t>>& getLegGroups() const { return legGroups_; }
    float maxLiftedLegSpeedCmPerSec() const { return maxRobotSpeedCmPerSec * liftedSpeedMultiplier_; }

    // Get legs in current group
    const std::vector<size_t>& getCurrentGroupLegs() const {
        return legGroups_[lastLiftedGroupIndex_];
    }

    const float getMinDistToBackEdge() const {
        return distToEdgePerGroup[minDistToBackGroupIndex_];
    }

    // Get time to edge for current group
    size_t lastLiftedGroupIndex_ = 0; // index of last lifted group
    size_t optimalLiftedGroupIndex_ = 0; // index of group to lift next
    std::vector<float> timeToEdgePerGroup;
    std::vector<float> distToEdgePerGroup;
};

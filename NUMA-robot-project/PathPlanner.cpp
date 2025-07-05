#include "PathPlanner.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>

constexpr float PI = 3.14159265358979323846f;

PathPlanner::PathPlanner(RobotController& robot)
    : robot_(robot)
{
    if (robot_.legCount_ != kNumLegs) {
        throw std::runtime_error("RobotController must have exactly 6 legs for PathPlanner");
    }

    // Initialize your walk cycles somewhere (or inject them)
    static WalkCycle walkCycle3Set({ {0, 2, 4}, {1, 3, 5} }, 1.f, 1.0f, 0.05f);
    static WalkCycle walkCycle2Set({ {0, 3}, {1, 4}, {2, 5} }, 3.f, 0.5f, 0.05f);
    static WalkCycle walkCycle1Set({ {0}, {1}, {2}, {3}, {4}, {5} }, 6.f, 0.2f, 0.05f);

    // Set a default walk cycle
    currentWalkCycle_ = &walkCycle3Set;

    // Initialize foot statuses with current foot positions (Vec3)
    for (size_t i = 0; i < kNumLegs; ++i) {
        Vec3 footPos3d = robot_.footTargetsWorld_[i];
        footStatuses_[i].currentPosition = footPos3d;  // Vec3 now
        footStatuses_[i].desiredTarget = footPos3d;  // Vec3 now
        footStatuses_[i].state = FootState::Grounded;
        footStatuses_[i].stepProgress = 0.f;
    }
}


void PathPlanner::update(const Vec2& joystickInput, float deltaTimeSeconds) {
    // Update step centers based on leg angle and robot heading (XY only)
    //float headingRad = robot_.Body.headingDeg * PI / 180.f;
    //for (size_t i = 0; i < robot_.legCount_; ++i) {
    //    updateStepAreaCenter(i, headingRad);
    //}

    // Update desired foot XY targets based on joystick input
    updateFootTargets(joystickInput, deltaTimeSeconds);

    // Update foot states and progress
    stepLogic(deltaTimeSeconds);

    // Push 3D foot targets (including height) to robot controller
    pushTargetsToRobot();
}

void PathPlanner::updateStepAreaCenter(size_t legIndex, float headingRad) {
    // Base vector along X-axis at stepAreaPlacementDistance
    Vec2 baseVec{ stepAreaPlacementDistance, 0.f };

    float legAngleDeg = robot_.legs[legIndex].legAngleRad_ * (180.f / PI);
    float headingDeg = headingRad * (180.f / PI);
    float totalAngleDeg = legAngleDeg + headingDeg;

    // Rotate baseVec by totalAngleDeg
    Vec2 rotatedVec = rotateZ(baseVec, totalAngleDeg);

    footStatuses_[legIndex].stepAreaCenter = rotatedVec;
}

void PathPlanner::updateStepAreaTargets(const Vec2& joystickInput, float headingRad) {
    if (joystickInput.x == 0.f && joystickInput.y == 0.f) {
        // No input, all targets reset to zero offset
        for (size_t i = 0; i < robot_.legCount_; ++i) {
            footStatuses_[i].stepAreaTarget = footStatuses_[i].stepAreaCenter; // Reset to center
        }
        return;
    }

    // Calculate input angle once
    float inputAngle = std::atan2(joystickInput.y, joystickInput.x);
    float totalAngle = inputAngle + headingRad;

    // Rotate a unit vector (stepRadius_ along X axis) by totalAngle
    Vec2 direction = rotateZ(Vec2{stepAreaRadius_, 0.f}, totalAngle * (180.f / PI));

    for (size_t i = 0; i < robot_.legCount_; ++i) {
        footStatuses_[i].stepAreaVector = direction;
        footStatuses_[i].stepAreaTarget = footStatuses_[i].stepAreaCenter + direction;
    }
}

void PathPlanner::stepLogic(float dt) {
    for (size_t i = 0; i < robot_.legCount_; ++i) {
        FootStatusInternal& foot = footStatuses_[i];

        Vec2 toTarget = Vec2(foot.desiredTarget.x - foot.currentPosition.x,
                             foot.desiredTarget.y - foot.currentPosition.y);
        float distToTarget = toTarget.length();

        const float liftThreshold = 1.0f; // cm

        if (foot.state == FootState::Grounded) {
            if (distToTarget > liftThreshold) {
                foot.state = FootState::Lifted;
                foot.stepProgress = 0.f;
            }
        }

        if (foot.state == FootState::Lifted) {
            foot.stepProgress += dt / stepDuration_;

            // Move currentPosition XY toward desiredTarget XY proportionally to step progress
            foot.currentPosition.x += toTarget.x * (dt / stepDuration_);
            foot.currentPosition.y += toTarget.y * (dt / stepDuration_);

            if (foot.stepProgress >= 1.f) {
                foot.stepProgress = 0.f;
                foot.state = FootState::Grounded;
                foot.currentPosition.x = foot.desiredTarget.x;
                foot.currentPosition.y = foot.desiredTarget.y;
            }
        }
        else {
            // Grounded feet track desired targets (XY)
            foot.currentPosition.x = foot.desiredTarget.x;
            foot.currentPosition.y = foot.desiredTarget.y;
        }
    }
}

void PathPlanner::stepLogic(float dt) {
    for (size_t i = 0; i < kNumLegs; ++i) {
        FootStatusInternal& foot = footStatuses_[i];

        Vec2 currentXY{foot.currentPosition.x, foot.currentPosition.y};
        Vec2 desiredXY{foot.desiredTarget.x, foot.desiredTarget.y};
        Vec2 toTarget = desiredXY - currentXY;
        float distToTarget = toTarget.length();

        const float liftThreshold = 1.0f; // cm

        if (foot.state == FootState::Grounded) {
            if (distToTarget > liftThreshold) {
                foot.state = FootState::Lifted;
                foot.stepProgress = 0.f;
            }
        }

        if (foot.state == FootState::Lifted) {
            foot.stepProgress += dt / stepDuration_;

            // Advance current XY toward desired XY proportional to dt / stepDuration_
            currentXY = currentXY + toTarget * (dt / stepDuration_);

            if (foot.stepProgress >= 1.f) {
                foot.stepProgress = 0.f;
                foot.state = FootState::Grounded;
                currentXY = desiredXY;
            }

            foot.currentPosition.x = currentXY.x;
            foot.currentPosition.y = currentXY.y;
        }
        else {
            // Grounded feet track desired targets directly in XY plane
            foot.currentPosition.x = desiredXY.x;
            foot.currentPosition.y = desiredXY.y;
        }
    }
}

float PathPlanner::computeFootHeight(const FootStatusInternal& footState, float progress) const {
    if (progress < 0.0f || progress > 1.0f)
        return 0.f;

    if (footState.state == FootState::Grounded) {
        return 0.f;
    }
    // Quadratic curve for step height
    return 4.0f * stepHeight * progress * (1.0f - progress);
}

void PathPlanner::pushTargetsToRobot() {
    for (size_t i = 0; i < robot_.legCount_; ++i) {
        const FootStatusInternal& foot = footStatuses_[i];

        float height = computeFootHeight(foot, foot.stepProgress);

        Vec3 footTarget3d = {
            foot.currentPosition.x,
            foot.currentPosition.y,
            height
        };

        robot_.setFootTarget(i, footTarget3d);
    }
}

void PathPlanner::updateFootTargets(const Vec2& joystickInput, float dt) {
    if (!currentWalkCycle_) return;  // Safety check

    float moveSpeed = joystickInput.length() * maxRobotSpeedCmPerSec;
    
    // Precompute the move increment vector = direction * speed * dt
    Vec2 moveIncrement = joystickInput * maxRobotSpeedCmPerSec * dt;


    for (size_t i = 0; i < robot_.legCount_; ++i) {
        FootStatusInternal& foot = footStatuses_[i];

        if (foot.state == FootState::Lifted) {
            // For lifted feet, apply a multiplier to the move increment
            foot.desiredTarget.x += moveIncrement.x * currentWalkCycle_->liftedSpeedMultiplier_;
            foot.desiredTarget.y += moveIncrement.y * currentWalkCycle_->liftedSpeedMultiplier_;
        }
    
    }

    for (size_t i = 0; i < robot_.legCount_; ++i) {
        FootStatusInternal& foot = footStatuses_[i];

        if (foot.state == FootState::Grounded) {
            // Move desired target opposite to move direction at normal speed
            foot.desiredTarget.x -= moveIncrement.x;
            foot.desiredTarget.y -= moveIncrement.y;
        } 
    

    }
}

void PathPlanner::updateFootStateTransitionsByGroup(WalkCycle* newWalkCycle = nullptr) {
    if (!currentWalkCycle_) return;  // Safety check

    float threshold = currentWalkCycle_->positionThreshold_;
    const auto& legGroups = currentWalkCycle_->getLegGroups();

    bool anyGroupLifted = false;

    // Step 1: Process groups with lifted legs for grounding
    for (const auto& group : legGroups) {
        bool hasLiftedLeg = false;
        for (size_t legIndex : group) {
            if (footStatuses_[legIndex].state == FootState::Lifted) {
                hasLiftedLeg = true;
                break;
            }
        }
        if (!hasLiftedLeg) continue;

        anyGroupLifted = true;

        bool allLiftedLegsReadyToGround = true;
        for (size_t legIndex : group) {
            FootStatusInternal& foot = footStatuses_[legIndex];
            if (foot.state == FootState::Lifted) {
                Vec2 currentXY{foot.currentPosition.x, foot.currentPosition.y};
                Vec2 targetXY{foot.stepAreaTarget.x, foot.stepAreaTarget.y};
                float distToTarget = (targetXY - currentXY).length();
                if (distToTarget > threshold) {
                    allLiftedLegsReadyToGround = false;
                    break;
                }
            }
        }

        if (allLiftedLegsReadyToGround) {
            // Ground all lifted legs in the group
            for (size_t legIndex : group) {
                FootStatusInternal& foot = footStatuses_[legIndex];
                if (foot.state == FootState::Lifted) {
                    Vec2 targetXY{foot.stepAreaTarget.x, foot.stepAreaTarget.y};
                    foot.state = FootState::Grounded;
                    foot.stepProgress = 0.f;
                    foot.currentPosition.x = targetXY.x;
                    foot.currentPosition.y = targetXY.y;
                }
            }
        }
    }

    // Step 2: Only run if no group currently has lifted legs
    if (anyGroupLifted) {
        return; // Wait for all lifted legs to ground before lifting new group
    }

    // Step 3: if no groups are lifted, change to the next requested walk cycle
    if (newWalkCycle) {
        currentWalkCycle_ = newWalkCycle;
    }

    bool liftNextGroup = false;
    bool liftDueToEdge = false;
    bool liftDueToSyncedStep = false;

    // Compute distance to back edge for each group using currentWalkCycle_
    computeDistanceToBackEdgePerGroup(*currentWalkCycle_);
    selectNextGroupToLift();

    // Spatial-based early lift: check if group is close enough to back edge, this is the snadard lift condition when legs are spaced as intended
    float earlyLiftRadius = stepAreaRadius_ * (1.f - currentWalkCycle_->earlyLiftFraction_);
    if (currentWalkCycle_->distToEdgePerGroup[currentWalkCycle_->optimalLiftedGroupIndex_] < earlyLiftRadius) {
        liftDueToEdge = true;
        liftNextGroup = true;
    }

    // Condition based on synchronized step area vector length
    if(!liftNextGroup) {

        Vec2 syncStepVec = getSynchronizedStepAreaVector(*currentWalkCycle_);

        if (syncStepVec.length() < stepAreaRadius_) {
            liftDueToSyncedStep = true;
            liftNextGroup = true;

        } else {

            float liftedSpeedMultiplier = currentWalkCycle_->liftedSpeedMultiplier_;
            if (liftedSpeedMultiplier < 0.001f) liftedSpeedMultiplier = 0.001f;
            // Compute maximum distance to target in the group with minimum distance to back edge
            float maxDistToTarget = computeMaxDistanceToTargetInGroup(currentWalkCycle_->optimalLiftedGroupIndex_);
            float syncStepDistanceAhead = syncStepVec.length() - stepAreaRadius_;
            float legCatchupTime = maxDistToTarget / liftedSpeedMultiplier;

            // Timing-based early lift
            if (legCatchupTime < syncStepDistanceAhead) {
                liftDueToSyncedStep = true;
                liftNextGroup = true;
            }
        }
    }

    // Premptive lift if next group spacing is too small
    // Early lift decision flag
    bool liftDueToSpacing = false;

    // Decide how many groups to check, e.g., half the available groups
    const size_t groupsToCheck = (currentWalkCycle_->distToEdgePerGroup.size() + 1) / 2;  // half

    if(groupsToCheck > 1 && !liftNextGroup) {

        // Get groups sorted by distToEdge ascending
        std::vector<std::pair<size_t, float>> groupDistances;
        for (size_t i = 0; i < currentWalkCycle_->distToEdgePerGroup.size(); ++i) {
            groupDistances.emplace_back(i, currentWalkCycle_->distToEdgePerGroup[i]);
        }
        std::sort(groupDistances.begin(), groupDistances.end(), 
            [](const auto& a, const auto& b) { return a.second < b.second; });

        // Target fractional spacing in cm or same units as distToEdge
        float targetSpacing = stepAreaRadius_ * 2 * currentWalkCycle_->fractionAhead_;
        float minAllowedSpacing = targetSpacing * (1 - currentWalkCycle_->earlyLiftFraction_); // 80% of ideal spacing when earlyLiftFraction_ = 0.20

        // Iterate pairs of consecutive groups in distance order
        for (size_t i = 1; i < groupsToCheck; ++i) {
            // If spacing less than minimum allowed spacing, trigger early lift
            float expectedMinSpacing = minAllowedSpacing * (i);  // Scale spacing by position
            if (groupDistances[i].second < expectedMinSpacing) {
                liftDueToSpacing = true;
                liftNextGroup = true;
                break;  // stop checking once condition is met
            }
        }
    }

    if (liftNextGroup) {
        size_t liftGroupIndex = currentWalkCycle_->optimalLiftedGroupIndex_;  // or minDistToBackGroupIndex_
        currentWalkCycle_->lastLiftedGroupIndex_ = liftGroupIndex;
        const auto& group = currentWalkCycle_->getLegGroups()[liftGroupIndex];
        for (size_t legIndex : group) {
            FootStatusInternal& foot = footStatuses_[legIndex];
            foot.state = FootState::Lifted;
            foot.stepProgress = 0.f; // Reset progress for lifted legs
        }
    } else {
        // If no group is ready to lift, just return
        return;
    }


}


void PathPlanner::selectNextGroupToLift() {
    if (!currentWalkCycle_) return;

    const auto& legGroups = currentWalkCycle_->getLegGroups();
    const auto& distances = currentWalkCycle_->distToEdgePerGroup;

    if (distances.empty() || legGroups.empty()) return;

    size_t numGroups = legGroups.size();

    // Use prime candidate computed by computeDistanceToBackEdgePerGroup
    size_t primeCandidate = currentWalkCycle_->minDistToBackGroupIndex_;

    // Find prime canndidate within 1 or -1 of last lifted group index
    size_t lastLiftedGroup = currentWalkCycle_->lastLiftedGroupIndex_;
    
    float adjacentMinDist = distances[lastLiftedGroup];
    size_t adjacentPrimeCandidate = lastLiftedGroup;

    // Check adjacent groups
    size_t nextGroup = (lastLiftedGroup + 1) % numGroups;
    size_t prevGroup = (lastLiftedGroup + numGroups - 1) % numGroups;


    if (distances[nextGroup] < adjacentMinDist) {
        adjacentMinDist = distances[nextGroup];
        adjacentPrimeCandidate = nextGroup;
    }

    if (distances[prevGroup] < adjacentMinDist) {
        adjacentMinDist = distances[prevGroup];
        adjacentPrimeCandidate = prevGroup;
    }

    // Update the optimal lifted group index to adjacent candidate
    // currentWalkCycle_->optimalLiftedGroupIndex_ = adjacentPrimeCandidate; // Test without cyclical behavior for now and check repports
    currentWalkCycle_->optimalLiftedGroupIndex_ = primeCandidate;


    currentWalkCycle_->minDistToBackGroupIndex_ = primeCandidate;
    if (primeCandidate != adjacentPrimeCandidate) {
        // Repport non cyclical group change
        std::cout << "PathPlanner: Non cyclical switching to group " << primeCandidate
                  << " with min distance " << distances[primeCandidate]
                  << " (adjacent candidate was " << adjacentPrimeCandidate
                  << " with distance " << distances[adjacentPrimeCandidate] << ")" << std::endl;
    }
}




// Compute minimum distance from current position to stepAreaTarget for all feet
float PathPlanner::computeMaxForwardDistanceToStepAreaTarget() const {
    float minDistanceToTargetY = std::numeric_limits<float>::max();
    float maxForwardDistancetoTarget = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < kNumLegs; ++i) {
        const auto& foot = footStatuses_[i];

        if (foot.state != FootState::Grounded) {
            // Skip feet that are lifted (in the air)
            continue;
        }

        // Vector from step area center (ideal foot placement) to current foot position
        Vec2 footPos{
            foot.currentPosition.x - foot.stepAreaCenter.x,
            foot.currentPosition.y - foot.stepAreaCenter.y
        };

        // Direction vector representing the allowed step direction (normalized)
        Vec2 stepDirection = foot.stepAreaVector.normalized();

        // Calculate angle (degrees) to rotate stepDirection onto +Y axis (0,1)
        float stepDirAngleDeg = std::atan2(stepDirection.y, stepDirection.x) * 180.f / PI;
        float rotationAngleDeg = 90.f - stepDirAngleDeg;

        // Rotate footPos so stepDirection aligns with Y axis
        Vec2 footForwardPos = rotateZ(footPos, rotationAngleDeg);

        // The Y component is the forward displacement along stepDirection from stepAreaCenter
        // DistanceToTargetY is the distance from footPos to the step area target along stepdirection axis
        float DistanceToTargetY = stepAreaRadius_ - footForwardPos.y;

        if (DistanceToTargetY < minDistanceToTargetY) {
            minDistanceToTargetY = DistanceToTargetY;
            maxForwardDistancetoTarget = footForwardPos.y;
        }
    }

    return maxForwardDistancetoTarget;
}

// Compute synchronized step area vector based on walk cycle and minimum distance
Vec2 PathPlanner::getSynchronizedStepAreaVector(const WalkCycle& walkCycle) const {
    float maxForwardDist = computeMaxForwardDistanceToStepAreaTarget();

    // Clamp minForwardDist to 0 if it’s very large (e.g., no grounded feet)
    if (maxForwardDist > stepAreaRadius_ * 2) {
        maxForwardDist = 0.f;
    }

    // Advance the position by a fraction (fractionAhead) of the step diameter (2 * radius)
    float scale = maxForwardDist + stepAreaRadius_ * 2 * walkCycle.fractionAhead_;

    // Assume all feet have aligned stepAreaVector, use first foot’s normalized vector
    Vec2 baseDir = footStatuses_[0].stepAreaVector.normalized();

    // Return the scaled vector representing the synchronized step target
    return baseDir * scale;
}

void PathPlanner::computeDistanceToBackEdgePerGroup(WalkCycle& walkCycle) {
    const auto& legGroups = walkCycle.getLegGroups();
    walkCycle.timeToEdgePerGroup.clear();
    walkCycle.timeToEdgePerGroup.resize(legGroups.size());

    float globalMinDistance = std::numeric_limits<float>::max();
    size_t minDistToBackGroupIndex = 0;

    for (size_t groupIndex = 0; groupIndex < legGroups.size(); ++groupIndex) {
        float minDistanceToBackEdge = std::numeric_limits<float>::max();

        for (size_t legIndex : legGroups[groupIndex]) {
            const auto& foot = footStatuses_[legIndex];

            Vec2 footPos{foot.currentPosition.x, foot.currentPosition.y};
            Vec2 stepCenter = foot.stepAreaCenter;

            // Vector from step center to foot position
            Vec2 offsetFromCenter = footPos - stepCenter;

            // Use the foot's own stepAreaVector (direction vector)
            Vec2 stepDir = foot.stepAreaVector.normalized();

            // Compute backward direction vector (opposite to stepDir)
            Vec2 backwardDir = stepDir * -1.f;

            // Compute angle (degrees) to rotate backwardDir onto +Y axis
            float backwardAngleRad = std::atan2(backwardDir.y, backwardDir.x);
            float rotationDegrees = 90.f - (backwardAngleRad * 180.f / PI);

            // Rotate offset so backwardDir aligns with +Y axis
            Vec2 rotatedOffset = rotateZ(offsetFromCenter, rotationDegrees);

            float x = rotatedOffset.x;

            if (std::abs(x) > stepAreaRadius_) {
                // Outside lateral radius, skip or treat distance as zero
                continue;
            }

            // Compute half circle edge Y at this X
            float y_edge = std::sqrt(stepAreaRadius_ * stepAreaRadius_ - x * x);

            // Distance along Y from foot to back edge
            float distanceToBackEdge = y_edge - rotatedOffset.y;

            if (distanceToBackEdge < 0.f) distanceToBackEdge = 0.f;

            if (distanceToBackEdge < minDistanceToBackEdge) {
                minDistanceToBackEdge = distanceToBackEdge;
            }
        }

        walkCycle.timeToEdgePerGroup[groupIndex] = minDistanceToBackEdge;

        if (minDistanceToBackEdge < globalMinDistance) {
            globalMinDistance = minDistanceToBackEdge;
            minDistToBackGroupIndex = groupIndex;
        }
    }

    // Store the index in WalkCycle (make sure you have this member)
    walkCycle.minDistToBackGroupIndex_ = minDistToBackGroupIndex;
}


float PathPlanner::computeMaxDistanceToTargetInGroup(size_t groupIndex) const {
    if (!currentWalkCycle_) return 0.f;  // Safety check

    const auto& legGroups = currentWalkCycle_->getLegGroups();

    if (groupIndex >= legGroups.size()) return 0.f;  // Out of bounds safety

    const auto& group = legGroups[groupIndex];

    float maxDistance = 0.f;

    for (size_t legIndex : group) {
        const auto& foot = footStatuses_[legIndex];

        Vec2 currentPos{foot.currentPosition.x, foot.currentPosition.y};
        Vec2 targetPos{foot.stepAreaTarget.x, foot.stepAreaTarget.y};

        float dist = (targetPos - currentPos).length();

        if (dist > maxDistance) {
            maxDistance = dist;
        }
    }

    return maxDistance;
}

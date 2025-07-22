#include "PathPlanner.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <iostream>


PathPlanner::PathPlanner(RobotController& robot)
    : robot_(robot)
{
    if (robot_.legCount_ != kNumLegs) {
        throw std::runtime_error("RobotController must have exactly 6 legs for PathPlanner");
    }

    // Initialize your walk cycles somewhere (or inject them)
    static WalkCycle walkCycle3Set({ {0, 2, 4}, {1, 3, 5} }, 1.f, 2.0f, 1.0f);
    static WalkCycle walkCycle2Set({ {0, 3}, {1, 4}, {2, 5} }, 3.f, 2.0f, 0.5f);
    static WalkCycle walkCycle1Set({ {0}, {1}, {2}, {3}, {4}, {5} }, 6.f, 3.0f, 0.2f);

    // Set a default walk cycle
    currentWalkCycle_ = &walkCycle3Set;


    // Initialize foot statuses with current foot positions (Vec3)
    for (size_t i = 0; i < kNumLegs; ++i) {
        
        updateStepAreaCenter(i, 0);  // Initialize step area centers
        footStatuses_[i].currentPosition.x = footStatuses_[i].stepAreaCenter.x;  // Vec3 now
        footStatuses_[i].currentPosition.y = footStatuses_[i].stepAreaCenter.y;  // Vec3 now
        footStatuses_[i].currentPosition.z = 0.f;  // Flat on ground
        footStatuses_[i].desiredTarget.x = footStatuses_[i].stepAreaCenter.x;  // Vec3 now
        footStatuses_[i].desiredTarget.y = footStatuses_[i].stepAreaCenter.y;  // Vec3 now
        footStatuses_[i].desiredTarget.z = 0.f;  // Flat on ground
        footStatuses_[i].state = FootState::Grounded;
        footStatuses_[i].stepProgress = 0.f;
    }
}


void PathPlanner::update(const Vec2& directionInput, float turningInput, float deltaTimeSeconds) {
    if (directionInput.length() <= 0.01f) {
        if (!returnToZeroRequested_) {
            std::cout << "[ReturnToZero] Activating return-to-zero mode (joystick idle)" << std::endl;
        }
        requestReturnToZero(true);
    }

    if (directionInput.length() > 0.01f) {
        if (returnToZeroRequested_) {
            std::cout << "[ReturnToZero] Cancelling return-to-zero mode (joystick active)" << std::endl;
        }
        requestReturnToZero(false);
    }


    // Update step centers based on leg angle and robot heading (XY only)
    // float headingRad = robot_.Body.headingDeg * M_PI / 180.f;
    float headingRad = 0; // Added for stabilized heading, condition for updating heading
    for (size_t i = 0; i < robot_.legCount_; ++i) {
        updateStepAreaCenter(i, headingRad);
    }

    // Update desired foot XY targets based on joystick input
    updateStepAreaTargets(directionInput);

    // Update foot states and progress
    stepPathLogic(directionInput, turningInput, deltaTimeSeconds);

    // Push 3D foot targets (including height) to robot controller
    pushTargetsToRobot();

    /*
    std::cout << "[PathPlanner] Leg Status Summary:\n";
    for (size_t i = 0; i < robot_.legCount_; ++i) {
        const auto& foot = footStatuses_[i];
        std::cout << "  Leg " << i
                << " Position: (" << foot.currentPosition.x << ", "
                                    << foot.currentPosition.y << ", "
                                    << foot.currentPosition.z << ")"
                << "  StepAreaCenter: (" << foot.stepAreaCenter.x << ", "
                                        << foot.stepAreaCenter.y << ")\n";
    }
    */
    
}

void PathPlanner::updateStepAreaCenter(size_t legIndex, float headingRad) {
    // Base vector along X-axis at stepAreaPlacementDistance
    Vec2 baseVec{ stepAreaPlacementDistance, 0.f };

    float legAngleDeg = robot_.legs[legIndex].legAngleRad_ * (180.f / M_PI);
    float headingDeg = headingRad * (180.f / M_PI);
    float totalAngleDeg = legAngleDeg + headingDeg;

    // Rotate baseVec by totalAngleDeg
    Vec2 rotatedVec = rotateZ(baseVec, totalAngleDeg);

    footStatuses_[legIndex].stepAreaCenter = rotatedVec;
}

void PathPlanner::updateStepAreaTargets(const Vec2& joystickInput) {
    if (joystickInput.x == 0.f && joystickInput.y == 0.f) {
        // No input, all targets reset to zero offset
        for (size_t i = 0; i < robot_.legCount_; ++i) {
            footStatuses_[i].stepAreaTarget = footStatuses_[i].stepAreaCenter; // Reset to center
        }
        return;
    }

    // Calculate input angle once
    float inputAngle = std::atan2(joystickInput.y, joystickInput.x);
    float totalAngle = inputAngle;

    // Rotate a unit vector (stepRadius_ along X axis) by totalAngle
    Vec2 direction = rotateZ(Vec2{stepAreaRadius_, 0.f}, totalAngle * (180.f / M_PI));

    for (size_t i = 0; i < robot_.legCount_; ++i) {
        footStatuses_[i].stepAreaVector = direction;
        footStatuses_[i].stepAreaTarget = footStatuses_[i].stepAreaCenter + direction;
    }
}

void PathPlanner::computeFootHeights(float deltaTimeSeconds, const Vec2& joystickInput) {
    const float maxHeightChangeSpeed = maxRobotSpeedCmPerSec; // cm per second max vertical speed

    for (size_t i = 0; i < robot_.legCount_; ++i) {
        FootStatusInternal& foot = footStatuses_[i];
        Vec2 currentXY{foot.currentPosition.x, foot.currentPosition.y};
        Vec2 targetXY = foot.stepAreaTarget;

        // Distance from current foot position to target in XY plane
        float distToTarget = (targetXY - currentXY).length();

        float currentHeight = foot.currentPosition.z;

        // Decide desired target height based on distance
        float desiredHeight = 0.f;

        if (foot.state == FootState::Lifted) {
            // If foot is lifted, desired height grows with distance but maxes at stepHeight
            // The farther from target, the higher the foot should lift (up to stepHeight)
            desiredHeight = std::min(distToTarget, stepHeight);
        } else {
            // Foot on ground → desired height is zero
            desiredHeight = 0.f;
        }

        float heightDiff = desiredHeight - currentHeight;
        float maxHeightChangeThisFrame = maxHeightChangeSpeed * deltaTimeSeconds;

        // Smoothly move height towards desiredHeight with speed limit
        if (std::abs(heightDiff) <= maxHeightChangeThisFrame) {
            foot.desiredTarget.z = desiredHeight;
        } else {
            foot.desiredTarget.z = currentHeight + (heightDiff > 0 ? maxHeightChangeThisFrame : -maxHeightChangeThisFrame);
        }
    }
}



void PathPlanner::pushTargetsToRobot() {
    for (size_t i = 0; i < robot_.legCount_; ++i) {
        FootStatusInternal& foot = footStatuses_[i];


        Vec3 footTarget3d = {
            foot.desiredTarget.x,
            foot.desiredTarget.y,
            foot.desiredTarget.z
        };

        robot_.setFootTarget(i, footTarget3d);
        foot.currentPosition = foot.desiredTarget;
    }
}

void PathPlanner::stepPathLogic(const Vec2& directionInput, float turningInput, float dt) {
    if (!currentWalkCycle_) return;  // Safety check

    // Precompute the move increment vector = direction * speed * dt
    Vec2 moveIncrement = directionInput * maxRobotSpeedCmPerSec * dt;

    // Variable to reduce speed if necessary
    // This will be used to limit the step increment of lifted legs and reduce robot speed if necessary    
    const float maxLiftedLegIncrement = maxRobotSpeedCmPerSec * dt * currentWalkCycle_->liftedSpeedMultiplier_;


    // Update step area centers based on leg angle and robot heading
    updateStepAreaTargets(directionInput);
    updateSynchronizedStepAreaTargets();
    
    // Compute distance to back edge for each group using currentWalkCycle_
    computeDistanceToBackEdgePerGroup(*currentWalkCycle_);

    if (turningInput != 0.0f) {
        returnToZeroSpacingDistance = stepAreaRadius_ * currentWalkCycle_->fractionAhead_ / 2;
    } else {
        returnToZeroSpacingDistance = currentWalkCycle_->positionThreshold_;
    }
    
    updateFootStateTransitionsByGroup();

    // --- 0) Steering rotation for grounded feet ---
    // Compute how many degrees to rotate this frame
    float deltaDeg = turningInput * maxRobotSteeringDeltaDegperSec * dt;
    // Pre–compute for Vec2 rotation
    float deltaRad = deltaDeg * (M_PI / 180.f);

    for (size_t i = 0; i < robot_.legCount_; ++i) {
        auto& foot = footStatuses_[i];
        if (foot.state == FootState::Grounded) {
            // Rotate the *current* XY positions around the body origin
            Vec2 pos{ foot.currentPosition.x, foot.currentPosition.y };
            Vec2 rotated = rotateZ(pos, deltaRad);
            foot.currentPosition.x = rotated.x;
            foot.currentPosition.y = rotated.y;

            // Also rotate the *desiredTarget* to keep them in sync
            Vec2 tgt{ foot.desiredTarget.x, foot.desiredTarget.y };
            Vec2 tgtRot = rotateZ(tgt, deltaRad);
            foot.desiredTarget.x = tgtRot.x;
            foot.desiredTarget.y = tgtRot.y;
        }
    }


    // Step 0.5: Leg Return‑to‑Zero Step Logic
    if (returnToZeroRequested_) {
        float fixedStepSpeed = maxRobotSpeedCmPerSec * currentWalkCycle_->liftedSpeedMultiplier_ / 2;
        float maxStep = fixedStepSpeed * dt;

        for (size_t i = 0; i < robot_.legCount_; ++i) {
            FootStatusInternal& foot = footStatuses_[i];

            if (foot.state == FootState::Lifted) {
                Vec2 currentXY{foot.currentPosition.x, foot.currentPosition.y};
                Vec2 center = foot.stepAreaCenter;
                Vec2 toCenter = center - currentXY;

                float dist = toCenter.length();
                Vec2 stepVec = (dist <= maxStep) ? toCenter : toCenter.normalized() * maxStep;

                foot.desiredTarget.x += stepVec.x;
                foot.desiredTarget.y += stepVec.y;

                foot.stepProgress = (dist <= maxStep) ? 1.f : foot.stepProgress;
            }
        }

        // Skip everything else and just update heights
        computeFootHeights(dt, Vec2{0.f, 0.f});
        return;
    }




    // -- Active robot moving lifted leg step logic --
    // 1. Find max speed reduction factor needed
    float speedReductionFactor = 1.0f;  // 1.0 means no reduction

    for (size_t i = 0; i < robot_.legCount_; ++i) {
        const FootStatusInternal& foot = footStatuses_[i];

        if (foot.state == FootState::Lifted) {
            Vec2 currentXY{foot.currentPosition.x, foot.currentPosition.y};

            // Vectors to both targets
            Vec2 toTarget = foot.stepAreaTarget - currentXY;
            Vec2 toSyncTarget = foot.stepAreaSyncronizedTarget - currentXY;
            Vec2 chosenTarget = (toSyncTarget.length() < toTarget.length()) ? foot.stepAreaSyncronizedTarget : foot.stepAreaTarget;
            // Pick the closer target
            Vec2 toChosenTarget = chosenTarget - currentXY;

            float minDistToBackEdge = currentWalkCycle_->getMinDistToBackEdge();
            if (minDistToBackEdge <= 0.f) continue;

            float minStepProgress = moveIncrement.length() / minDistToBackEdge;
            Vec2 minStepIncrement = toChosenTarget * minStepProgress;

            if (minStepIncrement.length() > maxLiftedLegIncrement) {
                float legReduction = maxLiftedLegIncrement / minStepIncrement.length();
                if (legReduction < speedReductionFactor) {
                    speedReductionFactor = legReduction;
                }
            }
        }
    }

    // 2. Apply step increments with computed speedReductionFactor
    for (size_t i = 0; i < robot_.legCount_; ++i) {
        FootStatusInternal& foot = footStatuses_[i];

        if (foot.state == FootState::Lifted) {
            Vec2 currentXY{foot.currentPosition.x, foot.currentPosition.y};

            // Vectors to both targets
            Vec2 toTarget = foot.stepAreaTarget - currentXY;
            Vec2 toSyncTarget = foot.stepAreaSyncronizedTarget - currentXY;
            Vec2 chosenTarget = (toSyncTarget.length() < toTarget.length()) ? foot.stepAreaSyncronizedTarget : foot.stepAreaTarget;
            // Pick the closer target
            Vec2 toChosenTarget = chosenTarget - currentXY;

            float stepMoveIncrementLength = moveIncrement.length() * currentWalkCycle_->liftedSpeedMultiplier_ * speedReductionFactor;

            float minDistToBackEdge = currentWalkCycle_->getMinDistToBackEdge();
            float minStepProgress = moveIncrement.length() / minDistToBackEdge;
            Vec2 minStepIncrement{0.f, 0.f};
            if (minDistToBackEdge > 0.f) {
                minStepIncrement = toChosenTarget * minStepProgress;
            }

            minStepIncrement = minStepIncrement * speedReductionFactor;

            if (minStepIncrement.length() > stepMoveIncrementLength) {
                if (toChosenTarget.length() < currentWalkCycle_->positionThreshold_) {
                    foot.stepProgress = 1.f;
                } else {
                    foot.stepProgress += minStepIncrement.length() / toChosenTarget.length();
                }

                foot.desiredTarget.x += minStepIncrement.x;
                foot.desiredTarget.y += minStepIncrement.y;
            } else {
                if (toChosenTarget.length() < currentWalkCycle_->positionThreshold_) {
                    foot.stepProgress = 1.f;
                } else {
                    foot.stepProgress += stepMoveIncrementLength / toChosenTarget.length();
                }

                Vec2 toChosenTargetNormalized = toChosenTarget.normalized();
                foot.desiredTarget.x += toChosenTargetNormalized.x * stepMoveIncrementLength;
                foot.desiredTarget.y += toChosenTargetNormalized.y * stepMoveIncrementLength;
            }
        }
    }

    for (size_t i = 0; i < robot_.legCount_; ++i) {
        FootStatusInternal& foot = footStatuses_[i];

        if (foot.state == FootState::Grounded) {
            // Move desired target opposite to move direction at normal speed
            foot.stepProgress = 0.f; // Reset step progress for grounded feet
            foot.desiredTarget.x -= moveIncrement.x * speedReductionFactor;
            foot.desiredTarget.y -= moveIncrement.y * speedReductionFactor;

            // Clamp desired target to remain within 120% of step circle radius
            Vec2 center = foot.stepAreaCenter;
            Vec2 desiredXY{foot.desiredTarget.x, foot.desiredTarget.y};
            Vec2 toDesired = desiredXY - center;

            float maxRadius = stepAreaRadius_ * 1.2f;
            if (toDesired.length() > maxRadius) {
                Vec2 clamped = center + toDesired.normalized() * maxRadius;
                foot.desiredTarget.x = clamped.x;
                foot.desiredTarget.y = clamped.y;
            }
        }
    }

    computeFootHeights(dt, directionInput);  // Update foot heights based on step progress

}

void PathPlanner::updateFootStateTransitionsByGroup() {
    if (!currentWalkCycle_) return;  // Safety check

    float threshold = currentWalkCycle_->positionThreshold_;
    const auto& legGroups = currentWalkCycle_->getLegGroups();

    bool anyGroupLifted = false;

    // Step 0.8: Ground groups returning to zero
    // Check if legs returning to center are ready to ground
    if (returnToZeroRequested_) {
        for (size_t groupIndex = 0; groupIndex < legGroups.size(); ++groupIndex) {
            const auto& group = legGroups[groupIndex];
            bool groupIsLifted = false;
            bool allWithinCenterThreshold = true;

            for (size_t legIndex : group) {
                const FootStatusInternal& foot = footStatuses_[legIndex];
                if (foot.state == FootState::Lifted) {
                    
                    anyGroupLifted = true;

                    groupIsLifted = true;
                    Vec2 pos{foot.currentPosition.x, foot.currentPosition.y};
                    Vec2 center = foot.stepAreaCenter;
                    float dist = (pos - center).length();
                    if (dist > (stepAreaRadius_ * 0.1f)) {
                        allWithinCenterThreshold = false;
                        break;
                    }
                }
            }

            if (groupIsLifted && allWithinCenterThreshold) {
                std::cout << "[Grounding - ReturnToZero] Grounding group " << groupIndex << " (all feet near center)\n";
                for (size_t legIndex : group) {
                    footStatuses_[legIndex].state = FootState::Grounded;
                    footStatuses_[legIndex].stepProgress = 0.f;
                }
            }
        }
    }


    // Step 1: Process groups with lifted legs for grounding
    if (!returnToZeroRequested_){
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

        bool anyLiftedLegReadyToGround = false;
        for (size_t legIndex : group) {
            FootStatusInternal& foot = footStatuses_[legIndex];
            if (foot.state == FootState::Lifted) {
                Vec2 currentXY{foot.currentPosition.x, foot.currentPosition.y};
                Vec2 targetXY = Vec2{foot.stepAreaTarget.x, foot.stepAreaTarget.y};

                Vec2 toTarget = targetXY - currentXY;

                float distToTarget = toTarget.length();

                // Direction vector for the step area (should be normalized)
                Vec2 stepDir = foot.stepAreaVector.normalized();

                // Check if overshot: dot product < 0 means went past target
                float dot = toTarget.x * stepDir.x + toTarget.y * stepDir.y;

                if (distToTarget < threshold || dot < 0.1f) {
                    anyLiftedLegReadyToGround = true;
                    break;  // One leg ready is enough
                }
            }
        }

        if (anyLiftedLegReadyToGround) {
            // Ground all lifted legs in the group by resetting their currentPosition to zero
            for (size_t legIndex : group) {
                FootStatusInternal& foot = footStatuses_[legIndex];
                if (foot.state == FootState::Lifted || foot.state == FootState::Grounded) {
                    foot.state = FootState::Grounded;
                    foot.stepProgress = 0.f;
                }
            }
        }
    }
    }

    // Step 2: Only run if no group currently has lifted legs
    if (anyGroupLifted) {
        return; // Wait for all lifted legs to ground before lifting new group
    }

    // Step 3: if no groups are lifted, change to the next requested walk cycle
    if (nextWalkCycle_) {
        currentWalkCycle_ = nextWalkCycle_;
    }

    bool liftNextGroup = false;
    bool liftDueToEdge = false;
    bool liftDueToSyncedStep = false;

    
    if (returnToZeroRequested_) {
        int resetGroup = pickNextResetGroup();
        if (resetGroup == -1) {
            std::cout << "[ReturnToZero] Completed return-to-zero, resuming normal walk\n";
            requestReturnToZero(false);
            return;
        } else {
            liftNextGroup = true;
            currentWalkCycle_->optimalLiftedGroupIndex_ = resetGroup;
        }
    } else {

    
    // Step 4: Select the next group to lift based on distance to back edge
    selectNextGroupToLift();

    // Spatial-based early lift: check if group is close enough to back edge, this is the standard lift condition when legs are spaced as intended
    float distToEdge = currentWalkCycle_->distToEdgePerGroup[currentWalkCycle_->optimalLiftedGroupIndex_];
    if (distToEdge < threshold) {
        liftDueToEdge = true;
        liftNextGroup = true;
        //std::cout << "[Lift] Due to edge proximity. Distance to edge: " << distToEdge 
        //          << " (threshold: " << threshold << ")\n";
    }

    // Condition based on synchronized step area vector length
    if (!liftNextGroup) {
        Vec2 syncStepVec = getSynchronizedStepAreaVector(*currentWalkCycle_);

        if (syncStepVec.length() < stepAreaRadius_) {
            liftDueToSyncedStep = true;
            liftNextGroup = true;
            //std::cout << "[Lift] Due to synchronized step vector length being small. Length: " << syncStepVec.length() << "\n";
        } else {
            float liftedSpeedMultiplier = currentWalkCycle_->liftedSpeedMultiplier_;
            if (liftedSpeedMultiplier < 0.001f) liftedSpeedMultiplier = 0.001f;
            float maxDistToTarget = computeMaxDistanceToTargetInGroup(currentWalkCycle_->optimalLiftedGroupIndex_);
            float syncStepDistanceAhead = syncStepVec.length() - stepAreaRadius_;
            float legCatchupTime = maxDistToTarget / liftedSpeedMultiplier;

            if (legCatchupTime > syncStepDistanceAhead) {
                liftDueToSyncedStep = true;
                liftNextGroup = true;
                //std::cout << "[Lift] Due to timing-based early lift. legCatchupTime: " << legCatchupTime
                //          << " syncStepDistanceAhead: " << syncStepDistanceAhead << "\n";
            }
        }
    }

    // Premptive lift if next group spacing is too small
    bool liftDueToSpacing = false;

    const size_t groupsToCheck = (currentWalkCycle_->distToEdgePerGroup.size() + 1) / 2;  // half

    if (groupsToCheck > 1 && !liftNextGroup) {
        std::vector<std::pair<size_t, float>> groupDistances;
        for (size_t i = 0; i < currentWalkCycle_->distToEdgePerGroup.size(); ++i) {
            groupDistances.emplace_back(i, currentWalkCycle_->distToEdgePerGroup[i]);
        }
        std::sort(groupDistances.begin(), groupDistances.end(), 
            [](const auto& a, const auto& b) { return a.second < b.second; });

        float targetSpacing = stepAreaRadius_ * 2 * currentWalkCycle_->fractionAhead_;
        float minAllowedSpacing = targetSpacing * (1 - currentWalkCycle_->earlyLiftFraction_);

        for (size_t i = 1; i < groupsToCheck; ++i) {
            float expectedMinSpacing = minAllowedSpacing * (i);
            if (groupDistances[i].second < expectedMinSpacing) {
                liftDueToSpacing = true;
                liftNextGroup = true;
                //std::cout << "[Lift] Due to group spacing too small. Group " << groupDistances[i].first
                //          << " distance: " << groupDistances[i].second
                //          << " expected min spacing: " << expectedMinSpacing << "\n";
                break;
            }
        }
    }

    }

    if (liftNextGroup) {
        size_t liftGroupIndex = currentWalkCycle_->optimalLiftedGroupIndex_;  // or minDistToBackGroupIndex_

        currentWalkCycle_->lastLiftedGroupIndex_ = liftGroupIndex;
        const auto& group = currentWalkCycle_->getLegGroups()[liftGroupIndex];
        //std::cout << "[Lift] Lifting group " << liftGroupIndex << " legs: ";
        
        for (size_t legIndex : group) {
            // std::cout << legIndex << " ";
            FootStatusInternal& foot = footStatuses_[legIndex];
            foot.state = FootState::Lifted;
            foot.stepProgress = 0.f;
        }
        // std::cout << std::endl;
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
        //std::cout << "PathPlanner: Non cyclical switching to group " << primeCandidate
        //          << " with min distance " << distances[primeCandidate]
        //          << " (adjacent candidate was " << adjacentPrimeCandidate
        //          << " with distance " << distances[adjacentPrimeCandidate] << ")" << std::endl;
    }
}




// Compute minimum distance from current position to stepAreaTarget for all feet
float PathPlanner::computeMaxDistanceAlongStep() const {
    float maxDistanceToTargetY = -std::numeric_limits<float>::max(); // Start very low

    for (size_t i = 0; i < kNumLegs; ++i) {
        const auto& foot = footStatuses_[i];

        if (foot.state != FootState::Grounded) {
            // Skip feet that are lifted (in the air)
            continue; //skip lifted feet
        }

        // Vector from step area center (ideal foot placement) to current foot position
        Vec2 footPos{
            foot.currentPosition.x - foot.stepAreaCenter.x,
            foot.currentPosition.y - foot.stepAreaCenter.y
        };

        // Direction vector representing the allowed step direction (normalized)
        Vec2 stepDirection = foot.stepAreaVector.normalized();

        // Calculate angle (degrees) to rotate stepDirection onto +Y axis (0,1)
        float stepDirAngleDeg = std::atan2(stepDirection.y, stepDirection.x) * 180.f / M_PI;
        float rotationAngleDeg = 90.f - stepDirAngleDeg;

        // Rotate footPos so stepDirection aligns with Y axis
        Vec2 footForwardPos = rotateZ(footPos, rotationAngleDeg);

        // footForwardPos.y is progress along step direction
        float distanceAlongStep = footForwardPos.y;

        if (distanceAlongStep > maxDistanceToTargetY) {
            maxDistanceToTargetY = distanceAlongStep;
        }
    }

    // std::cout << "Max forward distance to step area target: " << maxDistanceToTargetY << std::endl;
    return maxDistanceToTargetY;
}



// Compute synchronized step area vector based on walk cycle and minimum distance
Vec2 PathPlanner::getSynchronizedStepAreaVector(const WalkCycle& walkCycle) const {
    float maxForwardDist = computeMaxDistanceAlongStep();

    // Clamp minForwardDist to 0 if it’s very large (e.g., no grounded feet)
    if (maxForwardDist > stepAreaRadius_ * 2) {
        maxForwardDist = 0.f;
    }

    // Advance the position by a fraction (fractionAhead) of the step diameter (2 * radius)
    float scale = maxForwardDist + stepAreaRadius_ * 2 * walkCycle.fractionAhead_;

    // Assume all feet have aligned stepAreaVector, use first foot’s normalized vector
    Vec2 baseDir = footStatuses_[0].stepAreaVector.normalized();

    // Return the scaled vector representing the synchronized step target
    // std::cout << "Synchronized step area vector length: " << scale << std::endl;
    return baseDir * scale;
}

void PathPlanner::updateSynchronizedStepAreaTargets() {
    // Find the synchronized step area vector ahead distance
    float syncronizedStepAreaVectorLength = getSynchronizedStepAreaVector(*currentWalkCycle_).length();

    for (size_t i = 0; i < robot_.legCount_; ++i) {
        FootStatusInternal& foot = footStatuses_[i];

        // Compute target at edge or beyond of step area circle
        foot.stepAreaSyncronizedTarget = foot.stepAreaCenter + foot.stepAreaVector.normalized() * syncronizedStepAreaVectorLength;
    }
}


void PathPlanner::computeDistanceToBackEdgePerGroup(WalkCycle& walkCycle) {
    const auto& legGroups = walkCycle.getLegGroups();
    walkCycle.distToEdgePerGroup.clear();
    walkCycle.distToEdgePerGroup.resize(legGroups.size());

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
            float rotationDegrees = 90.f - (backwardAngleRad * 180.f / M_PI);

            // Rotate offset so backwardDir aligns with +Y axis
            Vec2 rotatedOffset = rotateZ(offsetFromCenter, rotationDegrees);

            float x = rotatedOffset.x;
            float distanceToBackEdge = stepAreaRadius_ * 2;

            if (std::abs(x) > stepAreaRadius_) {
                // Outside lateral radius, treat as negative distance
                distanceToBackEdge = -(std::abs(x) - stepAreaRadius_);
            } else {

                // Compute half circle edge Y at this X
                float y_edge = std::sqrt(stepAreaRadius_ * stepAreaRadius_ - x * x);

                // Distance along Y from foot to back edge
                distanceToBackEdge = y_edge - rotatedOffset.y;
            }


            if (distanceToBackEdge < minDistanceToBackEdge) {
                minDistanceToBackEdge = distanceToBackEdge;
            }
        }

        walkCycle.distToEdgePerGroup[groupIndex] = minDistanceToBackEdge;

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


void PathPlanner::printDebugStatus() const {
    std::cout << "PathPlanner Debug Status:" << std::endl;
    for (size_t i = 0; i < robot_.legCount_; ++i) {
        const auto& foot = footStatuses_[i];

        // Distance to target (currentPosition to stepAreaTarget)
        Vec2 currentPos{foot.currentPosition.x, foot.currentPosition.y};
        Vec2 targetPos = foot.stepAreaTarget;
        float distToTarget = (targetPos - currentPos).length();

        // Optionally compute distanceToBackEdge per leg (simplified)
        // For now just print the group min distance from WalkCycle
        float distToBackEdge = 0.f;
        if (currentWalkCycle_) {
            const auto& legGroups = currentWalkCycle_->getLegGroups();
            for (size_t groupIndex = 0; groupIndex < legGroups.size(); ++groupIndex) {
                if (std::find(legGroups[groupIndex].begin(), legGroups[groupIndex].end(), i) != legGroups[groupIndex].end()) {
                    distToBackEdge = currentWalkCycle_->distToEdgePerGroup[groupIndex];
                    break;
                }
            }
        }

        std::cout << " Leg " << i
                  << ": stepAreaCenter=(" << foot.stepAreaCenter.x << "," << foot.stepAreaCenter.y << ")"
                  << " stepAreaTarget=(" << foot.stepAreaTarget.x << "," << foot.stepAreaTarget.y << ")"
                  << " syncedTarget=(" << foot.stepAreaSyncronizedTarget.x << "," << foot.stepAreaSyncronizedTarget.y << ")"
                  << " distToBackEdge=" << distToBackEdge
                  << " distToTarget=" << distToTarget
                  << std::endl;
    }
}

void PathPlanner::requestReturnToZero(bool enabled) {
    if (returnToZeroRequested_ == enabled) return;  // No change
    returnToZeroRequested_ = enabled;
}

bool PathPlanner::needsReset(size_t legIndex) const {
    const auto& foot = footStatuses_[legIndex];
    if (foot.state == FootState::Lifted) return false;  // we're already moving this one

    Vec2 footPos{foot.currentPosition.x, foot.currentPosition.y};
    Vec2 center = foot.stepAreaCenter;
    float dist = (footPos - center).length();
    /*
    std::cout << "[ResetCheck] Leg " << legIndex
          << " | State: " << (int)footStatuses_[legIndex].state
          << " | Position: (" << foot.currentPosition.x << ", " << foot.currentPosition.y << ")"
          << " | Center: (" << center.x << ", " << center.y << ")"
          << " | DistToCenter: " << dist << std::endl;
    */

    return dist > (stepAreaRadius_ * 0.05f);
}


int PathPlanner::pickNextResetGroup() const {
    if (!currentWalkCycle_) return -1;

    const auto& groups = currentWalkCycle_->getLegGroups();
    int bestGroup = -1;
    float bestMinDist = std::numeric_limits<float>::max();

    std::cout << "[PickResetGroup] Checking groups...\n";

    for (size_t i = 0; i < groups.size(); ++i) {
        float groupMinDist = std::numeric_limits<float>::max();
        bool groupNeedsReset = false;

        std::cout << "  Group " << i << ":\n";

        for (size_t legIndex : groups[i]) {
            const auto& foot = footStatuses_[legIndex];

            if (foot.state == FootState::Lifted) {
                std::cout << "    Leg " << legIndex << " is lifted, skipping\n";
                continue;
            }

            Vec2 footPos{foot.desiredTarget.x, foot.desiredTarget.y};
            Vec2 center = foot.stepAreaCenter;
            float dist = (footPos - center).length();
            bool needs = dist > returnToZeroSpacingDistance;

            std::cout << "    Leg " << legIndex
                      << " | Pos=(" << footPos.x << "," << footPos.y << ")"
                      << " | Center=(" << center.x << "," << center.y << ")"
                      << " | Dist=" << dist
                      << " | NeedsReset=" << (needs ? "YES" : "no") << "\n";

            if (needs) {
                groupNeedsReset = true;
                groupMinDist = std::min(groupMinDist, dist);
            }
        }

        if (groupNeedsReset) {
            std::cout << "    → Group " << i << " needs reset (minDist=" << groupMinDist << ")\n";
            if (groupMinDist < bestMinDist) {
                bestMinDist = groupMinDist;
                bestGroup = static_cast<int>(i);
            }
        } else {
            std::cout << "    → Group " << i << " does NOT need reset\n";
        }
    }

    if (bestGroup == -1) {
        std::cout << "[PickResetGroup] No group needs reset.\n";
    } else {
        std::cout << "[PickResetGroup] Selected group " << bestGroup << " (minDist=" << bestMinDist << ")\n";
    }

    return bestGroup;
}




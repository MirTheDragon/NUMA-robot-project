#include "RobotController.hpp"
#include <cstdio>

RobotController::RobotController()
    : legs_{
        ThreePointLeg(-30.f),  // Front Left leg angle
        ThreePointLeg(-90.f),  // Left leg angle
        ThreePointLeg(-150.f), // Back Left leg angle
        ThreePointLeg(30.f),   // Front Right leg angle
        ThreePointLeg(90.f),   // Right leg angle
        ThreePointLeg(150.f)   // Back Right leg angle
      }
    , servoControllers_{
        ServoController(1, 0x41, 200),
        ServoController(1, 0x43, 200)
      }
{
    initServoMappingsAndConfigs();
}

void RobotController::initServoMappingsAndConfigs(int legCount) {
    if (legCount > 6) legCount = 6; // max legs
    if (legCount < 1) legCount = 1; // min legs

    // For each leg up to legCount, initialize it individually
    if (legCount > 0) {
        // --- Leg 0 (Right Middle, 0 deg) ---
        legs_[0] = ThreePointLeg(0.f);
        servoMappings_[0][0] = {0, 0};  // Hip Horizontal
        servoMappings_[0][1] = {0, 1};  // Hip Vertical
        servoMappings_[0][2] = {0, 2};  // Knee

        servoControllers_[0].setServoConfig(0, ServoConfig("Leg0_HipHorizontal", 500, 2500, 270.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(1, ServoConfig("Leg0_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(2, ServoConfig("Leg0_Knee", 500, 2500, 140.f, 0.f, 140.f));
    }
    if (legCount > 1) {
        // --- Leg 1 (Right Front, 60 deg) ---
        legs_[1] = ThreePointLeg(60.f);
        servoMappings_[1][0] = {0, 3};
        servoMappings_[1][1] = {0, 4};
        servoMappings_[1][2] = {0, 5};

        servoControllers_[0].setServoConfig(3, ServoConfig("Leg1_HipHorizontal", 500, 2500, 270.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(4, ServoConfig("Leg1_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(5, ServoConfig("Leg1_Knee", 500, 2500, 140.f, 0.f, 140.f));
    }
    if (legCount > 2) {
        // --- Leg 2 (Left Front, 120 deg) ---
        legs_[2] = ThreePointLeg(120.f);
        servoMappings_[2][0] = {0, 6};
        servoMappings_[2][1] = {0, 7};
        servoMappings_[2][2] = {0, 8};

        servoControllers_[0].setServoConfig(6, ServoConfig("Leg2_HipHorizontal", 500, 2500, 270.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(7, ServoConfig("Leg2_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(8, ServoConfig("Leg2_Knee", 500, 2500, 140.f, 0.f, 140.f));
    }
    if (legCount > 3) {
        // --- Leg 3 (Left Middle, 180 deg) ---
        legs_[3] = ThreePointLeg(180.f);
        servoMappings_[3][0] = {0, 9};
        servoMappings_[3][1] = {0, 10};
        servoMappings_[3][2] = {0, 11};

        servoControllers_[0].setServoConfig(9, ServoConfig("Leg3_HipHorizontal", 500, 2500, 270.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(10, ServoConfig("Leg3_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(11, ServoConfig("Leg3_Knee", 500, 2500, 140.f, 0.f, 140.f));
    }
    if (legCount > 4) {
        // --- Leg 4 (Left Back, 240 deg) ---
        legs_[4] = ThreePointLeg(240.f);
        servoMappings_[4][0] = {0, 12};
        servoMappings_[4][1] = {0, 13};
        servoMappings_[4][2] = {0, 14};

        servoControllers_[0].setServoConfig(12, ServoConfig("Leg4_HipHorizontal", 500, 2500, 270.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(13, ServoConfig("Leg4_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(14, ServoConfig("Leg4_Knee", 500, 2500, 140.f, 0.f, 140.f));
    }
    if (legCount > 5) {
        // --- Leg 5 (Right Back, 300 deg) ---
        legs_[5] = ThreePointLeg(300.f);
        servoMappings_[5][0] = {1, 0};
        servoMappings_[5][1] = {1, 1};
        servoMappings_[5][2] = {1, 2};

        servoControllers_[1].setServoConfig(0, ServoConfig("Leg5_HipHorizontal", 500, 2500, 270.f, -90.f, 90.f));
        servoControllers_[1].setServoConfig(1, ServoConfig("Leg5_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[1].setServoConfig(2, ServoConfig("Leg5_Knee", 500, 2500, 140.f, 0.f, 140.f));
    }
}


void RobotController::updateAllIK() {
    for (size_t i = 0; i < legs_.size(); ++i) {
        // Convert stored foot target in world coordinates to body coordinates
        Vec3 footInBody = Body_.worldToBodyCoords(footTargetsWorld_[i]); // you need footTargetsWorld_ to hold world targets
        
        // Convert from body coordinates to leg coordinates
        Vec3 footInLeg = legs_[i].bodyToLegCoords(footInBody);
        
        // Set the leg's foot target to leg coordinates for IK
        legs_[i].footTarget = footInLeg;
        
        // Solve IK for the leg with this foot target
        legs_[i].solveIK();
    }
}


void RobotController::updateServoAnglesFromIK() {
    // For each leg
    for (size_t legIndex = 0; legIndex < legs_.size(); ++legIndex) {
        const auto& leg = legs_[legIndex];

        // For each joint: HipHorizontal, HipVertical, Knee
        // Map to servo controller and channel
        for (size_t joint = 0; joint < 3; ++joint) {
            const ServoMapping& mapping = servoMappings_[legIndex][joint];
            ServoController& controller = servoControllers_[mapping.controllerIndex];

            float angleRad = 0.0f;
            if (joint == 0)
                angleRad = leg.servoHipHorizontalRad;
            else if (joint == 1)
                angleRad = leg.servoHipVerticalRad;
            else if (joint == 2)
                angleRad = leg.servoKneeRad;

            // Convert radians to degrees before sending to servo controller
            float angleDeg = angleRad * 180.0f / PI;

            controller.setServoAngle(mapping.channelIndex, angleDeg);
        }
    }
}

int RobotController::applyAll() {
    int ret = 0;
    for (auto& controller : servoControllers_) {
        int r = controller.apply();
        if (r != 0) ret = r;  // Capture any error, continue applying others
    }
    return ret;
}

void RobotController::updateKinematicsAndApply() {
    // Update all inverse kinematics based on current foot targets and body pose
    updateAllIK();

    // Update servo angles based on IK results
    updateServoAnglesFromIK();

    // Apply servo angles to hardware (send I2C commands)
    int ret = applyAll();
    if (ret != 0) {
        // Handle error if needed
        printf("Error applying servo commands: %d\n", ret);
    }

    // Update last pushed foot targets with the current world foot targets
    for (size_t i = 0; i < kNumLegs; ++i) {
        footTargetsLastPushedWorld_[i] = footTargetsWorld_[i];
    }
}



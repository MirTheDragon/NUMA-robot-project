#include "RobotController.hpp"
#include <cstdio>
#include <iostream>


RobotController::RobotController(int legCount)
    // Initialize the robot body
    : legCount_(legCount),
    legs{
        ThreePointLeg(0.f),   // Right leg angle
        ThreePointLeg(60.f),   // Front Right leg angle
        ThreePointLeg(120.f),  // Front Left leg angle
        ThreePointLeg(180.f),  // Left leg angle
        ThreePointLeg(240.f), // Back Left leg angle
        ThreePointLeg(300.f)   // Back Right leg angle
      }
    , servoControllers_{
        ServoController(1, 0x40, 200),
        ServoController(1, 0x41, 200),
        ServoController(1, 0x42, 200)
      }
{
    initServoMappingsAndConfigs(legCount);
}

void RobotController::initServoMappingsAndConfigs(int legCount) {

    // --- Face Pan, Tilt, and Light Configuration ---
    faceMappings_ = {
        .pan        = {0, 14},   // Controller 0, Channel 14
        .tilt       = {0, 15},   // Controller 0, Channel 15
        .lightRight = {0, 12},   // Controller 0, Channel 12
        .lightLeft  = {0, 13}    // Controller 0, Channel 13
    };

    // Face pan: -70 to +70 degrees, rest at 0
    Face.pan = FaceServo{ 0.f, -70.f, 70.f, 0.f, 0.f };

    // Face tilt: 20 to 60 degrees, rest at +40, retract to -60
    Face.tilt = FaceServo{ 40.f, 20.f, 60.f, 40.f, -60.f };

    // Set servo configs
    servoControllers_[0].setServoConfig(faceMappings_.pan.channelIndex,
        ServoConfig("Face_Pan", 500, 2500, 180.f, -70.f, 70.f));

    servoControllers_[0].setServoConfig(faceMappings_.tilt.channelIndex,
        ServoConfig("Face_Tilt", 500, 2500, 180.f, -60.f, 60.f));  // supports retraction

    servoControllers_[0].setServoConfig(faceMappings_.lightRight.channelIndex,
        ServoConfig("Face_Light_R", 200, 0.0f));
    
    servoControllers_[0].setServoConfig(faceMappings_.lightLeft.channelIndex,
        ServoConfig("Face_Light_L", 200, 0.0f));



    // -- Leg joints configuration --
    if (legCount > 6) legCount = 6; // max legs
    if (legCount < 0) legCount = 0; // min legs

    // For each leg up to legCount, initialize it individually
    if (legCount == 0) {
        // --- Leg 0 (Virtual test leg, 0 deg, only initialized by test programs) ---
        legCount_ = 1; // Ensure at least one leg is initialized for testing

        legs[0] = ThreePointLeg(0.f);
        servoMappings_[0][0] = {0, 0};  // Hip Horizontal
        servoMappings_[0][1] = {0, 1};  // Hip Vertical
        servoMappings_[0][2] = {0, 2};  // Knee

        servoControllers_[0].setServoConfig(0, ServoConfig("Leg0_HipHorizontal", 500, 2500, 270.f, -135.f, 135.f));
        servoControllers_[0].setServoConfig(1, ServoConfig("Leg0_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[0].setServoConfig(2, ServoConfig("Leg0_Knee", 500, 2500, 270.f, -135.f, 135.f));
    }
    if (legCount > 0) {
        // --- Leg 1 (Right Middle, 0 deg) ---
        legs[0] = ThreePointLeg(0.f);
        servoMappings_[0][0] = {1, 0};  // Hip Horizontal
        servoMappings_[0][1] = {2, 0};  // Hip Vertical
        servoMappings_[0][2] = {1, 1};  // Knee

        servoControllers_[1].setServoConfig(0, ServoConfig("Leg0_HipHorizontal", 500, 2500, 270.f, -135.f, 135.f));
        servoControllers_[2].setServoConfig(0, ServoConfig("Leg0_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[1].setServoConfig(1, ServoConfig("Leg0_Knee", 500, 2500, 270.f, -135.f, 135.f));
    }
    if (legCount > 1) {
        // --- Leg 2 (Right Front, 60 deg) ---
        legs[1] = ThreePointLeg(60.f);
        servoMappings_[1][0] = {1, 2};  // Hip Horizontal
        servoMappings_[1][1] = {2, 1};  // Hip Vertical
        servoMappings_[1][2] = {1, 3};  // Knee

        servoControllers_[1].setServoConfig(2, ServoConfig("Leg1_HipHorizontal", 500, 2500, 270.f, -135.f, 135.f));
        servoControllers_[2].setServoConfig(1, ServoConfig("Leg1_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[1].setServoConfig(3, ServoConfig("Leg1_Knee", 500, 2500, 270.f, -135.f, 135.f));
    }
    if (legCount > 2) {
        // --- Leg 3 (Left Front, 120 deg) ---
        legs[2] = ThreePointLeg(120.f);
        servoMappings_[2][0] = {1, 4};  // Hip Horizontal
        servoMappings_[2][1] = {2, 2};  // Hip Vertical
        servoMappings_[2][2] = {1, 5};  // Knee

        servoControllers_[1].setServoConfig(4, ServoConfig("Leg2_HipHorizontal", 500, 2500, 270.f, -135.f, 135.f));
        servoControllers_[2].setServoConfig(2, ServoConfig("Leg2_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[1].setServoConfig(5, ServoConfig("Leg2_Knee", 500, 2500, 270.f, -135.f, 135.f));
    }
    if (legCount > 3) {
        // --- Leg 4 (Left Middle, 180 deg) ---
        legs[3] = ThreePointLeg(180.f);
        servoMappings_[3][0] = {1, 8};  // Hip Horizontal
        servoMappings_[3][1] = {2, 8};  // Hip Vertical
        servoMappings_[3][2] = {1, 9};  // Knee

        servoControllers_[1].setServoConfig(8, ServoConfig("Leg3_HipHorizontal", 500, 2500, 270.f, -135.f, 135.f));
        servoControllers_[2].setServoConfig(8, ServoConfig("Leg3_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[1].setServoConfig(9, ServoConfig("Leg3_Knee", 500, 2500, 270.f, -135.f, 135.f));
    }
    if (legCount > 4) {
        // --- Leg 5 (Left Back, 240 deg) ---
        legs[4] = ThreePointLeg(240.f);
        servoMappings_[4][0] = {1, 10};  // Hip Horizontal
        servoMappings_[4][1] = {2, 9};  // Hip Vertical
        servoMappings_[4][2] = {1, 11};  // Knee

        servoControllers_[1].setServoConfig(10, ServoConfig("Leg4_HipHorizontal", 500, 2500, 270.f, -135.f, 135.f));
        servoControllers_[2].setServoConfig(9, ServoConfig("Leg4_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[1].setServoConfig(11, ServoConfig("Leg4_Knee", 500, 2500, 270.f, -135.f, 135.f));
    }
    if (legCount > 5) {
        // --- Leg 6 (Right Back, 300 deg) ---
        legs[5] = ThreePointLeg(300.f);
        servoMappings_[5][0] = {1, 12};  // Hip Horizontal
        servoMappings_[5][1] = {2, 10};  // Hip Vertical
        servoMappings_[5][2] = {1, 13};  // Knee

        servoControllers_[1].setServoConfig(12, ServoConfig("Leg5_HipHorizontal", 500, 2500, 270.f, -135.f, 135.f));
        servoControllers_[2].setServoConfig(10, ServoConfig("Leg5_HipVertical", 500, 2500, 180.f, -90.f, 90.f));
        servoControllers_[1].setServoConfig(13, ServoConfig("Leg5_Knee", 500, 2500, 270.f, -135.f, 135.f));
    }
}


void RobotController::updateAllIK() {
    // NEW: update face strobe state
    Face.updateStrobeState();

    for (size_t i = 0; i < legCount_; ++i) {
        // Convert stored foot target in world coordinates to body coordinates
        Vec3 footInBody = Body.worldToLegPlaneCoords(footTargetsWorld_[i]); // you need footTargetsWorld_ to hold world targets
        
        /*
        // Assuming `Vec3 bodyPos` is the position transformed into the body coordinate system
        std::cout << "Leg " << i
                << " foot target in body coords: ("
                << footInBody.x << ", "
                << footInBody.y << ", "
                << footInBody.z << ")\n";

        */        
        // Set the leg's foot target to leg coordinates for IK
        legs[i].footTarget = footInBody;
        
        // Solve IK for the leg with this foot target
        legs[i].solveIK();
    }
}


void RobotController::updateServoAnglesFromIK() {
    for (size_t legIndex = 0; legIndex < legCount_; ++legIndex) {
        const auto& leg = legs[legIndex];

        for (size_t joint = 0; joint < 3; ++joint) {
            const ServoMapping& mapping = servoMappings_[legIndex][joint];
            ServoController& controller = servoControllers_[mapping.controllerIndex];

            if (!controller.isInitialized()) {
                std::cerr << "Warning: ServoController " << mapping.controllerIndex
                          << " not initialized, skipping setServoAngle for leg " << legIndex
                          << ", joint " << joint << "\n";
                continue;
            }

            float angleRad = 0.0f;
            if (joint == 0)
                angleRad = leg.servoHipHorizontalRad;
            else if (joint == 1)
                angleRad = leg.servoHipVerticalRad;
            else if (joint == 2)
                angleRad = leg.servoKneeRad;

            float angleDeg = angleRad * 180.0f / PI;

            controller.setServoAngle(mapping.channelIndex, angleDeg);
        }
    }
}


void RobotController::updateServoAnglesFromBodyState() {
    // Light-based head retract logic
    if (Face.lightMode != LightMode::Off && Face.lightBrightness > 0.f) {
        Face.tilt.retract();
    } else {
        Face.tilt.isRetracted = false;
    }

    // Face pan
    if (servoControllers_[faceMappings_.pan.controllerIndex].isInitialized()) {
        float panDeg = Face.pan.getClampedPosition();
        servoControllers_[faceMappings_.pan.controllerIndex].setServoAngle(
            faceMappings_.pan.channelIndex, panDeg);
    }

    // Face tilt
    if (servoControllers_[faceMappings_.tilt.controllerIndex].isInitialized()) {
        float tiltDeg = Face.tilt.getClampedPosition();
        if ( Face.tilt.isRetracted) tiltDeg = Face.tilt.retractDeg;
        servoControllers_[faceMappings_.tilt.controllerIndex].setServoAngle(
            faceMappings_.tilt.channelIndex, tiltDeg);
    }

    // Face light — only active when face is retracted
    float brightness = Face.getLightOutput();

    if (servoControllers_[faceMappings_.lightLeft.controllerIndex].isInitialized()) {
        servoControllers_[faceMappings_.lightLeft.controllerIndex].setPwmPercent(
            faceMappings_.lightLeft.channelIndex, brightness);
    }

    if (servoControllers_[faceMappings_.lightRight.controllerIndex].isInitialized()) {
        servoControllers_[faceMappings_.lightRight.controllerIndex].setPwmPercent(
            faceMappings_.lightRight.channelIndex, brightness);
    }
}



int RobotController::applyAll() {
    int overallStatus = 0;
    for (size_t i = 0; i < servoControllers_.size(); ++i) {
        if (servoControllers_[i].isInitialized()) {  // You’d need to add this bool flag in ServoController
            int r = servoControllers_[i].apply();
            if (r != 0) overallStatus = r;
        } else {
            std::cerr << "Skipping ServoController " << i << " because it is not initialized\n";
        }
    }
    return overallStatus;
}


void RobotController::updateKinematicsAndApply() {
    // ✅ Smooth orientation first
    Body.updateSmoothedPose();


    // Update all inverse kinematics based on current foot targets and body pose
    updateAllIK();
    updateServoAnglesFromIK(); // Update servo angles based on IK results
    updateServoAnglesFromBodyState();   // Apply face servos and lights

    // Apply servo angles to hardware (send I2C commands)
    int ret = applyAll();
    if (ret != 0) {
        // Handle error if needed
        printf("Error applying servo commands: %d\n", ret);
    }

    // Update last pushed foot targets with the current world foot targets
    for (size_t i = 0; i < legCount_; ++i) {
        footTargetsLastPushedWorld_[i] = footTargetsWorld_[i];
    }
}


// Return reference to leg by index
ThreePointLeg& RobotController::getLeg(size_t legIndex) {
    return legs[legIndex];
}

// Initialize hardware etc. (implement as needed)
int RobotController::initialize() {
    bool anyInitialized = false;

    for (size_t i = 0; i < servoControllers_.size(); ++i) {
        int ret = servoControllers_[i].initialize();
        if (ret != 0) {
            std::cerr << "Warning: Failed to initialize ServoController " << i
                      << " at I2C addr 0x" << std::hex << (int)servoControllers_[i].getDeviceAddress() << std::dec << "\n";
            servoControllers_[i].setInitialized(false);  // You need to implement this setter and a bool flag in ServoController
        } else {
            servoControllers_[i].setInitialized(true);
            anyInitialized = true;
        }
    }

    if (!anyInitialized) {
        std::cerr << "Failed to initialize any servo controllers\n";
        return -1;
    }

    // Initialize other parts of robot here if needed

    return 0;
}

// Set foot target for a specific leg
void RobotController::setFootTarget(size_t legIndex, const Vec3& target) {
    if (legIndex < legCount_) {
        footTargetsWorld_[legIndex] = target;
    }
}

const ServoMapping& RobotController::getServoMapping(size_t legIndex, size_t jointIndex) const {
    return servoMappings_[legIndex][jointIndex];
}
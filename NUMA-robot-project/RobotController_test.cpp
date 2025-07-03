#include <iostream>
#include <thread>
#include <chrono>
#include "RobotController.hpp"

int main() {
    RobotController robot(1);  // Initialize robot with 1 leg for testing
    
    // Set all servo gearings to 1 for direct angle mapping
    //robot.getLeg(0).hipHorizontalGearRatio = 1.0f;
    //robot.getLeg(0).hipVerticalGearRatio = 1.0f;
    //robot.getLeg(0).kneeGearRatio = 1.0f;

    // Initialize hardware and configs
    if (robot.initialize() != 0) {
        std::cerr << "Failed to initialize robot\n";
        return 1;
    }

    const float RAD_TO_DEG = 180.0f / 3.14159265358979323846f;

    // Define some foot target positions in world coordinates (cm)
    std::vector<Vec3> footPositions = {
        {34.f, 0.f, 0.f},
        {28.f, -5.f, 0.f},
        {28.f, 5.f, 0.f},
        {34.f, 0.f, 0.f},
        {30.f, 0.f, -5.f},
        {20.f, 0.f, -5.f},
        {30.f, 0.f, -5.f},
        {40.f, 0.f, -5.f},
        {30.f, 0.f, -5.f},
        {30.f, 10.f, -5.f},
        {30.f, 0.f, -5.f},
        {30.f, -10.f, -5.f}
    };

    size_t currentTargetIndex = 0;

    while (true) {
        std::cout << "=== Setting new foot target index " << currentTargetIndex << " ===\n";
        
        Vec3 worldTarget = footPositions[currentTargetIndex];
        std::cout << "Foot target (world coords): x=" << worldTarget.x
                  << ", y=" << worldTarget.y
                  << ", z=" << worldTarget.z << "\n";

        robot.setFootTarget(0, worldTarget);
        // Update IKs for all legs
        robot.updateAllIK();

        // Access leg for debug
        const auto& leg = robot.getLeg(0);

        

        // Convert foot target to body coords and print
        std::cout << "Foot target (body coords with leg plane offset): x=" << leg.footTarget.x
                  << ", y=" << leg.footTarget.y
                  << ", z=" << leg.footTarget.z << "\n";

        // Convert to leg coords (for debug)
        std::cout << "Foot target (leg coords): x=" << leg.footInLegTarget.x
                  << ", y=" << leg.footInLegTarget.y
                  << ", z=" << leg.footInLegTarget.z << "\n";


        std::cout << "Raw IK joint angles:\n"
                  << "  Hip Horizontal: " << leg.rawHipHorizontalRad << " rad (" << leg.rawHipHorizontalRad * RAD_TO_DEG << " deg)\n"
                  << "  Hip Vertical:   " << leg.rawHipVerticalRad << " rad (" << leg.rawHipVerticalRad* RAD_TO_DEG << " deg)\n"
                  << "  Knee:           " << leg.rawKneeRad << " rad (" << leg.rawKneeRad * RAD_TO_DEG << " deg)\n";

        // Update servo angles from IK
        robot.updateServoAnglesFromIK();

        std::cout << "Final servo angles (after gear ratio and offset):\n"
                  << "  Servo Hip Horizontal: " << leg.servoHipHorizontalRad << " rad (" << leg.servoHipHorizontalRad * RAD_TO_DEG << " deg)\n"
                  << "  Servo Hip Vertical:   " << leg.servoHipVerticalRad << " rad (" << leg.servoHipVerticalRad * RAD_TO_DEG << " deg)\n"
                  << "  Servo Knee:           " << leg.servoKneeRad << " rad (" << leg.servoKneeRad * RAD_TO_DEG << " deg)\n";

        // Apply servo angles to controllers
        int ret = robot.applyAll();
        if (ret != 0) {
            std::cerr << "Error applying servo angles: code " << ret << "\n";
        } else {
            std::cout << "Servo angles applied successfully.\n";
        }

        // Wait 10 seconds before next update
        std::this_thread::sleep_for(std::chrono::seconds(1));

        currentTargetIndex = (currentTargetIndex + 1) % footPositions.size();
    }

    return 0;
}

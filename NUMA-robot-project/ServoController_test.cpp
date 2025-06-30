#include "ServoController.hpp"   // your class header
#include <iostream>
#include <thread>
#include <chrono>
#include <array>

int main() {
    ServoController controller(1, 0x43, 200);

    if (controller.initialize() != 0) {
        std::cerr << "Failed to initialize ServoController\n";
        return 1;
    }

    // Define your servos with names and configs bound to channels 14 and 15
    ServoConfig servoA("DS3240_270", 500, 2500, 270.0f, -90.0f, 90.0f);
    ServoConfig servoB("MG90S_180", 500, 2500, 180.0f, -90.0f, 90.0f);

    constexpr size_t channelA = 14;
    constexpr size_t channelB = 15;

    controller.setServoConfig(channelA, servoA);
    controller.setServoConfig(channelB, servoB);

    // Lookup channels by name (optional, but good to illustrate)
    int chA = controller.findServoByName("DS3240_270");
    int chB = controller.findServoByName("MG90S_180");

    if (chA < 0 || chB < 0) {
        std::cerr << "Servo names not found!\n";
        return 1;
    }

    // Sweep angles
    std::array<float, 3> sweepAngles = { -90.0f, 0.0f, 90.0f };

    constexpr float stepDegrees = 0.2f;          // angle increment per step
    constexpr int delayMs = 10;                   // delay per step for smoothness (~66Hz update)
    constexpr int delayS = 2000;                   // delay per fast step

    float minAngleA = controller.getServoConfig(chA).angleMinDeg;
    float maxAngleA = controller.getServoConfig(chA).angleMaxDeg;
    float minAngleB = controller.getServoConfig(chB).angleMinDeg;
    float maxAngleB = controller.getServoConfig(chB).angleMaxDeg;

    while (true) {
        // Sweep up
        for (float angle = minAngleA; angle <= maxAngleA; angle += stepDegrees) {
            float angleB = minAngleB + (angle - minAngleA) * (maxAngleB - minAngleB) / (maxAngleA - minAngleA);

            if (controller.setServoAngle(chA, angle) != 0 ||
                controller.setServoAngle(chB, angleB) != 0) {
                std::cerr << "Failed to set servo angles\n";
                return 1;
            }

            if (controller.apply() != 0) {
                std::cerr << "Failed to apply servo PWM values\n";
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        }

        /*
        // Sweep down
        for (float angle = maxAngleA; angle >= minAngleA; angle -= stepDegrees) {
            float angleB = minAngleB + (angle - minAngleA) * (maxAngleB - minAngleB) / (maxAngleA - minAngleA);

            if (controller.setServoAngle(chA, angle) != 0 ||
                controller.setServoAngle(chB, angleB) != 0) {
                std::cerr << "Failed to set servo angles\n";
                return 1;
            }

            if (controller.apply() != 0) {
                std::cerr << "Failed to apply servo PWM values\n";
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        }
        */

        // Quick pass through the 3 angles
        for (float angle : sweepAngles) {
            std::cout << "Quick pass to angle: " << angle << " degrees\n";
            if (controller.setServoAngle(chA, angle) != 0 ||
                controller.setServoAngle(chB, angle) != 0) return 1;
            if (controller.apply() != 0) return 1;
            std::this_thread::sleep_for(std::chrono::milliseconds(delayS));
        }
    }

    return 0;
}

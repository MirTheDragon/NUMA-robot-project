#include "ServoController.hpp"
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

    ServoConfig servoA("DS3240_270", 500, 2500, 270.0f, -90.0f, 90.0f);
    ServoConfig servoB("MG90S_180", 500, 2500, 180.0f, -90.0f, 90.0f);

    // Channel assignments
    constexpr size_t chanA1 = 0;
    constexpr size_t chanA2 = 1;
    constexpr size_t chanB1 = 2;
    constexpr size_t chanB2 = 3;

    controller.setServoConfig(chanA1, servoA);
    controller.setServoConfig(chanA2, servoA);
    controller.setServoConfig(chanB1, servoB);
    controller.setServoConfig(chanB2, servoB);

    // Angle sequence: min angle, 0, max angle
    std::array<float, 3> angles;

    angles[0] = servoA.angleMinDeg;  // -90.0f
    angles[1] = 0.0f;
    angles[2] = servoA.angleMaxDeg;  // 90.0f

    while (true) {
        for (float angle : angles) {
            std::cout << "Setting servos to angle: " << angle << " degrees" << std::endl;

            if (controller.setServoAngle(chanA1, angle) != 0 ||
                controller.setServoAngle(chanA2, angle) != 0 ||
                controller.setServoAngle(chanB1, angle) != 0 ||
                controller.setServoAngle(chanB2, angle) != 0) {
                std::cerr << "Failed to set servo angles\n";
                return 1;
            }

            if (controller.apply() != 0) {
                std::cerr << "Failed to apply servo PWM values\n";
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }

    return 0;
}

#include "ServoController.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <array>

int main() {
    ServoController controller(1, 0x40, 200);

    if (controller.initialize() != 0) {
        std::cerr << "Failed to initialize ServoController\n";
        return 1;
    }

    ServoConfig servoA("DS3240_270", 500, 2500, 270.0f, -90.0f, 90.0f);
    ServoConfig servoB("MG90S_180", 500, 2500, 180.0f, -90.0f, 90.0f);

    // Channel assignments
    constexpr size_t chanA1 = 0;
    constexpr size_t chanA2 = 1;
    constexpr size_t chanA3 = 2;
    constexpr size_t chanA4 = 3;
    constexpr size_t chanA5 = 4;
    constexpr size_t chanA6 = 5;
    constexpr size_t chanA7 = 6;
    constexpr size_t chanA8 = 7;

    
    // Channel assignments
    constexpr size_t chanB1 = 8;
    constexpr size_t chanB2 = 9;
    constexpr size_t chanB3 = 10;
    constexpr size_t chanB4 = 11;
    constexpr size_t chanB5 = 12;
    constexpr size_t chanB6 = 13;
    constexpr size_t chanB7 = 14;
    constexpr size_t chanB8 = 15;

    controller.setServoConfig(chanA1, servoA);
    controller.setServoConfig(chanA2, servoA);
    controller.setServoConfig(chanA3, servoA);
    controller.setServoConfig(chanA4, servoA);
    controller.setServoConfig(chanA5, servoA);
    controller.setServoConfig(chanA6, servoA);
    controller.setServoConfig(chanA7, servoA);
    controller.setServoConfig(chanA8, servoA);
    controller.setServoConfig(chanB1, servoB);
    controller.setServoConfig(chanB2, servoB);
    controller.setServoConfig(chanB3, servoB);
    controller.setServoConfig(chanB4, servoB);
    controller.setServoConfig(chanB5, servoB);
    controller.setServoConfig(chanB6, servoB);
    controller.setServoConfig(chanB7, servoB);
    controller.setServoConfig(chanB8, servoB);

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
                controller.setServoAngle(chanA3, angle) != 0 ||
                controller.setServoAngle(chanA4, angle) != 0 ||
                controller.setServoAngle(chanA5, angle) != 0 ||
                controller.setServoAngle(chanA6, angle) != 0 ||
                controller.setServoAngle(chanA7, angle) != 0 ||
                controller.setServoAngle(chanA8, angle) != 0 ||
                controller.setServoAngle(chanB1, angle) != 0 ||
                controller.setServoAngle(chanB2, angle) != 0 ||
                controller.setServoAngle(chanB3, angle) != 0 ||
                controller.setServoAngle(chanB4, angle) != 0 ||
                controller.setServoAngle(chanB5, angle) != 0 ||
                controller.setServoAngle(chanB6, angle) != 0 ||
                controller.setServoAngle(chanB7, angle) != 0 ||
                controller.setServoAngle(chanB8, angle) != 0) {
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

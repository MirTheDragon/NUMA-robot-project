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

    // Servo configurations
    ServoConfig knee("DS3240_270", 500, 2500, 270.0f, -135.0f, 135.0f);
    ServoConfig shoulder("MG90S_180", 500, 2500, 180.0f, -90.0f, 90.0f);
    ServoConfig head("DS3235_180", 500, 2500, 180.0f, -70.0f, 70.0f);
    ServoConfig light("pwm_led", 200, 0.0f);  // raw PWM mode with 0% initial duty

    // Assign configs to channels
    std::array<ServoConfig, 16> configs = {
        knee, knee, knee, knee,
        shoulder, shoulder, shoulder, shoulder,
        shoulder, shoulder, light, light,
        light, light, head, head
    };

    for (size_t i = 0; i < configs.size(); ++i) {
        controller.setServoConfig(i, configs[i]);
    }

    std::array<float, 3> angleSteps = {-90.0f, 0.0f, 90.0f};
    std::array<float, 3> pwmSteps = {0.2f, 0.0f, 1.0f};

    while (true) {
        for (int i = 0; i < 3; ++i) {
            std::cout << "\n--- Step " << i + 1 << " ---" << std::endl;

            for (size_t ch = 0; ch < configs.size(); ++ch) {
                const auto& cfg = configs[ch];

                if (cfg.rawPwmMode) {
                    controller.setPwmPercent(ch, pwmSteps[i]);
                    std::cout << "Channel " << ch << ": PWM = " << (pwmSteps[i] * 100.0f) << "%" << std::endl;
                } else {
                    controller.setServoAngle(ch, angleSteps[i]);
                    std::cout << "Channel " << ch << ": Angle = " << angleSteps[i] << "°" << std::endl;
                }
            }

            if (controller.apply() != 0) {
                std::cerr << "Failed to apply servo PWM values\n";
                return 1;
            }

            std::this_thread::sleep_for(std::chrono::seconds(4));
        }
    }

    return 0;
}
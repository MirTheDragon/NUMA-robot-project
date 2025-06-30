extern "C" {
#include "PCA9685.h"
}

#include <iostream>
#include <chrono>
#include <thread>

int main() {
    int fd = PCA9685_openI2C(1, 0x43);  // I2C bus 1, address 0x43

    if (fd < 0) {
        std::cerr << "Failed to open I2C device" << std::endl;
        return 1;
    }

    if (PCA9685_initPWM(fd, 0x43, 50) != 0) {
        std::cerr << "Failed to init PCA9685 at 0x43" << std::endl;
        return 1;
    }

    // Read back prescale register to verify freq set
    unsigned char prescaleVal = 0;
    unsigned char mode1Val = 0;
    // Registers from config or datasheet:
    const unsigned char PRESCALE_REG = 0xFE;
    const unsigned char MODE1_REG = 0x00;

    // Read prescale
    if (_PCA9685_readI2CReg(fd, 0x43, PRESCALE_REG, 1, &prescaleVal) == 0) {
        std::cout << "Prescale register after freq set: 0x" << std::hex << (int)prescaleVal << std::dec << std::endl;
    } else {
        std::cerr << "Failed to read prescale register" << std::endl;
    }

    // Read MODE1
    if (_PCA9685_readI2CReg(fd, 0x43, MODE1_REG, 1, &mode1Val) == 0) {
        std::cout << "MODE1 register after freq set: 0x" << std::hex << (int)mode1Val << std::dec << std::endl;
    } else {
        std::cerr << "Failed to read MODE1 register" << std::endl;
    }

    std::cout << "Starting servo test loop on all channels..." << std::endl;

    unsigned int pulseLengths[] = {204, 307, 410}; // approx 1000, 1500, 2000 us in 12-bit counts

    while (true) {
        for (auto pulse : pulseLengths) {
            std::cout << "Setting all servos to pulse count: " << pulse << std::endl;

            // Write PWM values to all channels
            bool allSuccess = true;
            for (int ch = 0; ch < _PCA9685_CHANS; ch++) {
                unsigned char reg = 0x06 + 4 * ch;
                if (PCA9685_setPWMVal(fd, 0x43, reg, 0, pulse) != 0) {
                    std::cerr << "Failed to set PWM on channel " << ch << std::endl;
                    allSuccess = false;
                }
            }

            if (allSuccess) {
                std::cout << "All PWM writes successful. Verifying by reading back values..." << std::endl;
                // Read back and verify each channel's PWM OFF value
                for (int ch = 0; ch < _PCA9685_CHANS; ch++) {
                    unsigned char reg = 0x06 + 4 * ch;
                    unsigned int onVal = 0, offVal = 0;
                    if (PCA9685_getPWMVal(fd, 0x43, reg, &onVal, &offVal) == 0) {
                        std::cout << "Channel " << ch << ": ON=" << onVal << ", OFF=" << offVal;
                        if (offVal == pulse) {
                            std::cout << " [OK]";
                        } else {
                            std::cout << " [Mismatch!]";
                        }
                        std::cout << std::endl;
                    } else {
                        std::cerr << "Failed to read PWM on channel " << ch << std::endl;
                    }
                }
            } else {
                std::cerr << "Some PWM writes failed, skipping verification." << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    return 0;
}

#include "PCA9685_library.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>  // for exit()

// GPIO helpers
void PCA9685::gpioExport(int pin) {
    std::ofstream exportFile("/sys/class/gpio/export");
    if (exportFile.is_open()) {
        exportFile << pin;
        exportFile.close();
    }
}

void PCA9685::gpioUnexport(int pin) {
    std::ofstream unexportFile("/sys/class/gpio/unexport");
    if (unexportFile.is_open()) {
        unexportFile << pin;
        unexportFile.close();
    }
}

void PCA9685::gpioSetDirection(int pin, const std::string& direction) {
    std::ofstream dirFile("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction");
    if (dirFile.is_open()) {
        dirFile << direction;
        dirFile.close();
    }
}

void PCA9685::gpioWrite(int pin, bool value) {
    std::ofstream valFile("/sys/class/gpio/gpio" + std::to_string(pin) + "/value");
    if (valFile.is_open()) {
        valFile << (value ? "1" : "0");
        valFile.close();
    }
}

// Enable outputs (OE pin low = active)
void PCA9685::enableOutputsGPIO() {
    gpioExport(gpio_oe_pin);
    gpioSetDirection(gpio_oe_pin, "out");
    gpioWrite(gpio_oe_pin, false); // OE active low = enabled
}

// Disable outputs (OE pin high = inactive)
void PCA9685::disableOutputsGPIO() {
    gpioWrite(gpio_oe_pin, true);  // Disable outputs by setting OE pin high
}

// Initialization function for PCA9685 chip
void PCA9685::init() {
    // Wake up chip by resetting MODE1 register to 0
    writeRegister(0x00, 0x00); // MODE1 = 0x00

    // Set MODE2 to totem pole outputs (default is 0x04)
    writeRegister(0x01, 0x04); // MODE2

    std::this_thread::sleep_for(std::chrono::milliseconds(10)); // small delay

    // Set default PWM frequency (50Hz for servos)
    setPWMFreq(50.0f);

    // Enable outputs via OE pin GPIO
    enableOutputsGPIO();
}

PCA9685::PCA9685(int bus, int addr, int oe_pin) : i2c_addr(addr), fd(-1), gpio_oe_pin(oe_pin) { // default OE GPIO4 (pin 7)
    std::string filename = "/dev/i2c-" + std::to_string(bus);
    fd = open(filename.c_str(), O_RDWR);
    if (fd < 0) {
        perror("Failed to open I2C bus");
        exit(1);
    }
    if (ioctl(fd, I2C_SLAVE, i2c_addr) < 0) {
        perror("Failed to set I2C address");
        exit(1);
    }
}

PCA9685::~PCA9685() {
    disableOutputsGPIO();
    gpioUnexport(gpio_oe_pin);

    if (fd >= 0) close(fd);
}

void PCA9685::writeRegister(uint8_t reg, uint8_t val) {
    std::cout << "Writing register 0x" << std::hex << (int)reg << " value 0x" << std::hex << (int)val << std::endl;
    uint8_t buf[2] = {reg, val};
    int res = write(fd, buf, 2);
    if (res != 2) {
        perror("Failed to write register");
    } else {
        std::cout << "Write success" << std::endl;
    }
}


uint8_t PCA9685::readRegister(uint8_t reg) {
    if (write(fd, &reg, 1) != 1) {
        perror("Failed to set register for read");
        return 0;
    }
    uint8_t val = 0;
    if (read(fd, &val, 1) != 1) {
        perror("Failed to read register");
    }
    return val;
}

void PCA9685::setPWMFreq(float freq) {
    // Formula from datasheet: prescale_val = round(25MHz / (4096 * freq)) - 1
    float prescaleval = 25000000.0f / (4096.0f * freq) - 1.0f;
    uint8_t prescale = static_cast<uint8_t>(prescaleval + 0.5f);

    // Sleep
    uint8_t oldmode = readRegister(0x00);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
    writeRegister(0x00, newmode);
    writeRegister(0xFE, prescale); // PRE_SCALE register
    writeRegister(0x00, oldmode);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    writeRegister(0x00, oldmode | 0x80); // restart
}

void PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t buf[5];
    buf[0] = 0x06 + 4 * channel; // LED0_ON_L
    buf[1] = on & 0xFF;
    buf[2] = on >> 8;
    buf[3] = off & 0xFF;
    buf[4] = off >> 8;
    if (write(fd, buf, 5) != 5) {
        perror("Failed to write PWM");
    }
}

void PCA9685::setServoPulse(uint8_t channel, float pulse_us) {
    // Assuming 50Hz PWM freq (20ms period)
    // pulse_us in microseconds, e.g. 1000 to 2000
    // 12-bit resolution => 4096 steps per 20,000us
    uint16_t pulse_length = static_cast<uint16_t>(pulse_us * 4096 / 20000);
    setPWM(channel, 0, pulse_length);
}

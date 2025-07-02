// PCA9685Module.cpp
#include "ServoController.hpp"
#include "PCA9685.h"
#include <cstdio>
#include <algorithm>
#include <unistd.h> // close()

ServoController::ServoController(uint8_t i2cBus, uint8_t deviceAddress, uint16_t pwmFrequencyHz)
    : i2cBus_(i2cBus),
      deviceAddress_(deviceAddress),
      pwmFrequency_(pwmFrequencyHz),
      fd_(-1)
{
    servoAngles_.fill(0.0f);
    pwmOffCounts_.fill(0);
    // Initialize servo configs to defaults
    for (auto& cfg : servoConfigs_) {
        cfg = ServoConfig{};
    }
}

int ServoController::initialize() {
    fd_ = PCA9685_openI2C(i2cBus_, deviceAddress_);
    if (fd_ < 0) {
        fprintf(stderr, "Failed to open I2C bus %d at addr 0x%02X\n", i2cBus_, deviceAddress_);
        isInitialized_ = false;
        return -1;
    }
    int ret = PCA9685_initPWM(fd_, deviceAddress_, pwmFrequency_);
    if (ret != 0) {
        fprintf(stderr, "Failed to initialize PWM on fd %d addr 0x%02X\n", fd_, deviceAddress_);
        isInitialized_ = false;
        return ret;
    }

    isInitialized_ = true;
    return 0;
}

void ServoController::setServoConfig(size_t channel, const ServoConfig& config) {
    if (channel < 16) {
        servoConfigs_[channel] = config;
    }
}

// In ServoController.cpp
const ServoConfig& ServoController::getServoConfig(size_t channel) const {
    return servoConfigs_[channel];
}

// Returns the channel index of the servo with given name, or -1 if not found
int ServoController::findServoByName(const std::string& name) const {
    for (size_t i = 0; i < servoConfigs_.size(); ++i) {
        if (servoConfigs_[i].name == name)
            return static_cast<int>(i);
    }
    return -1;
}


float ServoController::clampAngle(float angle, const ServoConfig& cfg) const {
    return std::max(cfg.angleMinDeg, std::min(angle, cfg.angleMaxDeg));
}

uint16_t ServoController::pulseWidthToCounts(uint16_t pulseWidthUs) const {
    float periodUs = 1'000'000.0f / pwmFrequency_;
    float counts = (pulseWidthUs / periodUs) * 4096.0f;
    if (counts > 4095.0f) counts = 4095.0f;
    else if (counts < 0.0f) counts = 0.0f;
    return static_cast<uint16_t>(counts);
}

// setServoAngle: just clamp and store angle
int ServoController::setServoAngle(size_t channel, float angleDegrees) {
    if (channel >= 16) return -1;
    const ServoConfig& cfg = servoConfigs_[channel];
    servoAngles_[channel] = clampAngle(angleDegrees, cfg);
    return 0;
}



// apply: compute all pulse widths and send batch
int ServoController::apply() {
    if (fd_ < 0) {
        fprintf(stderr, "Device not initialized\n");
        return -1;
    }

    unsigned int on[16] = {0};
    unsigned int off[16];

    for (size_t i = 0; i < 16; ++i) {
        const ServoConfig& cfg = servoConfigs_[i];
        float clampedAngle = servoAngles_[i];

        float span = cfg.angleMaxDeg - cfg.angleMinDeg;
        float ratio = (span > 0.0f)
            ? (clampedAngle - cfg.angleMinDeg) / span
            : 0.5f;  // default to center if zero span

        // Clamp ratio to 0..1
        if (ratio < 0.0f) ratio = 0.0f;
        else if (ratio > 1.0f) ratio = 1.0f;

        uint16_t pulseWidthUs = static_cast<uint16_t>(
            cfg.pulseMinUs + ratio * (cfg.pulseMaxUs - cfg.pulseMinUs)
        );

        off[i] = pulseWidthToCounts(pulseWidthUs);
    }

    return PCA9685_setPWMVals(fd_, deviceAddress_, on, off);
}
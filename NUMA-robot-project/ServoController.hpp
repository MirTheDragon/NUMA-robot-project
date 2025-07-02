// ServoController.hpp
#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <string>


struct ServoConfig {
    std::string name;        // servo descriptive name
    uint16_t pulseMinUs = 1000;
    uint16_t pulseMaxUs = 2000;
    float angleSpanDeg = 270.0f;
    float angleMinDeg = 0.0f;
    float angleMaxDeg = 270.0f;

    ServoConfig(const std::string& n = "",
                uint16_t pMin = 1000,
                uint16_t pMax = 2000,
                float span = 270.0f,
                float minA = 0.0f,
                float maxA = 270.0f)
      : name(n), pulseMinUs(pMin), pulseMaxUs(pMax),
        angleSpanDeg(span), angleMinDeg(minA), angleMaxDeg(maxA) {}
};

class ServoController {
public:
    uint8_t i2cBus_;
    uint8_t deviceAddress_;
    uint16_t pwmFrequency_;
    int fd_;
    
    ServoController(uint8_t i2cBus, uint8_t deviceAddress, uint16_t pwmFrequencyHz);

    // Initialize the hardware (open I2C, setup PWM frequency)
    int initialize();

    

    void setInitialized(bool val) { isInitialized_ = val; }
    bool isInitialized() const { return isInitialized_; }
    uint8_t getDeviceAddress() const { return deviceAddress_; }

    // Set the servo config for a given channel (0-15)
    void setServoConfig(size_t channel, const ServoConfig& config);

    // In ServoController.hpp
    const ServoConfig& getServoConfig(size_t channel) const;

    // Set the target angle (degrees) for a servo channel, clamped internally
    int setServoAngle(size_t channel, float angleDegrees);

    // Push all servo PWM values to the hardware in one batch
    int apply();

    int findServoByName(const std::string& name) const;

private:
    
    bool isInitialized_ = false;

    std::array<ServoConfig, 16> servoConfigs_;
    std::array<float, 16> servoAngles_;  // current target angles per servo
    std::array<uint16_t, 16> pwmOffCounts_; // cached PWM off counts for hardware write

    uint16_t pulseWidthToCounts(uint16_t pulseWidthUs) const;
    float clampAngle(float angle, const ServoConfig& cfg) const;
};

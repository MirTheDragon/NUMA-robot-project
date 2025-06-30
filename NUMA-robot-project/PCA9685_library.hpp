#pragma once
#include <cstdint>
#include <string>

class PCA9685 {
    int fd;
    int i2c_addr;
    int gpio_oe_pin;  // GPIO pin controlling OE (Output Enable), active low

public:
    PCA9685(int bus, int addr, int oe_pin = 4); // default OE pin is GPIO4 (physical pin 7)
    ~PCA9685();

    void init(); // initialize PCA9685 chip and enable outputs
    void enableOutputsGPIO();
    void disableOutputsGPIO();

    void writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);
    void setPWMFreq(float freq);
    void setPWM(uint8_t channel, uint16_t on, uint16_t off);
    void setServoPulse(uint8_t channel, float pulse_us);

private:
    void gpioExport(int pin);
    void gpioUnexport(int pin);
    void gpioSetDirection(int pin, const std::string& direction);
    void gpioWrite(int pin, bool value);
};

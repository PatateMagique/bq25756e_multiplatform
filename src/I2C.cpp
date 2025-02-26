#include "I2C.h"

// Write a value to a specific DRV8214 register for a given driver address
void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
#ifdef DRV8214_PLATFORM_ARDUINO
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
#elif defined(DRV8214_PLATFORM_STM32)
    uint8_t data[2] = { reg, value };
    HAL_I2C_Master_Transmit(&hi2c1, address << 1, data, 2, HAL_MAX_DELAY);
#endif
}

// Read a single byte from a specific DRV8214 register for a given driver address
uint8_t readRegister(uint8_t address, uint8_t reg) {
#ifdef DRV8214_PLATFORM_ARDUINO
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission(false);  // Send restart condition
    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
#elif defined(DRV8214_PLATFORM_STM32)
    uint8_t data = 0;
    if (HAL_I2C_Master_Transmit(&hi2c1, address << 1, &reg, 1, HAL_MAX_DELAY) == HAL_OK &&
        HAL_I2C_Master_Receive(&hi2c1, address << 1, &data, 1, HAL_MAX_DELAY) == HAL_OK) {
        return data;
    }
    return 0;
#endif
}

// Modify specific bits in a specific DRV8214 register
void modifyRegister(uint8_t address, uint8_t reg, uint8_t mask, bool enable) {
    uint8_t value = readRegister(address, reg);
    if (enable) {
        value |= mask;  // Set bits
    } else {
        value &= ~mask; // Clear bits
    }
    writeRegister(address, reg, value);
}

// Modify specific bits in a specific DRV8214 register using a new value
void modifyRegisterBits(uint8_t address, uint8_t reg, uint8_t mask, uint8_t newValue) {
    uint8_t current = readRegister(address, reg);
    current = (current & ~mask) | (newValue & mask);
    writeRegister(address, reg, current);
}

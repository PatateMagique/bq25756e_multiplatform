/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the bq25756e_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#include "bq25756e_platform_i2c.h"

#ifdef BQ25756E_PLATFORM_STM32
    static I2C_HandleTypeDef* bq25756e_i2c_handle = NULL; // Static pointer to the I2C handle for BQ25756E

    /**
     * @brief Sets the I2C handle for STM32 platforms.
     * @param hi2c Pointer to the I2C_HandleTypeDef structure.
     */
    void bq25756e_i2c_set_handle(I2C_HandleTypeDef* hi2c) {
        bq25756e_i2c_handle = hi2c;
    }
#endif

/**
 * @brief Write an 8-bit value to a specific BQ25756E register.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to write to.
 * @param value The 8-bit value to write.
 */
void bq25756e_i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t value) {
#ifdef BQ25756E_PLATFORM_ARDUINO
    Wire.beginTransmission(device_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
#elif defined(BQ25756E_PLATFORM_STM32)
    if (bq25756e_i2c_handle == NULL) {
        // Handle error: I2C handle not set
        return;
    }
    // For STM32, HAL_I2C_Mem_Write is often preferred for register writes.
    // It sends the register address and then the data.
    HAL_I2C_Mem_Write(bq25756e_i2c_handle, (uint16_t)(device_address << 1), reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
    // Add error handling for HAL_StatusTypeDef if needed
#endif
}

/**
 * @brief Write a 16-bit value to a specific BQ25756 register.
 * BQ25756 expects LSB first, then MSB for the data payload.
 * @param device_address The I2C address of the BQ25756.
 * @param reg The register address to write to.
 * @param value The 16-bit value to write.
 */
void bq25756e_i2c_write_register16(uint8_t device_address, uint8_t reg, uint16_t value) {
#ifdef BQ25756E_PLATFORM_ARDUINO
    Wire.beginTransmission(device_address);
    Wire.write(reg);
    Wire.write(value & 0xFF);          // Write LSB
    Wire.write((value >> 8) & 0xFF);   // Write MSB
    Wire.endTransmission();
#elif defined(BQ25756E_PLATFORM_STM32)
    if (bq25756e_i2c_handle == NULL) {
        // Handle error: I2C handle not set
        return;
    }
    uint8_t data_payload[2];
    data_payload[0] = value & 0xFF;          // LSB
    data_payload[1] = (value >> 8) & 0xFF;   // MSB
    
    // HAL_I2C_Mem_Write will send: START | ADDR+W | REG_ADDR | DATA_PAYLOAD[0] | DATA_PAYLOAD[1] | STOP
    HAL_I2C_Mem_Write(bq25756e_i2c_handle, (uint16_t)(device_address << 1), reg, I2C_MEMADD_SIZE_8BIT, data_payload, 2, HAL_MAX_DELAY);
    // Add error handling for HAL_StatusTypeDef if needed
#endif
}

/**
 * @brief Read an 8-bit value from a specific BQ25756E register.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to read from.
 * @return The 8-bit value read from the register, or 0 on error.
 */
uint8_t bq25756e_i2c_read_register(uint8_t device_address, uint8_t reg) {
#ifdef BQ25756E_PLATFORM_ARDUINO
    Wire.beginTransmission(device_address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { // Send repeated start, check for NACK on address phase
        return 0; // Error condition
    }
    
    if (Wire.requestFrom(device_address, (uint8_t)1) != 1) { // Request 1 byte
        return 0; // Error condition, did not receive 1 byte
    }

    if (Wire.available()) {
        return Wire.read();
    }
    return 0; // Should not happen if requestFrom succeeded and Wire.available() is true
#elif defined(BQ25756E_PLATFORM_STM32)
    if (bq25756e_i2c_handle == NULL) {
        // Handle error: I2C handle not set
        return 0;
    }
    uint8_t data = 0;
    // HAL_I2C_Mem_Read is suitable here.
    // It sends: START | ADDR+W | REG_ADDR | RESTART | ADDR+R | READ_DATA | STOP
    if (HAL_I2C_Mem_Read(bq25756e_i2c_handle, (uint16_t)(device_address << 1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY) == HAL_OK) {
        return data;
    }
    return 0; // Error
#endif
}

/**
 * @brief Read a 16-bit value from a specific BQ25756E register.
 * BQ25756 returns LSB first, then MSB.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to read from.
 * @return The 16-bit value read from the register, or 0 on error.
 */
uint16_t bq25756e_i2c_read_register16(uint8_t device_address, uint8_t reg) {
#ifdef BQ25756E_PLATFORM_ARDUINO
    Wire.beginTransmission(device_address);
    Wire.write(reg);  
    if (Wire.endTransmission(false) != 0) { // Send repeated start
        return 0; // Error
    }

    if (Wire.requestFrom(device_address, (uint8_t)2) != 2) { // Request 2 bytes
        return 0; // Error, did not receive 2 bytes
    }

    if (Wire.available() >= 2) {
        uint8_t lsb = Wire.read();
        uint8_t msb = Wire.read();
        return (static_cast<uint16_t>(msb) << 8) | lsb;
    }
    return 0; // Should not happen
#elif defined(BQ25756E_PLATFORM_STM32)
    if (bq25756e_i2c_handle == NULL) {
        // Handle error: I2C handle not set
        return 0;
    }
    uint8_t data_buffer[2]; // Buffer to store LSB and MSB

    // HAL_I2C_Mem_Read will read 2 bytes starting from the specified register address.
    // The device (BQ25756E) sends LSB then MSB. So, data_buffer[0] will be LSB, data_buffer[1] will be MSB.
    if (HAL_I2C_Mem_Read(bq25756e_i2c_handle, (uint16_t)(device_address << 1), reg, I2C_MEMADD_SIZE_8BIT, data_buffer, 2, HAL_MAX_DELAY) == HAL_OK) {
        // Combine LSB (data_buffer[0]) and MSB (data_buffer[1]) into a 16-bit value
        return (static_cast<uint16_t>(data_buffer[1]) << 8) | data_buffer[0];
    }
    return 0; // Error
#endif
}

/**
 * @brief Modify specific bits in a specific BQ25756E register.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address.
 * @param mask The bitmask to apply.
 * @param enable True to set bits, false to clear bits.
 */
void bq25756e_i2c_modify_register(uint8_t device_address, uint8_t reg, uint8_t mask, bool enable) {
    uint8_t current_value = bq25756e_i2c_read_register(device_address, reg);
    
    uint8_t new_value;
    if (enable) {
        new_value = current_value | mask;  // Set bits
    } else {
        new_value = current_value & ~mask; // Clear bits
    }
    // Only write if the value has changed
    if (new_value != current_value) {
        bq25756e_i2c_write_register(device_address, reg, new_value);
    }
}

/**
 * @brief Modify specific bits in a specific BQ25756E register using a new value for those bits.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address.
 * @param mask The bitmask indicating which bits to modify.
 * @param new_value_for_bits The new value for the bits defined by the mask. Other bits are preserved.
 */
void bq25756e_i2c_modify_register_bits(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value_for_bits) {
    uint8_t current_value = bq25756e_i2c_read_register(device_address, reg);
    
    // Clear the bits defined by the mask in the current value, then OR with the new value (also masked)
    uint8_t new_value = (current_value & ~mask) | (new_value_for_bits & mask);
    
    if (new_value != current_value) {
        bq25756e_i2c_write_register(device_address, reg, new_value);
    }
}

// // Write a value to a specific DRV8214 register for a given driver address
// void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
//     Wire.beginTransmission(address);
//     Wire.write(reg);
//     Wire.write(value);
//     Wire.endTransmission();
// }

// void writeRegister16(uint8_t address, uint8_t reg, uint16_t value) {
//     Wire.beginTransmission(address);
//     Wire.write(reg);
//     Wire.write(value & 0xFF);          // Write LSB
//     Wire.write((value >> 8) & 0xFF);   // Write MSB first
//     Wire.endTransmission();
// }

// uint8_t readRegister(uint8_t address, uint8_t reg) {
//     Wire.beginTransmission(address);
//     Wire.write(reg);
//     // End the transmission with a STOP condition instead of a repeated start
//     // True = send STOP condition after transmission
//     // False = send a repeated start after transmission
//     // If Wire.endTransmission(false) fails but Wire.endTransmission(true) works, the device might not support a repeated start condition.
//     int errorCode = Wire.endTransmission(false);
//     if (errorCode != 0) {
//         return errorCode;  // Return the actual error code
//     }
    
//     // Request a single byte from the device
//     Wire.requestFrom(address, (uint8_t)1);
//     if (Wire.available()) {
//         return Wire.read();
//     }
//     return 0; // Return 0 if no data was received
// }

// uint16_t readRegister16(uint8_t address, uint8_t reg) { 
//     // 0) Send register address
//     Wire.beginTransmission(address);
//     Wire.write(reg);  
//     // 1) End the transmission with a repeated start
//     Wire.endTransmission(false);

//     // 2) Request 2 bytes
//     Wire.requestFrom((int)address, 2);

//     // 3) Read LSB then MSB (typical for TI registers)
//     uint8_t low_bits  = Wire.read();
//     uint8_t high_bits = Wire.read();

//     // 4) Combine into 16-bit
//     return (static_cast<uint16_t>(high_bits) << 8) | low_bits;
// }

// // Modify specific bits in a specific DRV8214 register
// void modifyRegister(uint8_t address, uint8_t reg, uint8_t mask, bool enable) {
//     uint8_t value = readRegister(address, reg);
//     if (enable) {
//         value |= mask;  // Set bits
//     } else {
//         value &= ~mask; // Clear bits
//     }
//     writeRegister(address, reg, value);
// }

// // Modify specific bits in a specific DRV8214 register using a new value
// void modifyRegisterBits(uint8_t address, uint8_t reg, uint8_t mask, uint8_t newValue) {
//     uint8_t current = readRegister(address, reg);
//     current = (current & ~mask) | (newValue & mask);
//     writeRegister(address, reg, current);
// }
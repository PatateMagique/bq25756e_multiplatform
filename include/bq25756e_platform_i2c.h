/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the bq25756e_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#ifndef BQ25756E_PLATFORM_I2C_H
#define BQ25756E_PLATFORM_I2C_H

#include "bq25756e_platform_config.h" // For platform detection

#ifdef BQ25756E_PLATFORM_ARDUINO
    #include <Arduino.h> // Includes Wire.h indirectly for most Arduino cores
    #include <Wire.h>    // Explicit include for clarity and some cores
#endif

#ifdef BQ25756E_PLATFORM_STM32
    // Forward declaration of the I2C_HandleTypeDef.
    // The actual definition will be included in the .cpp file via the main HAL header
    // (e.g., "stm32xxxx_hal.h" or a specific family like "stm32f1xx_hal.h").
    // This keeps the header file lightweight and avoids pulling in large HAL headers here.
    typedef struct I2C_HandleTypeDef I2C_HandleTypeDef; 

    /**
     * @brief Sets the I2C handle for STM32 platforms.
     * This function must be called once during initialization (e.g., in main.c after MX_I2Cx_Init())
     * before any other I2C operations from this library are used.
     * @param hi2c Pointer to the I2C_HandleTypeDef structure configured for the BQ25756E.
     */
    void bq25756e_i2c_set_handle(I2C_HandleTypeDef* hi2c);
#endif

// --- Common I2C Function Declarations ---
// Note: Function names are now prefixed with "bq25756e_i2c_" to match the .cpp file

/**
 * @brief Write an 8-bit value to a specific BQ25756E register.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to write to.
 * @param value The 8-bit value to write.
 */
void bq25756e_i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t value);

/**
 * @brief Write a 16-bit value to a specific BQ25756E register.
 * BQ25756E expects LSB first, then MSB for the data payload.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to write to.
 * @param value The 16-bit value to write.
 */
void bq25756e_i2c_write_register16(uint8_t device_address, uint8_t reg, uint16_t value);

/**
 * @brief Read an 8-bit value from a specific BQ25756E register.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to read from.
 * @return The 8-bit value read from the register, or 0 on error/if no data.
 */
uint8_t bq25756e_i2c_read_register(uint8_t device_address, uint8_t reg);

/**
 * @brief Read a 16-bit value from a specific BQ25756E register.
 * BQ25756E returns LSB first, then MSB.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address to read from.
 * @return The 16-bit value read from the register, or 0 on error/if no data.
 */
uint16_t bq25756e_i2c_read_register16(uint8_t device_address, uint8_t reg);

/**
 * @brief Modify specific bits in a specific BQ25756E register.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address.
 * @param mask The bitmask indicating which bits to affect.
 * @param enable True to set the masked bits, false to clear the masked bits.
 */
void bq25756e_i2c_modify_register(uint8_t device_address, uint8_t reg, uint8_t mask, bool enable);

/**
 * @brief Modify specific bits in a specific BQ25756E register using a new value for those bits.
 * Other bits in the register are preserved.
 * @param device_address The I2C address of the BQ25756E.
 * @param reg The register address.
 * @param mask The bitmask indicating which bits to modify.
 * @param new_value_for_bits The new value for the bits defined by the mask. Only bits within the mask are considered from this value.
 */
void bq25756e_i2c_modify_register_bits(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value_for_bits);

#endif // BQ25756E_PLATFORM_I2C_H
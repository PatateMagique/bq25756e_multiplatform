/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the bq25756e_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#ifndef BQ25756E_PLATFORM_CONFIG_H
#define BQ25756E_PLATFORM_CONFIG_H

// Automatic Platform Detection
#if defined(ESP32) || defined(ESP_PLATFORM) || defined(ARDUINO)
    #define BQ25756E_PLATFORM_ARDUINO
#elif defined(STM32WB5Mxx) || defined(STM32F4xx) || defined(USE_STM32_HAL_DRIVER) || defined(USE_HAL_DRIVER)
    #define BQ25756E_PLATFORM_STM32
#else
    #error "Unsupported platform. Define BQ25756E_PLATFORM_ARDUINO or BQ25756E_PLATFORM_STM32 manually."
#endif

#endif // BQ25756E_PLATFORM_CONFIG_H
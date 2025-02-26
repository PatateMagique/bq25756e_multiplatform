/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the bq25756e_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

// Automatic Platform Detection
#if defined(ARDUINO)
    #define DRV8214_PLATFORM_ARDUINO
#elif defined(STM32)
    #define DRV8214_PLATFORM_STM32
#else
    #error "Unsupported platform. Define DRV8214_PLATFORM_ARDUINO or DRV8214_PLATFORM_STM32 manually."
#endif

#endif // PLATFORM_H

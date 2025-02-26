// Platform.h
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

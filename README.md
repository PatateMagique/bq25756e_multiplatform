# **WORK IN PROGRESS**

# BQ25756E I2C Control Library

## Overview

This library provides a set of straightforward commands to control and configure the **BQ25756E** battery charge controller from Texas Instruments via the I²C interface​. The BQ25756E is a high-performance bidirectional buck-boost battery charge controller featuring wide-range multi-cell (multi-chemistry) battery support and automatic maximum power point tracking (MPPT) for solar charging​, along with additional functionalities such as precise charge voltage and current regulation​, integrated current sensing and battery temperature monitoring​, and comprehensive protection circuitry.

## Features

- **I2C Communication**: Seamlessly interface with the BQ25756E over I2C for efficient motor control.
- **Charging Control**: TBW
- **Configuration Settings**: Access and modify device parameters to suit specific application requirements.
- **Status Monitoring**: Retrieve real-time data on charging status and fault conditions.

## Getting Started

### Prerequisites

- **Hardware**: A microcontroller with I2C capability (e.g., Arduino, ESP32, STM32) and the BQ25756E battery charge controller.
- **Software**: Arduino IDE or PlatformIO for code development and uploading.

### Installation

1. **Clone the Repository using SSH**:
   ```bash
   git clone git@github.com:PatateMagique/bq25756e_multiplatform.git
   ```

2. **Include the Library**: Add the cloned library to your project's libraries directory.

### Usage Example for Arduino/ESP32

In you ``main.cpp``:

```cpp
#include "bq25756e.h"

// BQ25756E Configuration Parameters - Hardware dependent
#define BQ25756E_ADDRESS 0x6A
#define SWITCHING_FREQUENCY 600
#define MAX_CHARGE_CURRENT 10000
#define MAX_INPUT_CURRENT 20000
#define MIN_INPUT_VOLTAGE 4200
#define MAX_INPUT_VOLTAGE 36000

/* ------------------------ BQ25756E Configuration Parameters - Xplore imposed specifications ------------------------ */
// charge current constraints for pre-charge mode between 0V and 20.48V
#define MIN_CHARGE_CURRENT_PRE 250
#define DEFAULT_CHARGE_CURRENT_PRE 500
#define MAX_CHARGE_CURRENT_PRE 1000
#define CURRENT_STEP_PRE 50
// charge current constraints for CC mode between 20.48V and 23V
#define MIN_CHARGE_CURRENT_CC_20 400
#define DEFAULT_CHARGE_CURRENT_CC_20 1000
#define MAX_CHARGE_CURRENT_CC_20 1500
#define CURRENT_STEP_CC_20 50
// charge current constraints for CC mode between 23V and 24V
#define MIN_CHARGE_CURRENT_CC_23 1000
#define DEFAULT_CHARGE_CURRENT_CC_23 1500
#define MAX_CHARGE_CURRENT_CC_23 2000
#define CURRENT_STEP_CC_23 100
// charge current constraints for CC mode between 24V and 28.3V
#define MIN_CHARGE_CURRENT_CC_24 1500
#define DEFAULT_CHARGE_CURRENT_CC_24 2300
#define MAX_CHARGE_CURRENT_CC_24 5500
#define CURRENT_STEP_CC_24 100
// charge current constraints for CV mode at 28.3V
#define MIN_CHARGE_CURRENT_CV 250
#define DEFAULT_CHARGE_CURRENT_CV 500
#define MAX_CHARGE_CURRENT_CV 1050
#define CURRENT_STEP_CV 50

// Creat a BQ25756E object
BQ25756E charger(BQ25756E_ADDRESS, SWITCHING_FREQUENCY, MAX_CHARGE_CURRENT, MAX_INPUT_CURRENT, MIN_INPUT_VOLTAGE, MAX_INPUT_VOLTAGE);

// Create a BQ25756E configuration structure  
BQ25756E_Config chargerConfig = {
    .chargeVoltageLimit = 1506, // Range: 1504mV (28.31V) to 1566 mV(29.48V)
    .chargeCurrentLimit = DEFAULT_CHARGE_CURRENT_CC_24, // Displayed charge current will be lower by around 10%, but the real current will be close to the set value. Range: 0.4A to 10A
    .inputCurrentDPMLimit = 7000, // Range: 0.4A to 20A
    .inputVoltageDPMLimit = 15000, // voltage in mV under which the charger will reduce the input current
    .prechargeCurrentLimit = DEFAULT_CHARGE_CURRENT_PRE, // Range: 0.25A to 10A
    .terminationCurrentLimit = DEFAULT_CHARGE_CURRENT_CV, // Range: 0.25A to 10A
    .terminationControlEnabled = true, // Enable termination current control
    .fastChargeThreshold = 0b11, // 0b00 = 30% x VFB_REG, 0b01 = 55% x VFB_REG, 0b10 = 66.7% x VFB_REG, 0b11 = 71.4% x VFB_REG = 71.4% x 1524 = 1088mV -> fast charge above 20.48V
    .prechargeControlEnabled = true, // Enable pre-charge and trickle charge functions
    .topOffTimer = 0b00, // 0b00 = Disable, 0b01 = 15 minutes, 0b10 = 30 minutes, 0b11 = 45 minutes
    .watchdogTimer = 0b00, // 0b00 = Disable, 0b01 = 40s, 0b10 = 80s, 0b11 = 160s
    .safetyTimerEnabled = false, // disabled
    .safetyTimer = 0b00, // 0b00 = 5h, 0b01 = 8h, 0b10 = 12h, 0b11 = 24h
    .safetyTimerSpeed = false,
    .constantVoltageTimer = 0b0010, // 0b0000 = disable, 0b0001 = 1h, 0b0010 = 2h, ... 0b1111 = 15h
    .autoRechargeThreshold = 0b11, // 0b00 = 93%, 0b01 = 94.3%, 0b10 = 95.2%, 0b11 = 97.6%
    .watchdogTimerResetEnabled = false, 
    .CEPinEnabled = true, // Enable the control of the charger with the switch connected to the CE pin
    .ChargeBehaviorWatchdogExpired = true, // 0b = EN_CHG resets to 0, 1b = EN_CHG resets to 1 when watchdog expires
    .highZModeEnabled = false,
    .batteryLoadEnabled = false, // Disable battery load
    .chargeEnabled = true, // Enable charging
    .enableMPPT = false,
    .verbose = true
};

void setup() {

  // Initialize the I2C bus
  Wire.begin(SDA_PIN, SCL_PIN); // Default I2C bus for the charger

  console->println("\nInitializing BQ25756E charger...\n");
  charger.setDebugStream(console); // Let the charger's library know which stream to use for debug
  charger.init(chargerConfig); // Initialize the charger with the configuration structure
  previousChargeState = charger.getChargeCycleStatus(); // Get the initial charge state
  console->println("\nBQ25756E charger initialized successfully!\n");
}
```


## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Acknowledgments

Special thanks to the open-source community and Texas Instruments for their comprehensive documentation and support.

---

*Note: This library is under active development. Features and implementations are subject to change. Users are encouraged to regularly update and refer to the latest documentation.* 

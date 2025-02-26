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
   git clone git@github.com:PatateMagique/drv8214_multiplatform.git
   ```

2. **Include the Library**: Add the cloned library to your project's libraries directory.

### Usage


```cpp
#include <BQ25756E.h>

// TBW
```


## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

## Acknowledgments

Special thanks to the open-source community and Texas Instruments for their comprehensive documentation and support.

---

*Note: This library is under active development. Features and implementations are subject to change. Users are encouraged to regularly update and refer to the latest documentation.* 

/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the bq25756e_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#ifndef BQ25756E_H
#define BQ25756E_H

#include <stdint.h>

// --- REGISTER DEFINITIONS ---
// (Refer to the datasheet for detailed register information)
#define BQ25756E_REG_CHARGE_VOLTAGE_LIMIT              0x00  // Charge Voltage Limit Register
#define BQ25756E_REG_CHARGE_CURRENT_LIMIT              0x02  // Charge Current Limit Register
#define BQ25756E_REG_INPUT_CURRENT_DPM_LIMIT           0x06  // Input Current DPM Limit Register
#define BQ25756E_REG_INPUT_VOLTAGE_DPM_LIMIT           0x08  // Input Voltage DPM Limit Register
#define BQ25756E_REG_REVERSE_MODE_INPUT_CURRENT_LIMIT    0x0A // Reverse Mode Input Current Limit
#define BQ25756E_REG_REVERSE_MODE_INPUT_VOLTAGE_LIMIT    0x0C // Reverse Mode Input Voltage Limit
#define BQ25756E_REG_PRECHARGE_CURRENT_LIMIT           0x10  // Precharge Current Limit Register
#define BQ25756E_REG_TERMINATION_CURRENT_LIMIT         0x12  // Termination Current Limit Register
#define BQ25756E_REG_PRECHARGE_TERMINATION_CONTROL       0x14  // Precharge & Termination Control
#define BQ25756E_REG_TIMER_CONTROL                     0x15  // Timer Control Register
#define BQ25756E_REG_THREE_STAGE_CHARGE_CONTROL         0x16  // Three-Stage Charge Control
#define BQ25756E_REG_CHARGER_CONTROL                   0x17  // Charger Control Register
#define BQ25756E_REG_PIN_CONTROL                       0x18  // Pin Control Register
#define BQ25756E_REG_POWER_PATH_REVERSE_MODE_CONTROL     0x19  // Power Path & Reverse Mode Control
#define BQ25756E_REG_MPPT_CONTROL                      0x1A  // MPPT Control Register
#define BQ25756E_REG_TS_CHARGING_THRESHOLD_CONTROL      0x1B  // TS Charging Threshold Control
#define BQ25756E_REG_TS_CHARGING_REGION_BEHAVIOR_CONTROL  0x1C  // TS Charging Region Behavior Control
#define BQ25756E_REG_TS_REVERSE_MODE_THRESHOLD_CONTROL   0x1D  // TS Reverse Mode Threshold Control
#define BQ25756E_REG_REVERSE_UNDERVOLTAGE_CONTROL        0x1E  // Reverse Undervoltage Control
#define BQ25756E_REG_VAC_MAX_POWER_POINT_DETECTED        0x1F  // VAC Max Power Point Detected
#define BQ25756E_REG_CHARGER_STATUS_1                  0x21  // Charger Status 1
#define BQ25756E_REG_CHARGER_STATUS_2                  0x22  // Charger Status 2
#define BQ25756E_REG_CHARGER_STATUS_3                  0x23  // Charger Status 3
#define BQ25756E_REG_FAULT_STATUS                      0x24  // Fault Status Register
#define BQ25756E_REG_CHARGER_FLAG_1                    0x25  // Charger Flag 1
#define BQ25756E_REG_CHARGER_FLAG_2                    0x26  // Charger Flag 2
#define BQ25756E_REG_FAULT_FLAG                        0x27  // Fault Flag Register
#define BQ25756E_REG_CHARGER_MASK_1                    0x28  // Charger Mask 1
#define BQ25756E_REG_CHARGER_MASK_2                    0x29  // Charger Mask 2
#define BQ25756E_REG_FAULT_MASK                        0x2A  // Fault Mask Register
#define BQ25756E_REG_ADC_CONTROL                       0x2B  // ADC Control Register
#define BQ25756E_REG_ADC_CHANNEL_CONTROL               0x2C  // ADC Channel Control Register
#define BQ25756E_REG_IAC_ADC                           0x2D  // IAC ADC Register
#define BQ25756E_REG_IBAT_ADC                          0x2F  // IBAT ADC Register
#define BQ25756E_REG_VAC_ADC                           0x31  // VAC ADC Register
#define BQ25756E_REG_VBAT_ADC                          0x33  // VBAT ADC Register
#define BQ25756E_REG_TS_ADC                            0x37  // TS ADC Register
#define BQ25756E_REG_VFB_ADC                           0x39  // VFB ADC Register
#define BQ25756E_REG_GATE_DRIVER_STRENGTH_CONTROL       0x3B  // Gate Driver Strength Control
#define BQ25756E_REG_GATE_DRIVER_DEAD_TIME_CONTROL       0x3C  // Gate Driver Dead Time Control
#define BQ25756E_REG_PART_INFORMATION                  0x3D  // Part Information Register
#define BQ25756E_REG_REVERSE_MODE_BATTERY_DISCHARGE_CURRENT 0x62  // Reverse Mode Battery Discharge Current

// --- BIT MASKS ---
// Example bit masks for the Fault Status Register (0x24)
#define BQ25756E_FAULT_VAC_UV_STAT    0x80  // Bit 7: Input under-voltage
#define BQ25756E_FAULT_VAC_OV_STAT    0x40  // Bit 6: Input over-voltage
#define BQ25756E_FAULT_IBAT_OCP_STAT  0x20  // Bit 5: Battery over-current
#define BQ25756E_FAULT_VBAT_OV_STAT   0x10  // Bit 4: Battery over-voltage
#define BQ25756E_FAULT_TSHUT_STAT     0x08  // Bit 3: Thermal shutdown
#define BQ25756E_FAULT_CHG_TMR_STAT   0x04  // Bit 2: Charge safety timer
#define BQ25756E_FAULT_DRV_OKZ_STAT   0x02  // Bit 1: DRV_SUP pin voltage fault
// Bit 0 is reserved

// Additional bit masks for other registers can be defined here...

// --- ENUMERATIONS ---
// Example charge state enumeration based on Charger Status 1
enum ChargeState {
    CHARGE_STATE_NOT_CHARGING = 0,
    CHARGE_STATE_TRICKLE,
    CHARGE_STATE_PRECHARGE,
    CHARGE_STATE_FASTCHARGE,
    CHARGE_STATE_TAPER,
    CHARGE_STATE_CHARGE_DONE
};

// Additional enums (e.g., for timer settings or MPPT modes) can be added here

// --- CONFIGURATION STRUCTURE ---
struct BQ25756E_Config {
    uint16_t chargeVoltageLimit;       // in mV; valid range and resolution as per datasheet
    uint16_t chargeCurrentLimit;       // in mA; fast charge current regulation limit
    uint16_t inputCurrentLimit;        // in mA; input current DPM limit
    uint16_t prechargeCurrentLimit;    // in mA; precharge current regulation limit
    uint16_t terminationCurrentLimit;  // in mA; termination current limit
    bool enableMPPT;                   // Enable or disable MPPT control
    // Additional configuration parameters can be added as needed
    bool verbose;                      // Enable verbose debug output
};

// --- BQ25756E CLASS ---
class BQ25756E {
private:
    uint8_t address;         // I2C address of the charger
    BQ25756E_Config config;  // Current configuration

    // Private helper functions for I2C communication can be declared here

public:
    // Constructor
    BQ25756E(uint8_t addr) : address(addr) {}

    // Initialization function
    void init(const BQ25756E_Config& cfg);

    // --- Helper Functions ---
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t data);
    uint16_t readRegister16(uint8_t reg);  // For 16-bit registers (Little Endian)

    // --- Status Functions ---
    uint8_t getFaultStatus();
    uint8_t getChargerStatus1();
    uint8_t getChargerStatus2();
    uint8_t getChargerStatus3();

    ChargeState getChargeState();

    // --- Control Functions ---
    void setChargeVoltageLimit(uint16_t voltage_mV);
    void setChargeCurrentLimit(uint16_t current_mA);
    void setInputCurrentLimit(uint16_t current_mA);
    void setPrechargeCurrentLimit(uint16_t current_mA);
    void setTerminationCurrentLimit(uint16_t current_mA);
    
    void enableMPPT();
    void disableMPPT();

    void resetWatchdog();

    // --- ADC Functions ---
    uint16_t getIbatADC();
    uint16_t getVacADC();
    uint16_t getVbatADC();
    uint16_t getTsADC();
    uint16_t getVfbADC();

    // --- Debug Function ---
    void printChargerConfig();
};

#endif // BQ25756E_H

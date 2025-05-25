/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the bq25756e_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#include "BQ25756E.h"

// Initialize the BQ25756E charger with the configuration structure
void BQ25756E::init(const BQ25756E_Config& cfg) {

    // Save the configuration
    config = cfg;

    setChargeVoltageLimit(cfg.chargeVoltageLimit);           // Set the charge voltage limit
    setChargeCurrentLimit(cfg.chargeCurrentLimit);           // Set the charge current limit
    setInputCurrentLimit(cfg.inputCurrentDPMLimit);          // Set the input current limit
    setInputVoltageDPM(cfg.inputVoltageDPMLimit);            // Set the input voltage DPM limit
    setPrechargeCurrentLimit(cfg.prechargeCurrentLimit);     // Set the precharge current limit
    setTerminationCurrentLimit(cfg.terminationCurrentLimit); // Set the termination current limit
    configurePrechargeTermination(cfg.terminationControlEnabled, cfg.fastChargeThreshold, cfg.prechargeControlEnabled); // Configure precharge and termination control
    // Timer Control Register
    configureTopOffTimer(cfg.topOffTimer); // Configure the top-off timer
    configureWatchdogTimer(cfg.watchdogTimer); // Configure the watchdog timer
    configureChargeSafetyTimer(cfg.safetyTimerEnabled, cfg.safetyTimer, cfg.safetyTimerSpeed); // Configure the charge safety timer
    // Three-Stage_Charge_Control Register
    configureConstantVoltageTimer(cfg.constantVoltageTimer); // Configure the constant voltage timer
    // Charger Control Register
    setAutoRechargeThreshold(cfg.autoRechargeThreshold); // Set the auto-recharge threshold
    configureWatchdogTimerReset(); // Configure the watchdog timer reset
    setCEpin(cfg.CEPinEnabled); // Set the CE pin
    configureChargeBehaiorWatchdogExpires(cfg.ChargeBehaviorWatchdogExpired); // Configure the charge behavior when the watchdog expires
    setHighZmode(cfg.highZModeEnabled); // Set the high-Z mode
    enableControlledDischarge(cfg.batteryLoadEnabled); // Enable controlled discharge
    if (cfg.chargeEnabled) {enableCharge();} else {disableCharge();}; // Enable charging
    enablePins(1, 1, 1, 1); // Enable the ICHG, ILIM_HIZ, PG, and STAT pins
    configureVACLoad(true); // Configure the VAC load
    setPFMMode(false); // Set the PFM mode
    setTSPinFunction(); // Set the TS pin function
    configureADC(); // Configure the ADC

    printChargerConfig(true); // Print the charger configuration
}

// --- Helper Functions ---

uint16_t BQ25756E::getChargeVoltageLimitRegister() {
    return readRegister16(address, BQ25756E_REG_CHARGE_VOLTAGE_LIMIT) & BQ25756E_CHG_VOLTAGE_LIMIT;
}

uint16_t BQ25756E::getChargeVoltageLimit() {
    // Get the raw register value (which is masked to contain only the VFB_REG field, bits 4:0)
    uint16_t rawValue = getChargeVoltageLimitRegister();
    // Each unit in the VFB_REG field represents a 2 mV increment, with an offset of 1504 mV.
    // Thus, actual voltage in mV = (raw value * 2 mV) + 1504 mV.
    return rawValue * 2 + 1504;
}


uint16_t BQ25756E::getChargeCurrentLimitRegister() {
    return (readRegister16(address, BQ25756E_REG_CHARGE_CURRENT_LIMIT) & BQ25756E_CHG_CURRENT_LIMIT);
}

uint16_t BQ25756E::getChargeCurrentLimit() {
    // Get the raw register value (masked, but not shifted)
    uint16_t rawValue = getChargeCurrentLimitRegister();
    // The ICHG_REG field is located in bits 10:2. Shift right by 2 bits to get the raw numeric value.
    uint16_t ichg_reg = rawValue >> 2;
    // Multiply by 50 mA per step to obtain the actual current in mA.
    return ichg_reg * 50;
}

uint16_t BQ25756E::getInputCurrentDPMLimitRegister() {
    return (readRegister16(address, BQ25756E_REG_INPUT_CURRENT_DPM_LIMIT) & BQ25756E_INPUT_CURRENT_DPM_LIMIT);
}

uint16_t BQ25756E::getInputCurrentDPMLimit() {
    // Get the raw register value (masked, but not shifted)
    uint16_t rawValue = getInputCurrentDPMLimitRegister();
    // The IAC_DPM field is located in bits 10:2. Shift right by 2 bits to get the raw numeric value.
    uint16_t iac_dpm = rawValue >> 2;
    // Multiply by 50 mA per step to obtain the actual current in mA.
    return iac_dpm * 50;
}

uint16_t BQ25756E::getInputVoltageDPMLimitRegister() {
    uint8_t high_bits = readRegister(address, BQ25756E_REG_INPUT_VOLTAGE_DPM_LIMIT + 1);
    uint8_t low_bits =  readRegister(address, BQ25756E_REG_INPUT_VOLTAGE_DPM_LIMIT);
    uint16_t regValue = (high_bits << 8) | low_bits;
    return (regValue & BQ25756E_INPUT_VOLTAGE_DPM_LIMIT);
}

uint16_t BQ25756E::getInputVoltageDPMLimit() {
    // Get the raw register value (masked, but not shifted)
    uint16_t rawValue = getInputVoltageDPMLimitRegister();
    // The VAC_DPM field is located in bits 13:2. Shift right by 2 bits to get the raw numeric value.
    uint16_t vac_dpm = rawValue >> 2;
    // Multiply by 20 mV per step to obtain the actual voltage in mV.
    return vac_dpm * 20;
}

uint16_t BQ25756E::getReverseModeInputCurrentLimitRegister() {
    uint8_t high_bits = readRegister(address, BQ25756E_REG_REVERSE_MODE_INPUT_CURRENT_LIMIT + 1);
    uint8_t low_bits =  readRegister(address, BQ25756E_REG_REVERSE_MODE_INPUT_CURRENT_LIMIT);
    uint16_t regValue = (high_bits << 8) | low_bits;
    return (regValue & BQ25756E_REVERSE_MODE_INPUT_CURRENT_LIMIT);
}

uint16_t BQ25756E::getReverseModeInputCurrentLimit() {
    // Get the raw register value (masked, but not shifted)
    uint16_t rawValue = getReverseModeInputCurrentLimitRegister();
    // The IAC_REV field is located in bits 10:2. Shift right by 2 bits to get the raw numeric value.
    uint16_t iac_rev = rawValue >> 2;
    // Multiply by 50 mA per step to obtain the actual current in mA.
    return iac_rev * 50;
}

uint16_t  BQ25756E::getReverseModeInputVoltageLimitRegister() {
    uint8_t high_bits = readRegister(address, BQ25756E_REG_REVERSE_MODE_INPUT_CURRENT_LIMIT + 1);
    uint8_t low_bits =  readRegister(address, BQ25756E_REG_REVERSE_MODE_INPUT_CURRENT_LIMIT);
    uint16_t regValue = (high_bits << 8) | low_bits;
    return (regValue & BQ25756E_REVERSE_MODE_INPUT_VOLTAGE_LIMIT);
}

uint16_t BQ25756E::getReverseModeInputVoltageLimit() {
    // Get the raw register value (masked, but not shifted)
    uint16_t rawValue = getReverseModeInputVoltageLimitRegister();
    // The VAC_REV field is located in bits 13:2. Shift right by 2 bits to get the raw numeric value.
    uint16_t vac = rawValue >> 2;
    // Multiply by 20 mV per step to obtain the actual voltage in mV.
    return vac * 20;
}

uint16_t BQ25756E::getPrechargeCurrentLimitRegister() {
    uint8_t high_bits = readRegister(address, BQ25756E_REG_PRECHARGE_CURRENT_LIMIT + 1);
    uint8_t low_bits =  readRegister(address, BQ25756E_REG_PRECHARGE_CURRENT_LIMIT);
    uint16_t regValue = (high_bits << 8) | low_bits;
    return (regValue & BQ25756E_PRECHARGE_CURRENT_LIMIT);
}

uint16_t BQ25756E::getPrechargeCurrentLimit() {
    // Get the raw register value (masked, but not shifted)
    uint16_t rawValue = getPrechargeCurrentLimitRegister();
    // The IPRECHG field is located in bits 9:2. Shift right by 2 bits to get the raw numeric value.
    uint16_t ichg_pre = rawValue >> 2;
    // Multiply by 50 mA per step to obtain the actual current in mA.
    return ichg_pre * 50;
}

uint16_t BQ25756E::getTerminationCurrentLimitRegister() {
    uint8_t high_bits = readRegister(address, BQ25756E_REG_TERMINATION_CURRENT_LIMIT + 1);
    uint8_t low_bits =  readRegister(address, BQ25756E_REG_TERMINATION_CURRENT_LIMIT);
    uint16_t regValue = (high_bits << 8) | low_bits;
    return (regValue & BQ25756E_TERMINATION_CURRENT_LIMIT);
}

uint16_t BQ25756E::getTerminationCurrentLimit() {
    // Get the raw register value (masked, but not shifted)
    uint16_t rawValue = getTerminationCurrentLimitRegister();
    // The ITERM field is located in bits 9:2. Shift right by 2 bits to get the raw numeric value.
    uint16_t iter = rawValue >> 2;
    // Multiply by 50 mA per step to obtain the actual current in mA.
    return iter * 50;
}

uint8_t BQ25756E::getPrechargeTerminationControl() {
    return readRegister(address, BQ25756E_REG_PRECHARGE_TERMINATION_CONTROL) & BQ25756E_PRECHARGE_TERMINATION_CONTROL;
}

uint8_t BQ25756E::getTimerControl() {
    return readRegister(address, BQ25756E_REG_TIMER_CONTROL);
}

uint8_t BQ25756E::getThreeStageChageControl() {
    return readRegister(address, BQ25756E_REG_THREE_STAGE_CHARGE_CONTROL);
}

uint8_t BQ25756E::getChargerControl() {
    return readRegister(address, BQ25756E_REG_CHARGER_CONTROL);
}

uint8_t BQ25756E::getPinControl() {
    return readRegister(address, BQ25756E_REG_PIN_CONTROL);
}

uint8_t BQ25756E::getPowerPathReverseModeControl() {
    return readRegister(address, BQ25756E_REG_POWER_PATH_REVERSE_MODE_CONTROL) & BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL;
}

uint8_t BQ25756E::getMPPTControl() {
    return readRegister(address, BQ25756E_REG_MPPT_CONTROL) & BQ25756E_MPPT_CONTROL;
}

uint8_t BQ25756E::getTSChargingThresholdControl() {
    return readRegister(address, BQ25756E_REG_TS_CHARGING_THRESHOLD_CONTROL);
}

uint8_t BQ25756E::getTSChargingRegionBehaviorControl() {
    return readRegister(address, BQ25756E_REG_TS_CHARGING_REGION_BEHAVIOR_CONTROL) & BQ25756E_TS_CHARGING_REGION_BEHAVIOR;
}

uint8_t BQ25756E::getTSReverseModeThresholdControl() {
    return readRegister(address, BQ25756E_REG_TS_REVERSE_MODE_THRESHOLD_CONTROL) & BQ25756E_TS_REVERSE_MODE_THRESHOLD_CONTROL;
}

uint8_t BQ25756E::getReverseUndervoltageControl() {
    return readRegister(address, BQ25756E_REG_REVERSE_UNDERVOLTAGE_CONTROL) & BQ25756E_REVERSE_UNDERVOLTAGE_SYSREV_UV;
}

uint16_t BQ25756E::getVACMaxPowerPointDetectedRegister() {
    uint8_t high_bits = readRegister(address, BQ25756E_REG_VAC_MAX_POWER_POINT_DETECTED + 1);
    uint8_t low_bits =  readRegister(address, BQ25756E_REG_VAC_MAX_POWER_POINT_DETECTED);
    uint16_t regValue = (high_bits << 8) | low_bits;
    return (regValue & BQ25756E_VAC_MAX_POWER_POINT_DETECTED_MASK);
}

uint16_t BQ25756E::getVACMaxPowerPointDetected() {
    // Get the raw register value (masked, but not shifted)
    uint16_t rawValue = getVACMaxPowerPointDetectedRegister();
    // The VAC_MPP field is located in bits 13:2. Shift right by 2 bits to get the raw numeric value.
    uint16_t vac_max = rawValue >> 2;
    // Multiply by 20 mV per step to obtain the actual voltage in mV.
    return vac_max * 20;
}

uint8_t BQ25756E::getChargerStatus1() {
    return readRegister(address, BQ25756E_REG_CHARGER_STATUS_1);
}

uint8_t BQ25756E::getChargeCycleStatus() {
    uint8_t regVal = getChargerStatus1();
    // According to the table, bits [2:0] of this register indicate charge cycle status
    return regVal & BQ25756E_CHG_STATUS_1_CHARGE_STAT_2; // Mask the lower 3 bits
}

uint8_t BQ25756E::getChargerStatus2() {
    return readRegister(address, BQ25756E_REG_CHARGER_STATUS_2);
}

uint8_t BQ25756E::getChargerStatus3() {
    return readRegister(address, BQ25756E_REG_CHARGER_STATUS_3);
}

uint8_t BQ25756E::getFaultStatus() {
    return readRegister(address, BQ25756E_REG_FAULT_STATUS);
}

uint8_t BQ25756E::getChargerFlag1() {
    return readRegister(address, BQ25756E_REG_CHARGER_FLAG_1);
}

uint8_t BQ25756E::getChargerFlag2() {
    return readRegister(address, BQ25756E_REG_CHARGER_FLAG_2);
}

uint8_t BQ25756E::getFaultFlag() {
    return readRegister(address, BQ25756E_REG_FAULT_FLAG);
}

uint8_t BQ25756E::getChargerMask1() {
    return readRegister(address, BQ25756E_REG_CHARGER_MASK_1);
}

uint8_t BQ25756E::getChargerMask2() {
    return readRegister(address, BQ25756E_REG_CHARGER_MASK_2);
}

uint8_t BQ25756E::getFaultMask() {
    return readRegister(address, BQ25756E_REG_FAULT_MASK);
}

uint8_t BQ25756E::getADCControl() {
    return readRegister(address, BQ25756E_REG_ADC_CONTROL);
}

uint8_t BQ25756E::getADCChannelControl() {
    return readRegister(address, BQ25756E_REG_ADC_CHANNEL_CONTROL);
}

uint16_t BQ25756E::getIACADCRegister() {
    return readRegister16(address, BQ25756E_REG_IAC_ADC);
}

uint16_t BQ25756E::getIACADC() {
    return getIACADCRegister() * 0.8;
}

uint16_t BQ25756E::getIBATADCRegister() {
    return readRegister16(address, BQ25756E_REG_IBAT_ADC);
}

uint16_t BQ25756E::getIBATADC() {
    return getIBATADCRegister() * 2;
}

uint16_t BQ25756E::getVACADCRegister() {
    return readRegister16(address, BQ25756E_REG_VAC_ADC);
}

uint16_t BQ25756E::getVACADC() {
    return getVACADCRegister() * 2;
}

uint16_t BQ25756E::getVBATADCRegister() {
    return readRegister16(address, BQ25756E_REG_VBAT_ADC);
}

uint16_t BQ25756E::getVBATADC() {
    return getVBATADCRegister() * 2;
}

uint16_t BQ25756E::getTSADCRegister() {
    return readRegister16(address, BQ25756E_REG_TS_ADC);
}

double BQ25756E::getTSADC() {
    return getTSADCRegister() * 0.09765625;
}

uint16_t BQ25756E::getVFBADCRegister() {
    return readRegister16(address, BQ25756E_REG_VFB_ADC);
}

uint16_t BQ25756E::getVFBADC() {
    return getVFBADCRegister();
}

uint8_t BQ25756E::getGateDriverStrengthControl() {
    return readRegister(address, BQ25756E_REG_GATE_DRIVER_STRENGTH_CONTROL);
}

uint8_t BQ25756E::getGateDriverDeadTimeControl() {
    return readRegister(address, BQ25756E_REG_GATE_DRIVER_DEAD_TIME_CONTROL) & BQ25756E_BOOST_DEAD_TIME_CONTROL;
}

uint8_t BQ25756E::getPartInformation() {
    return readRegister(address, BQ25756E_REG_PART_INFORMATION) & BQ25756E_PART_INFO;
}

uint8_t BQ25756E::getReverseModeBatteryDischargeCurrent() {
    return readRegister(address, BQ25756E_REG_REVERSE_MODE_BATTERY_DISCHARGE_CURRENT) & BQ25756E_IBAT_REV_CONTROL;
}

// --- Control Functions ---

void BQ25756E::setChargeVoltageLimit(uint16_t voltage_mV) {
    const uint16_t minVoltage = 1504;
    const uint16_t maxVoltage = 1566; // Only a software limit, as no hardware voltage limit attribute exists
    if (voltage_mV < minVoltage || voltage_mV > maxVoltage) {
        Serial.print("Error: Voltage out of range (");
        Serial.print(minVoltage);
        Serial.print("mV - ");
        Serial.print(maxVoltage);
        Serial.println("mV)");
        // Assign a default value
        voltage_mV = 1536;
    }
    // Compute the 5-bit register value (step = 2mV, offset = 1504mV)
    uint8_t regValue = (voltage_mV - minVoltage) / 2;
    // Use the defined bit mask for voltage limit (lower 5 bits)
    uint8_t registerValue = regValue & BQ25756E_CHG_VOLTAGE_LIMIT;
    // Write the 16-bit value to the Charge Voltage Limit register
    writeRegister(address, BQ25756E_REG_CHARGE_VOLTAGE_LIMIT, registerValue);
}

void BQ25756E::setChargeCurrentLimit(uint16_t current_mA) {
    const uint16_t minCurrent = 400;
    // Use the lower between software limit (20000 mA) and the hardware limit (max_charge_current)
    uint16_t effectiveMaxCurrent = (max_charge_current < 20000 ? max_charge_current : 20000);
    if (current_mA < minCurrent || current_mA > effectiveMaxCurrent) {
        _debugPort->print("Error: Requested charge current limit out of range (");
        _debugPort->print(minCurrent);
        _debugPort->print("mA - ");
        _debugPort->print(effectiveMaxCurrent);
        _debugPort->println("mA)");
        // Assign a safe default value
        current_mA = minCurrent;
    }
    config.chargeCurrentLimit = current_mA;
    // Compute the 9-bit register value (step = 50mA)
    uint16_t regValue = (current_mA / 50) << 2;  // Align with bits 10:2
    // Read the current register value
    uint16_t currentRegisterValue = readRegister16(address, BQ25756E_REG_CHARGE_CURRENT_LIMIT);
    // Preserve reserved bits (mask out only bits 10:2)
    currentRegisterValue &= ~BQ25756E_CHG_CURRENT_LIMIT;  // Clear bits 10:2
    currentRegisterValue |= regValue; // Set new charge current limit
    // Write the modified value back to the register
    writeRegister16(address, BQ25756E_REG_CHARGE_CURRENT_LIMIT, currentRegisterValue);
}

void BQ25756E::setInputCurrentLimit(uint16_t current_mA) {
    const uint16_t minCurrent = 400;
    // Use the lower between software limit (20000 mA) and the hardware limit (max_input_current)
    uint16_t effectiveMaxCurrent = (max_input_current < 20000 ? max_input_current : 20000);
    if (current_mA < minCurrent || current_mA > effectiveMaxCurrent) {
        Serial.print("Error: Requested input current limit out of range (");
        Serial.print(minCurrent);
        Serial.print("mA - ");
        Serial.print(effectiveMaxCurrent);
        Serial.println("mA)");
        return;
    }
    // Compute the 9-bit register value (step = 50mA)
    uint16_t regValue = (current_mA / 50) << 2;  // Align with bits 10:2

    // Read the current register value
    uint16_t currentRegisterValue = readRegister16(address, BQ25756E_REG_INPUT_CURRENT_DPM_LIMIT);
    // Preserve reserved bits (mask out only bits 10:2)
    currentRegisterValue &= ~BQ25756E_INPUT_CURRENT_DPM_LIMIT;  // Clear bits 10:2
    currentRegisterValue |= regValue; // Set new input current limit
    // Write the modified value back to the register
    writeRegister16(address, BQ25756E_REG_INPUT_CURRENT_DPM_LIMIT, currentRegisterValue);
}

void BQ25756E::setInputVoltageDPM(uint16_t voltage_mV) {
    // Define the minimum voltage required to allow charging
    // Use the higher between software limit (up to 36000 mV) and the hardware limit (min_voltage)
    if (voltage_mV < min_voltage || voltage_mV > max_voltage) {
        Serial.print("Error: Requested Input Voltage DPM out of range (");
        Serial.print(min_voltage);
        Serial.print("mV - ");
        Serial.print(max_voltage);
        Serial.println("mV). Defaulting to minimum DPM limit.");
        voltage_mV = min_voltage;
    }
    // Compute the 12-bit register value (step = 20mV)
    uint16_t regValue = (voltage_mV / 20) << 2; // The result will occupy bits [13:2], so we shift left by 2

    // Read the current register value
    uint16_t registerValue = readRegister16(address, BQ25756E_REG_INPUT_VOLTAGE_DPM_LIMIT);
    // Preserve reserved bits (mask out only bits 13:2)
    registerValue &= ~BQ25756E_INPUT_VOLTAGE_DPM_LIMIT;  // Clear bits 13:2
    registerValue |= regValue; // Set new input voltage DPM limit
    // Write the modified value back to the register
    writeRegister16(address, BQ25756E_REG_INPUT_VOLTAGE_DPM_LIMIT, registerValue);
}

void BQ25756E::setPrechargeCurrentLimit(uint16_t current_mA) {
    if (current_mA < 250 || current_mA > 10000) {
        Serial.print("Error: Requested precharge current limit out of range (250mA - 10000A)");
        current_mA = 250; // Assign a safe default value
    }
    config.prechargeCurrentLimit = current_mA;
    // Compute the 9-bit register value (step = 50mA)
    uint16_t regValue = (current_mA / 50) << 2; // Align with bits 9:2

    // Read the current register value
    uint16_t currentRegisterValue = readRegister16(address, BQ25756E_REG_PRECHARGE_CURRENT_LIMIT);
    // Preserve reserved bits (mask out only bits 9:2)
    currentRegisterValue &= ~BQ25756E_PRECHARGE_CURRENT_LIMIT;  // Clear bits 9:2
    currentRegisterValue |= regValue; // Set new precharge current limit
    // Write the modified value back to the register
    writeRegister16(address, BQ25756E_REG_PRECHARGE_CURRENT_LIMIT, currentRegisterValue);
}

void BQ25756E::setTerminationCurrentLimit(uint16_t current_mA) {
    if (current_mA < 250 || current_mA > 10000) {
        Serial.print("Error: Requested termination current limit out of range (250mA - 10000mA)");
        current_mA = 250; // Assign a safe default value
    }
    config.terminationCurrentLimit = current_mA;
    // Compute the 9-bit register value (step = 50mA)
    uint16_t regValue = (current_mA / 50) << 2; // Align with bits 9:2

    // Read the current register value
    uint16_t currentRegisterValue = readRegister16(address, BQ25756E_REG_TERMINATION_CURRENT_LIMIT);
    // Preserve reserved bits (mask out only bits 9:2)
    currentRegisterValue &= ~BQ25756E_TERMINATION_CURRENT_LIMIT;  // Clear bits 9:2
    currentRegisterValue |= regValue; // Set new termination current limit
    // Write the modified value back to the register
    writeRegister16(address, BQ25756E_REG_TERMINATION_CURRENT_LIMIT, currentRegisterValue);
}

void BQ25756E::configurePrechargeTermination(bool terminationControlEnabled, uint8_t terminationThreshold, bool prechargeControlEnabled) {
    modifyRegister(address, BQ25756E_REG_PRECHARGE_TERMINATION_CONTROL, BQ25756E_EN_TERM, terminationControlEnabled);
    modifyRegisterBits(address, BQ25756E_REG_PRECHARGE_TERMINATION_CONTROL, BQ25756E_VBAT_LOWV, terminationThreshold << 1);
    modifyRegister(address, BQ25756E_REG_PRECHARGE_TERMINATION_CONTROL, BQ25756E_EN_TERM, prechargeControlEnabled);
}

void BQ25756E::configureTopOffTimer(uint8_t top_off_timer) {
    modifyRegisterBits(address, BQ25756E_REG_TIMER_CONTROL, BQ25756E_TIMER_CONTROL_TOPOFF_TMR, top_off_timer << 6);
}

void BQ25756E::configureWatchdogTimer(uint8_t watchdog_timer) {
    modifyRegisterBits(address, BQ25756E_REG_TIMER_CONTROL, BQ25756E_TIMER_CONTROL_WATCHDOG, watchdog_timer << 4);
}

void BQ25756E::configureChargeSafetyTimer(bool enabble_safe_timer, uint8_t safety_timer, bool speed_during_DPM) {
    modifyRegister(address, BQ25756E_REG_TIMER_CONTROL, BQ25756E_TIMER_CONTROL_EN_CHG_TMR, enabble_safe_timer);
    modifyRegisterBits(address, BQ25756E_REG_TIMER_CONTROL, BQ25756E_TIMER_CONTROL_CHG_TMR, safety_timer << 1);
    modifyRegister(address, BQ25756E_REG_TIMER_CONTROL, BQ25756E_TIMER_CONTROL_EN_TMR2X, speed_during_DPM);
}

void BQ25756E::configureConstantVoltageTimer(uint8_t CV_timer) {
    modifyRegisterBits(address, BQ25756E_REG_THREE_STAGE_CHARGE_CONTROL, BQ25756E_THREE_STAGE_CHARGE_CONTROL_CV_TMR, CV_timer);
}

void BQ25756E::setAutoRechargeThreshold(uint8_t auto_recharge_threshold) {
    // Set the auto-recharge threshold
    modifyRegister(address, BQ25756E_REG_CHARGER_CONTROL, BQ25756E_CHARGER_CONTROL_VRECHG, auto_recharge_threshold);
}

void BQ25756E::configureWatchdogTimerReset(bool enable_watchdog_reset) {
    // Enable the watchdog timer reset
    modifyRegister(address, BQ25756E_REG_CHARGER_CONTROL, BQ25756E_CHARGER_CONTROL_WD_RST, enable_watchdog_reset);
}
void BQ25756E::setCEpin(bool enable_CE_pin) {
    // Set the CE pin to enable the charger
    modifyRegister(address, BQ25756E_REG_CHARGER_CONTROL, BQ25756E_CHARGER_CONTROL_DIS_CE_PIN, !enable_CE_pin);
}

void BQ25756E::configureChargeBehaiorWatchdogExpires(bool disable_charge) {
    // Set the EN_CHG bit to reset to 1 when the watchdog expires
    modifyRegister(address, BQ25756E_REG_CHARGER_CONTROL, BQ25756E_CHARGER_CONTROL_EN_CHG_BIT_RST, !disable_charge);
}

void BQ25756E::setHighZmode(bool enable_highZ) {
    // Enable the high-Z mode
    modifyRegister(address, BQ25756E_REG_CHARGER_CONTROL, BQ25756E_CHARGER_CONTROL_EN_HIZ, enable_highZ);
}

void BQ25756E::enableControlledDischarge(bool enable_discharge) {
    // Enable controlled discharge
    modifyRegister(address, BQ25756E_REG_CHARGER_CONTROL, BQ25756E_CHARGER_CONTROL_EN_IBAT_LOAD, enable_discharge);
}

void BQ25756E::enableCharge() {
    // Enable the charger
    config.chargeEnabled = true;
    modifyRegister(address, BQ25756E_REG_CHARGER_CONTROL, BQ25756E_CHARGER_CONTROL_EN_CHG, true);
}

void BQ25756E::disableCharge() {
    // Disable the charger
    config.chargeEnabled = false;
    modifyRegister(address, BQ25756E_REG_CHARGER_CONTROL, BQ25756E_CHARGER_CONTROL_EN_CHG, false);
}

void BQ25756E::enablePins(bool enable_ICHG, bool enableILIM_HIZ, bool enable_PG, bool enable_STAT) {
    // Enable the ICHG, ILIM_HIZ, PG, and STAT pins
    uint8_t pinControl = enable_ICHG << 7 | enableILIM_HIZ << 6 | !enable_PG << 5 | !enable_STAT << 4;
    writeRegister(address, BQ25756E_REG_PIN_CONTROL, pinControl);
}

void BQ25756E::resetRegisters() {
    // Reset all registers to default values
    modifyRegister(address, BQ25756E_REG_POWER_PATH_REVERSE_MODE_CONTROL, BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL_REG_RST, true);
}

void BQ25756E::configureVACLoad(bool enable_load) {
    // Enable the VAC load
    modifyRegister(address, BQ25756E_REG_POWER_PATH_REVERSE_MODE_CONTROL, BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL_EN_IAC_LOAD, enable_load);
}

void BQ25756E::setPFMMode(bool enable_PFM) {
    // Enable PFM mode
    modifyRegister(address, BQ25756E_REG_POWER_PATH_REVERSE_MODE_CONTROL, BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL_EN_PFM, enable_PFM);
}

void BQ25756E::setReverseMode(bool enable_reverse_mode) {
    // Enable reverse mode
    modifyRegister(address, BQ25756E_REG_POWER_PATH_REVERSE_MODE_CONTROL, BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL_EN_REV, enable_reverse_mode);
}

void BQ25756E::setTSPinFunction(bool enable_TS) {
    // Set the TS pin function
    modifyRegister(address, BQ25756E_REG_TS_CHARGING_REGION_BEHAVIOR_CONTROL, BQ25756E_TS_CHARGING_REGION_BEHAVIOR_EN_TS, enable_TS);
}

void BQ25756E::configureADC(bool enable_ADC, bool one_shot, uint8_t sample_speed, bool running_avg, bool init_avg) {

    uint8_t adcControl = enable_ADC << 7 | one_shot << 6 | sample_speed << 4 | running_avg << 3 | init_avg << 2;
    writeRegister(address, BQ25756E_REG_ADC_CONTROL, adcControl);
}

void BQ25756E::chargPrint(const char* msg) {
    #ifdef DRV8214_PLATFORM_ARDUINO
        if (_debugPort) {
            _debugPort->print(msg);
        }
    #elif defined(DRV8214_PLATFORM_STM32)
        // Option 1: Using HAL_UART_Transmit directly
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
        // Option 2: If you have retargeted printf to UART, you could simply use:
        // printf("%s", msg);
    #endif
}

void BQ25756E::printChargerConfig(bool initial_config) 
{
    chargPrint("----- CHARGER CONFIGURATION REGISTERS -----\n");
    chargPrint("Charge Voltage Limit: ");
    print2BytesAsBinary(getChargeVoltageLimitRegister());
    chargPrint("(");
    _debugPort->print(getChargeVoltageLimit());
    chargPrint(" mV) ");
    chargPrint("Charge Current Limit: ");
    print2BytesAsBinary(getChargeCurrentLimitRegister());
    chargPrint("(");
    _debugPort->print(getChargeCurrentLimit());
    chargPrint(" mA) ");
    chargPrint("Input Current DPM Limit: ");
    print2BytesAsBinary(getInputCurrentDPMLimitRegister());
    chargPrint("(");
    _debugPort->print(getInputCurrentDPMLimit());
    chargPrint(" mA)\n");
    chargPrint("Input Voltage DPM Limit: ");
    print2BytesAsBinary(getInputVoltageDPMLimitRegister());
    chargPrint("(");
    _debugPort->print(getInputVoltageDPMLimit());
    chargPrint(" mV) ");
    chargPrint("Precharge Current Limit: ");
    print2BytesAsBinary(getPrechargeCurrentLimitRegister());
    chargPrint("(");
    _debugPort->print(getPrechargeCurrentLimit());
    chargPrint(" mA) ");
    chargPrint("Termination Current Limit: ");
    print2BytesAsBinary(getTerminationCurrentLimitRegister());
    chargPrint("(");
    _debugPort->print(getTerminationCurrentLimit());
    chargPrint(" mA)\n");
    chargPrint("Precharge Termination Control: ");
    printByteAsBinary(getPrechargeTerminationControl());
    chargPrint("- Timer Control: ");
    printByteAsBinary(getTimerControl());
    chargPrint("- Three-Stage Charge Control: ");
    printByteAsBinary(getThreeStageChageControl());
    chargPrint("- Charger Control: ");
    printByteAsBinary(getChargerControl());
    chargPrint("- Pin Control: ");
    printByteAsBinary(getPinControl());
    chargPrint("\n");
    chargPrint("Power Path Reverse Mode Control: ");
    printByteAsBinary(getPowerPathReverseModeControl());;
    chargPrint("MPPT Control: ");
    printByteAsBinary(getMPPTControl());
    chargPrint(" - TS Charging Threshold Control: ");
    printByteAsBinary(getTSChargingThresholdControl());
    chargPrint(" - TS Charging Region Behavior Control: ");
    printByteAsBinary(getTSChargingRegionBehaviorControl());
    chargPrint("\n");
    chargPrint("TS Reverse Mode Threshold Control: ");
    printByteAsBinary(getTSReverseModeThresholdControl());
    chargPrint(" - Reverse Undervoltage Control: ");
    printByteAsBinary(getReverseUndervoltageControl());
    chargPrint("\n");
    chargPrint("VAC Max Power Point Detected: ");
    print2BytesAsBinary(getVACMaxPowerPointDetectedRegister());
    chargPrint("(");
    _debugPort->print(getVACMaxPowerPointDetected());
    chargPrint(" mV) ");
    chargPrint("Charger Status 1: ");
    printByteAsBinary(getChargerStatus1());
    chargPrint("Charger Status 2: ");
    printByteAsBinary(getChargerStatus2());
    chargPrint("Charger Status 3: ");
    printByteAsBinary(getChargerStatus3());
    chargPrint("Fault Status: ");
    printByteAsBinary(getFaultStatus());
    chargPrint("\n");
    chargPrint("Charger Flag 1: ");
    printByteAsBinary(getChargerFlag1());
    chargPrint("Charger Flag 2: ");
    printByteAsBinary(getChargerFlag2());
    chargPrint("Fault Flag: ");
    printByteAsBinary(getFaultFlag());
    chargPrint("Charger Mask 1: ");
    printByteAsBinary(getChargerMask1());
    chargPrint("Charger Mask 2: ");
    printByteAsBinary(getChargerMask2());
    chargPrint("Fault Mask: ");
    printByteAsBinary(getFaultMask());
    chargPrint("\n");
    chargPrint("ADC Control: ");
    printByteAsBinary(getADCControl());
    chargPrint("ADC Channel Control: ");
    printByteAsBinary(getADCChannelControl());
    chargPrint("IAC ADC: ");
    print2BytesAsBinary(getIACADCRegister());
    chargPrint("(");
    _debugPort->print(getIACADC());
    chargPrint(" mA) ");
    chargPrint("IBAT ADC: ");
    print2BytesAsBinary(getIBATADCRegister());
    chargPrint("(");
    _debugPort->print(getIBATADC());
    chargPrint(" mA) ");
    chargPrint("VAC ADC: ");
    print2BytesAsBinary(getVACADCRegister());
    chargPrint("(");
    _debugPort->print(getVACADC());
    chargPrint(" mV) ");
    chargPrint("\n");
    chargPrint("VBAT ADC: ");
    print2BytesAsBinary(getVBATADCRegister());
    chargPrint("(");
    _debugPort->print(getVBATADC());
    chargPrint(" mV) ");
    chargPrint("TS ADC: ");
    print2BytesAsBinary(getTSADCRegister());
    chargPrint("(");
    _debugPort->print(getTSADC());
    chargPrint(" C) ");
    chargPrint("VFB ADC: ");
    print2BytesAsBinary(getVFBADCRegister());
    chargPrint("(");
    _debugPort->print(getVFBADC());
    chargPrint(" mV)\n");
    chargPrint("Gate Driver Strength Control: ");
    printByteAsBinary(getGateDriverStrengthControl());
    chargPrint("Gate Driver Dead Time Control: ");
    printByteAsBinary(getGateDriverDeadTimeControl());
    chargPrint("Part Information: ");
    printByteAsBinary(getPartInformation());
    chargPrint("Reverse Mode Battery Discharge Current: ");
    printByteAsBinary(getReverseModeBatteryDischargeCurrent());
    chargPrint("\n\n");
}


void BQ25756E::setDebugStream(Stream* debugPort) {
    _debugPort = debugPort;
}

// Function to print a byte as binary (8 bits)
void BQ25756E::printByteAsBinary(uint8_t value) {
    char bit[2] = {'0', '\0'};
    for (int i = 7; i >= 0; i--) {
        bit[0] = ((value >> i) & 1) ? '1' : '0';
        chargPrint(bit);
    }
    chargPrint("  ");
}

// Function to print two bytes as binary (16 bits) 
void BQ25756E::print2BytesAsBinary(uint16_t value) {
    char bit[2] = {'0', '\0'};
    for (int i = 15; i >= 0; i--) {
        bit[0] = ((value >> i) & 1) ? '1' : '0';
        chargPrint(bit);
    }
    chargPrint("  ");
}
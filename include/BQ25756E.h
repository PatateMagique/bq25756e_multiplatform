/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the bq25756e_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#ifndef BQ25756E_H
#define BQ25756E_H

#include "drv8214_platform_config.h" // For platform detection
#include "drv8214_platform_i2c.h"    // For abstracted I2C functions

// --- REGISTER DEFINITIONS ---
// (Refer to the datasheet for detailed register information)
#define BQ25756E_REG_CHARGE_VOLTAGE_LIMIT                   0x00  // Charge Voltage Limit Register
#define BQ25756E_REG_CHARGE_CURRENT_LIMIT                   0x02  // Charge Current Limit Register
#define BQ25756E_REG_INPUT_CURRENT_DPM_LIMIT                0x06  // Input Current DPM Limit Register
#define BQ25756E_REG_INPUT_VOLTAGE_DPM_LIMIT                0x08  // Input Voltage DPM Limit Register
#define BQ25756E_REG_REVERSE_MODE_INPUT_CURRENT_LIMIT       0x0A  // Reverse Mode Input Current Limit
#define BQ25756E_REG_REVERSE_MODE_INPUT_VOLTAGE_LIMIT       0x0C  // Reverse Mode Input Voltage Limit
#define BQ25756E_REG_PRECHARGE_CURRENT_LIMIT                0x10  // Precharge Current Limit Register
#define BQ25756E_REG_TERMINATION_CURRENT_LIMIT              0x12  // Termination Current Limit Register
#define BQ25756E_REG_PRECHARGE_TERMINATION_CONTROL          0x14  // Precharge & Termination Control
#define BQ25756E_REG_TIMER_CONTROL                          0x15  // Timer Control Register
#define BQ25756E_REG_THREE_STAGE_CHARGE_CONTROL             0x16  // Three-Stage Charge Control
#define BQ25756E_REG_CHARGER_CONTROL                        0x17  // Charger Control Register
#define BQ25756E_REG_PIN_CONTROL                            0x18  // Pin Control Register
#define BQ25756E_REG_POWER_PATH_REVERSE_MODE_CONTROL        0x19  // Power Path & Reverse Mode Control
#define BQ25756E_REG_MPPT_CONTROL                           0x1A  // MPPT Control Register
#define BQ25756E_REG_TS_CHARGING_THRESHOLD_CONTROL          0x1B  // TS Charging Threshold Control
#define BQ25756E_REG_TS_CHARGING_REGION_BEHAVIOR_CONTROL    0x1C  // TS Charging Region Behavior Control
#define BQ25756E_REG_TS_REVERSE_MODE_THRESHOLD_CONTROL      0x1D  // TS Reverse Mode Threshold Control
#define BQ25756E_REG_REVERSE_UNDERVOLTAGE_CONTROL           0x1E  // Reverse Undervoltage Control
#define BQ25756E_REG_VAC_MAX_POWER_POINT_DETECTED           0x1F  // VAC Max Power Point Detected
#define BQ25756E_REG_CHARGER_STATUS_1                       0x21  // Charger Status 1
#define BQ25756E_REG_CHARGER_STATUS_2                       0x22  // Charger Status 2
#define BQ25756E_REG_CHARGER_STATUS_3                       0x23  // Charger Status 3
#define BQ25756E_REG_FAULT_STATUS                           0x24  // Fault Status Register
#define BQ25756E_REG_CHARGER_FLAG_1                         0x25  // Charger Flag 1
#define BQ25756E_REG_CHARGER_FLAG_2                         0x26  // Charger Flag 2
#define BQ25756E_REG_FAULT_FLAG                             0x27  // Fault Flag Register
#define BQ25756E_REG_CHARGER_MASK_1                         0x28  // Charger Mask 1
#define BQ25756E_REG_CHARGER_MASK_2                         0x29  // Charger Mask 2
#define BQ25756E_REG_FAULT_MASK                             0x2A  // Fault Mask Register
#define BQ25756E_REG_ADC_CONTROL                            0x2B  // ADC Control Register
#define BQ25756E_REG_ADC_CHANNEL_CONTROL                    0x2C  // ADC Channel Control Register
#define BQ25756E_REG_IAC_ADC                                0x2D  // IAC ADC Register
#define BQ25756E_REG_IBAT_ADC                               0x2F  // IBAT ADC Register
#define BQ25756E_REG_VAC_ADC                                0x31  // VAC ADC Register
#define BQ25756E_REG_VBAT_ADC                               0x33  // VBAT ADC Register
#define BQ25756E_REG_TS_ADC                                 0x37  // TS ADC Register
#define BQ25756E_REG_VFB_ADC                                0x39  // VFB ADC Register
#define BQ25756E_REG_GATE_DRIVER_STRENGTH_CONTROL           0x3B  // Gate Driver Strength Control
#define BQ25756E_REG_GATE_DRIVER_DEAD_TIME_CONTROL          0x3C  // Gate Driver Dead Time Control
#define BQ25756E_REG_PART_INFORMATION                       0x3D  // Part Information Register
#define BQ25756E_REG_REVERSE_MODE_BATTERY_DISCHARGE_CURRENT 0x62  // Reverse Mode Battery Discharge Current

// --- BIT MASKS FOR CONTROL REGISTERS ---

// -----------------------------------------------------------------------------
// REG_CHARGE_VOLTAGE_LIMIT (0x00)
// -----------------------------------------------------------------------------
// Bit 15-5 are reserved
#define BQ25756E_CHG_VOLTAGE_LIMIT           0x1F   // Bit 4-0: Charge voltage limit (mV)
                                                    // FB Voltage Regulation Limit:
                                                    // POR: 1536mV (10h)
                                                    // Range: 1504mV-1566mV (0h-1Fh)
                                                    // Bit Step: 2mV - Offset: 1504mV

// -----------------------------------------------------------------------------
// REG_CHARGE_CURRENT_LIMIT (0x02)
// -----------------------------------------------------------------------------
// Bit 15-11 are reserved
#define BQ25756E_CHG_CURRENT_LIMIT           0x07FC // Bit 10-2: Fast Charge Current Regulation Limit with 5mΩ
                                                    // RBAT_SNS:
                                                    // Actual charge current is the lower of ICHG_REG and
                                                    // ICHG pin
                                                    // POR: 20000mA (190h)
                                                    // Range: 400mA-20000mA (8h-190h)
                                                    // Clamped Low
                                                    // Clamped High
                                                    // Bit Step: 50mA
// Bit 1-0 are reserved

// -----------------------------------------------------------------------------
// REG_INPUT_CURRENT_DPM_LIMIT (0x06)
// -----------------------------------------------------------------------------
// Bit 15-11 are reserved
#define BQ25756E_INPUT_CURRENT_DPM_LIMIT     0x07FC // Bit 10-2: Input Current DPM Regulation Limit with 5mΩ
                                                    // RAC_SNS:
                                                    // Actual input current limit is the lower of IAC_DPM and
                                                    // ILIM_HIZ pin
                                                    // POR: 20000mA (190h)
                                                    // Range: 400mA-20000mA (8h-190h)
                                                    // Clamped Low
                                                    // Clamped High
                                                    // Bit Step: 50mA
// Bit 1-0 are reserved

// -----------------------------------------------------------------------------
// REG_INPUT_VOLTAGE_DPM_LIMIT (0x08)
// -----------------------------------------------------------------------------
// Bit 15-14 are reserved
#define BQ25756E_INPUT_VOLTAGE_DPM_LIMIT     0x3FFC // Bit 13-2: Input Voltage Regulation Limit:
                                                    // Note if EN_MPPT = 1, the Full Sweep method will use
                                                    // this limit as the lower search window for Full Panel
                                                    // Sweep
                                                    // POR: 4200mV (D2h)
                                                    // Range: 4200mV-36000mV (D2h-708h)
                                                    // Clamped Low
                                                    // Clamped High
                                                    // Bit Step: 20mV
// Bit 1-0 are reserved

// -----------------------------------------------------------------------------
// REG_REVERSE_MODE_INPUT_CURRENT_LIMIT (0x0A)
// -----------------------------------------------------------------------------
// Bit 15-11 are reserved
#define BQ25756E_REVERSE_MODE_INPUT_CURRENT_LIMIT 0x07FC    // Bit 10-2: Input Current Regulation in Reverse Mode with 5mΩ
                                                            // RAC_SNS:
                                                            // POR: 20000mA (190h)
                                                            // Range: 400mA-20000mA (8h-190h)
                                                            // Clamped Low
                                                            // Clamped High
                                                            // Bit Step: 50mA
// Bit 1-0 are reserved

// -----------------------------------------------------------------------------
// REG_REVERSE_MODE_INPUT_VOLTAGE_LIMIT (0x0C)
// -----------------------------------------------------------------------------
// Bit 15-14 are reserved
#define BQ25756E_REVERSE_MODE_INPUT_VOLTAGE_LIMIT 0x3FFC    // Bit 13-2: VAC Voltage Regulation in Reverse Mode:
                                                            // POR: 5000mV (FAh)
                                                            // Range: 3300mV-36000mV (A5h-708h)
                                                            // Clamped Low
                                                            // Clamped High
                                                            // Bit Step: 20mV
// Bit 1-0 are reserved

// -----------------------------------------------------------------------------
// REG_PRECHARGE_CURRENT_LIMIT (0x10)
// -----------------------------------------------------------------------------
// Bit 15-10 are reserved
#define BQ25756E_PRECHARGE_CURRENT_LIMIT     0x03FC // Bit 9-2: Pre-charge current regulation limit with 5mΩ
                                                    // RBAT_SNS:
                                                    // POR: 4000mA (50h)
                                                    // Range: 250mA-10000mA (5h-C8h)
                                                    // Clamped Low
                                                    // Clamped High
                                                    // Bit Step: 50mA
// Bit 1-0 are reserved

// -----------------------------------------------------------------------------
// REG_TERMINATION_CURRENT_LIMIT (0x12)
// -----------------------------------------------------------------------------
// Bit 15-10 are reserved
#define BQ25756E_TERMINATION_CURRENT_LIMIT   0x03FC // Bit 9-2: // Termination Current Threshold with 5mΩ RBAT_SNS:
                                                    // POR: 2000mA (28h)
                                                    // Range: 250mA-10000mA (5h-C8h)
                                                    // Clamped Low
                                                    // Clamped High
                                                    // Bit Step: 50mA
// Bit 1-0 are reserved

// -----------------------------------------------------------------------------
// REG_PRECHARGE_TERMINATION_CONTROL (0x14)
// -----------------------------------------------------------------------------
#define BQ25756E_PRECHARGE_TERMINATION_CONTROL 0x0F
// Bit 7-4 are reserved
#define BQ25756E_EN_TERM                     0x08   // Bit 3: Enable termination control 0b = Disable - 1b = Enable
#define BQ25756E_VBAT_LOWV                   0x06   // Bit 2-1: Battery threshold for PRECHG to FASTCHG transition, as percentage of VFB_REG:
                                                    // 00b = 30% x VFB_REG
                                                    // 01b = 55% x VFB_REG
                                                    // 10b = 66.7% x VFB_REG
                                                    // 11b = 71.4% x VFB_REG
#define BQ25756E_EN_PRECHG                   0x01   // Bit 0: Enable pre-charge and trickle charge functions: 0b = Disable - 1b = Enable

// -----------------------------------------------------------------------------
// REG_TIMER_CONTROL (0x15) - Timer Control Register
// -----------------------------------------------------------------------------
#define BQ25756E_TIMER_CONTROL               0xFF  // Bit 7-0: Timer control register
#define BQ25756E_TIMER_CONTROL_TOPOFF_TMR    0xC0  // Bits 7:6: Top-off timer control: 00b = Disable, 01b = 15 mins, 10b = 30 mins, 11b = 45 mins
#define BQ25756E_TIMER_CONTROL_WATCHDOG      0x30  // Bits 5:4: Watchdog timer control: 00b = Disable, 01b = 40s, 10b = 80s, 11b = 160s
#define BQ25756E_TIMER_CONTROL_EN_CHG_TMR    0x08  // Bit 3: Enable charge safety timer (0 = Disable, 1 = Enable)
#define BQ25756E_TIMER_CONTROL_CHG_TMR       0x06  // Bits 2:1: Charge safety timer setting: 00b = 5hr, 01b = 8hr, 10b = 12hr, 11b = 24hr
#define BQ25756E_TIMER_CONTROL_EN_TMR2X      0x01  // Bit 0: Timer speed control in DPM: 0b = Normal count, 1 = Slowed by 2×

// -----------------------------------------------------------------------------
// REG_THREE_STAGE_CHARGE_CONTROL (0x16) - Three-Stage Charge Control
// -----------------------------------------------------------------------------
// Bit 7-4 are reserved
#define BQ25756E_THREE_STAGE_CHARGE_CONTROL_CV_TMR  0x0F   // Bits 3:0: CV timer setting: 0000b = Disable, 0001b = 1hr, …, 1111b = 15hr

// -----------------------------------------------------------------------------
// REG_CHARGER_CONTROL (0x17) - Charger Control Register
// -----------------------------------------------------------------------------
#define BQ25756E_CHARGER_CONTROL_VRECHG          0xC0  // Bits 7:6: Battery auto-recharge threshold:
                                                       // 00b = 93.0% x VFB_REG, 01b = 94.3% x VFB_REG, 10b = 95.2% x VFB_REG, 11b = 97.6% x VFB_REG
#define BQ25756E_CHARGER_CONTROL_WD_RST          0x20  // Bit 5: I2C watchdog timer reset control: 0b = Normal, 1b = Reset (auto-clears)
#define BQ25756E_CHARGER_CONTROL_DIS_CE_PIN      0x10  // Bit 4: /CE pin function disable: 0b = Enabled, 1b = Disabled
#define BQ25756E_CHARGER_CONTROL_EN_CHG_BIT_RST  0x08  // Bit 3: EN_CHG bit behavior control: 0b = Resets to 0, 1b = Resets to 1 when watchdog expires
#define BQ25756E_CHARGER_CONTROL_EN_HIZ          0x04  // Bit 2: HIZ mode enable: 0b = Disabled, 1b = Enabled
#define BQ25756E_CHARGER_CONTROL_EN_IBAT_LOAD    0x02  // Bit 1: Battery load (IBAT_LOAD) enable: 0b = Disabled, 1b = Enabled
#define BQ25756E_CHARGER_CONTROL_EN_CHG          0x01  // Bit 0: Charge enable control: 0b = Disable charging, 1b = Enable charging

// -----------------------------------------------------------------------------
// REG_PIN_CONTROL (0x18) - Pin Control Register
// -----------------------------------------------------------------------------
#define BQ25756E_PIN_CONTROL                     0xFF  // Bit 7-0: Pin control register
#define BQ25756E_PIN_CONTROL_EN_ICHG_PIN         0x80  // Bit 7: ICHG pin function enable (0 = Disabled, 1 = Enabled)
#define BQ25756E_PIN_CONTROL_EN_ILIM_HIZ_PIN     0x40  // Bit 6: ILIM_HIZ pin function enable (0 = Disabled, 1 = Enabled)
#define BQ25756E_PIN_CONTROL_DIS_PG_PIN          0x20  // Bit 5: PG pin function disable (0 = Enabled, 1 = Disabled)
#define BQ25756E_PIN_CONTROL_DIS_STAT_PINS       0x10  // Bit 4: STAT pins function disable (0 = Enabled, 1 = Disabled)
#define BQ25756E_PIN_CONTROL_FORCE_STAT4_ON      0x08  // Bit 3: Force CE_STAT4 on (0 = Off, 1 = Pulls LOW)
#define BQ25756E_PIN_CONTROL_FORCE_STAT3_ON      0x04  // Bit 2: Force PG_STAT3 on (0 = Off, 1 = Pulls LOW)
#define BQ25756E_PIN_CONTROL_FORCE_STAT2_ON      0x02  // Bit 1: Force STAT2 on (0 = Off, 1 = Pulls LOW)
#define BQ25756E_PIN_CONTROL_FORCE_STAT1_ON      0x01  // Bit 0: Force STAT1 on (0 = Off, 1 = Pulls LOW)

// -----------------------------------------------------------------------------
// REG_POWER_PATH_REVERSE_MODE_CONTROL (0x19) - Power Path & Reverse Mode Control
// -----------------------------------------------------------------------------
#define BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL             0xE1  // Bit 7-0: Power path & reverse mode control register
#define BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL_REG_RST     0x80  // Bit 7: Register reset control (auto-clear)
#define BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL_EN_IAC_LOAD 0x40  // Bit 6: VAC Load (IAC_LOAD) enable: 0b = Disabled, 1b = Enabled
#define BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL_EN_PFM      0x20  // Bit 5: PFM mode enable in light-load: 0b = Fixed-frequency operation, 1b = PFM operation
// Bits 4-1 are reserved.
#define BQ25756E_POWER_PATH_REVERSE_MODE_CONTROL_EN_REV      0x01  // Bit 0: Reverse Mode control: 0b = Disable reverse mode, 1b = Enable reverse mode

// -----------------------------------------------------------------------------
// REG_MPPT_CONTROL (0x1A) - MPPT Control Register
// -----------------------------------------------------------------------------
#define BQ25756E_MPPT_CONTROL                      0x87  // Bit 7-0: MPPT control register
#define BQ25756E_MPPT_CONTROL_FORCE_SWEEP          0x80  // Bit 7: Force Full Panel Sweep and reset MPPT timers (auto-clear) 0b = Normal, 1b = Start full panel sweep
// Bits 6-3 are reserved
#define BQ25756E_MPPT_CONTROL_FULL_SWEEP_TMR_MASK  0x06  // Bits 2:1: Full Panel Sweep timer control: 00b = 3 min, 01b = 10 min, 10b = 15 min, 11b = 20 min
#define BQ25756E_MPPT_CONTROL_EN_MPPT              0x01  // Bit 0: MPPT algorithm control: 0b = Disable MPPT, 1b = Enable MPPT

// -----------------------------------------------------------------------------
// REG_TS_CHARGING_THRESHOLD_CONTROL (0x1B) - TS Charging Threshold Control - See datasheet for details
// -----------------------------------------------------------------------------
#define BQ25756E_TS_CHARGING_THRESHOLD_TS_T5  0xC0  // Bits 7:6: TS T5 (HOT) threshold control
#define BQ25756E_TS_CHARGING_THRESHOLD_TS_T3  0x30  // Bits 5:4: TS T3 (WARM) threshold control
#define BQ25756E_TS_CHARGING_THRESHOLD_TS_T2  0x0C  // Bits 3:2: TS T2 (COOL) threshold control
#define BQ25756E_TS_CHARGING_THRESHOLD_TS_T1  0x03  // Bits 1:0: TS T1 (COLD) threshold control

// -----------------------------------------------------------------------------
// REG_TS_CHARGING_REGION_BEHAVIOR_CONTROL (0x1C) - TS Charging Region Behavior Control
// -----------------------------------------------------------------------------
#define BQ25756E_TS_CHARGING_REGION_BEHAVIOR                   0x7F  // Bit 7-0: TS Charging Region Behavior Control
// Bit 7 is reserved
#define BQ25756E_TS_CHARGING_REGION_BEHAVIOR_JEITA_VSET_MASK   0x60  // Bits 6-5: JEITA Warm regulation voltage setting as percentage of VFB_REG:
                                                                     // 00b = Charge Suspend 01b = 94.3% x VFB_REG 10b = 97.6% x VFB_REG 11b = 100% x VFB_REG
#define BQ25756E_TS_CHARGING_REGION_BEHAVIOR_JEITA_ISETH       0x10  // Bit 4: JEITA Warm regulation current setting as percentage of ICHG_REG:
                                                                     // 0b = 40% x ICHG_REG 1b = 100% x ICHG_REG
#define BQ25756E_TS_CHARGING_REGION_BEHAVIOR_JEITA_ISETC_MASK  0x0C  // Bits 3-2: JEITA Cool regulation current setting
                                                                     // 00b = Charge Suspend 01b = 20% x ICHG_REG 10b = 40% x ICHG_REG 11b = 100% x ICHG_REG
#define BQ25756E_TS_CHARGING_REGION_BEHAVIOR_EN_JEITA          0x02  // Bit 1: JEITA profile control (0 = Disabled (COLD/HOT control only), 1 = Enabled (COLD/COOL/WARM/HOT control))
#define BQ25756E_TS_CHARGING_REGION_BEHAVIOR_EN_TS             0x01  // Bit 0: TS pin function control (0 = Ignore TS pin, 1 = Enable TS function)

// -----------------------------------------------------------------------------
// REG_TS_REVERSE_MODE_THRESHOLD_CONTROL (0x1D) - TS Reverse Mode Threshold Control
// -----------------------------------------------------------------------------
#define BQ25756E_TS_REVERSE_MODE_THRESHOLD_CONTROL   0xE0   // Bit 7-5: TS Reverse Mode Threshold Control
#define BQ25756E_TS_REVERSE_MODE_THRESHOLD_BHOT_MASK 0xC0   // Bits 7-6: Reverse Mode TS HOT threshold control: 
                                                            // 00b = 37.7% (55C) 01b = 34.2% (60C) 10b = 31.25%(65C) 11b = Disable
#define BQ25756E_TS_REVERSE_MODE_THRESHOLD_BCOLD     0x20   // Bit 5: Reverse Mode TS COLD threshold control: 0b = 77.15% (-10C) 1b = 80% (-20C)
// Bits 4:0 are reserved

// -----------------------------------------------------------------------------
// REG_REVERSE_UNDERVOLTAGE_CONTROL (0x1E) - Reverse Undervoltage Control
// -----------------------------------------------------------------------------
// Bit 7-6 are reserved
#define BQ25756E_REVERSE_UNDERVOLTAGE_SYSREV_UV  0x20  // Bit 5: Reverse Mode system undervoltage control: 0 = 80% of VSYS_REV target, 1 = Fixed at 3.3V
// Bits 4:0 are reserved

// -----------------------------------------------------------------------------
// REG_VAC_MAX_POWER_POINT_DETECTED (0x1F) - VAC Max Power Point Detected
// -----------------------------------------------------------------------------
// Bit 15-14 are reserved
#define BQ25756E_VAC_MAX_POWER_POINT_DETECTED_MASK  0x3FFC  // Bits 13:2: Input Voltage for Max Power Point detected
                                                            // POR: 0mV (0h)
                                                            // Range: 0mV-60000mV (0h-BB8h)
                                                            // Clamped High
                                                            // Bit Step: 20mV
// Bit 1-0 are reserved

// -----------------------------------------------------------------------------
// REG_CHARGER_STATUS_1 (0x21) - Read Only
// -----------------------------------------------------------------------------
#define BQ25756E_CHG_STATUS_1_ADC_DONE_STAT   0x80  // Bit 7: ADC conversion status
#define BQ25756E_CHG_STATUS_1_IAC_DPM_STAT    0x40  // Bit 6: Input current regulation
#define BQ25756E_CHG_STATUS_1_VAC_DPM_STAT    0x20  // Bit 5: Input voltage regulation
// Bit 4 is reserved
#define BQ25756E_CHG_STATUS_1_WD_STAT         0x08  // Bit 3: I2C watchdog timer
#define BQ25756E_CHG_STATUS_1_CHARGE_STAT_2   0x07  // Bit 4-0: Charge cycle status: 
                                                    // 000b = Not charging
                                                    // 001b = Trickle Charge (VBAT < VBAT_SHORT)
                                                    // 010b = Pre-Charge (VBAT < VBAT_LOWV)
                                                    // 011b = Fast Charge (CC mode)
                                                    // 100b = Taper Charge (CV mode)
                                                    // 101b = Reserved
                                                    // 110b = Top-off Timer Charge
                                                    // 111b = Charge Termination Done

// -----------------------------------------------------------------------------
// REG_CHARGER_STATUS_2 (0x22) - Read Only
// -----------------------------------------------------------------------------
#define BQ25756E_CHG_STATUS_2_PG_STAT         0x80  // Bit 7: Power Good status 0b: Not PG - 1b: PG
#define BQ25756E_CHG_STATUS_2_TS_STAT         0x70  // Bit 6-4: TS (Battery NTC) status:
                                                    // 000b = Normal
                                                    // 001b = TS Warm
                                                    // 010b = TS Cool
                                                    // 011b = TS Colde
                                                    // 100b = TS Hot
// Bits 3-2 are reserved
#define BQ25756E_CHG_STATUS_2_MPPT_STAT       0x03  // Bit 1-0: MPPT status
                                                    // 00b = MPPT disabled
                                                    // 01b = MPPT enabled, but not running
                                                    // 10b = Full panel sweep in progress
                                                    // 11b = Max power voltage detected

// -----------------------------------------------------------------------------
// REG_CHARGER_STATUS_3 (0x23) - Read Only
// -----------------------------------------------------------------------------
// Bits 7-6 are reserved 
#define BQ25756E_CHG_STATUS_3_FSW_SYNC_STAT    0x30 // Bit 5-4: FSW_SYNC pin status
                                                    // 00b = Normal, no external clock detected
                                                    // 01b = Valid ext. clock detected
                                                    // 10b = Pin fault (frequency out-of-range)
                                                    // 11b = Reserved
#define BQ25756E_CHG_STATUS_3_CV_TMR_STAT      0x08 // Bit 3: CV timer status 0b = Normal - 1b = Expired
#define BQ25756E_CHG_STATUS_3_REVERSE_STAT     0x04 // Bit 2: Converter Reverse Mode status 0b = Reverse mode disabled - 1b = Reverse mode enabled
// Bits 1-0 are reserved

// -----------------------------------------------------------------------------
// REG_FAULT_STATUS (0x24) - Read Only (Clear on Read)
// -----------------------------------------------------------------------------
#define BQ25756E_FAULT_STATUS_VAC_UV_FLAG       0x80  // Bit 7: Input UV 0b: Normal - 1b: Device in UVLO
#define BQ25756E_FAULT_STATUS_VAC_OV_FLAG       0x40  // Bit 6: Input OV 0b: Normal - 1b: Device in OVP
#define BQ25756E_FAULT_STATUS_IBAT_OCP_FLAG     0x20  // Bit 5: Battery over-current 0b: Normal - 1b: Battery OCP
#define BQ25756E_FAULT_STATUS_VBAT_OV_FLAG      0x10  // Bit 4: Battery over-voltage 0b: Normal - 1b: Device in OVP
#define BQ25756E_FAULT_STATUS_TSHUT_FLAG        0x08  // Bit 3: Thermal shutdown 0b: Normal - 1b: Device in thermal shutdown
#define BQ25756E_FAULT_STATUS_CHG_TMR_FLAG      0x04  // Bit 2: Charge safety timer expired 0b: Normal - 1b: Charge safety timer expired
#define BQ25756E_FAULT_STATUS_DRV_OKZ_FLAG      0x02  // Bit 1: DRV_SUP pin fault 0b: Normal - 1b: DRV_SUP pin voltage is out of valid range
// Bit 0 is reserved

// -----------------------------------------------------------------------------
// REG_CHARGER_FLAG_1 (0x25) - Charger Flag 1 Register
// -----------------------------------------------------------------------------
#define BQ25756E_CHARGER_FLAG_ADC_DONE    0x80  // Bit 7: ADC conversion INT flag (ClearOnRead) - 0b = Conversion not complete, 1b = Conversion complete
#define BQ25756E_CHARGER_FLAG_IAC_DPM     0x40  // Bit 6: Input Current regulation INT flag (ClearOnRead) - 0b = Normal, 1b = In Input Current regulation
#define BQ25756E_CHARGER_FLAG_VAC_DPM     0x20  // Bit 5: Input Voltage regulation INT flag (ClearOnRead)- 0b = Normal, 1b = In Input Voltage regulation
// Bits 4 is reserved.
#define BQ25756E_CHARGER_FLAG_WD          0x08  // Bit 3: I2C Watchdog timer INT flag (ClearOnRead) - 0b = Normal, 1b = WD timer rising edge detected
// Bit 2 is reserved.
#define BQ25756E_CHARGER_FLAG_CV_TMR      0x02  // Bit 1: CV timer INT flag (ClearOnRead) - 0b = Normal, 1b = CV timer expired (rising edge)
#define BQ25756E_CHARGER_FLAG_CHARGE      0x01  // Bit 0: Charge cycle INT flag (ClearOnRead) - 0b = Not charging, 1b = CHARGE status changed

// -----------------------------------------------------------------------------
// REG_CHARGER_FLAG_2 (0x26) - Charger Flag 2 Register
// -----------------------------------------------------------------------------
#define BQ25756E_CHARGER_FLAG2_PG         0x80  // Bit 7: PG flag (Input Power Good INT flag) (ClearOnRead) - 0b = Normal, 1b = PG signal toggle detected
// Bits 6-5 are reserved.
#define BQ25756E_CHARGER_FLAG2_TS         0x10  // Bit 4: TS flag (Battery NTC INT flag) (ClearOnRead) - 0b = Normal, 1b = TS status changed
#define BQ25756E_CHARGER_FLAG2_REVERSE    0x08  // Bit 3: Reverse Mode INT flag (ClearOnRead) - 0b = Normal, 1b = Reverse Mode toggle detected
// Bit 2 is reserved.
#define BQ25756E_CHARGER_FLAG2_FSW_SYNC   0x02  // Bit 1: FSW_SYNC pin signal INT flag (ClearOnRead) - 0b = Normal, 1b = FSW_SYNC status changed
#define BQ25756E_CHARGER_FLAG2_MPPT       0x01  // Bit 0: MPPT INT flag (ClearOnRead) - 0b = Normal, 1b = MPPT status changed

// -----------------------------------------------------------------------------
// REG_FAULT_FLAG (0x27) - Fault Flag Register
// -----------------------------------------------------------------------------
#define BQ25756E_FAULT_FLAG_VAC_UV    0x80  // Bit 7: Input under-voltage fault flag 0 = Normal, 1 = Entered input under-voltage fault
#define BQ25756E_FAULT_FLAG_VAC_OV    0x40  // Bit 6: Input over-voltage fault flag  0 = Normal, 1 = Entered Input over-voltage fault
#define BQ25756E_FAULT_FLAG_IBAT_OCP  0x20  // Bit 5: Battery over-current fault flag 0 = Normal, 1 = Entered Battery over-current fault
#define BQ25756E_FAULT_FLAG_VBAT_OV   0x10  // Bit 4: Battery over-voltage fault flag 0 = Normal, 1 = Entered Battery over-voltage fault
#define BQ25756E_FAULT_FLAG_TSHUT     0x08  // Bit 3: Thermal shutdown fault flag 0 = Normal, 1 = Entered TSHUT fault
#define BQ25756E_FAULT_FLAG_CHG_TMR   0x04  // Bit 2: Charge safety timer fault flag 0 = Normal, 1 = Charge Safety timer expired rising edge detected
#define BQ25756E_FAULT_FLAG_DRV_OKZ   0x02  // Bit 1: DRV_SUP pin voltage fault flag 0 = Normal, 1 = DRV_SUP pin fault detected
// Bit 0 is reserved.

// -----------------------------------------------------------------------------
// REG_CHARGER_MASK_1 (0x28) - Charger Mask 1 Register
// -----------------------------------------------------------------------------
#define BQ25756E_CHARGER_MASK1_ADC_DONE   0x80  // Bit 7: ADC_DONE mask: 0 = ADC_DONE produces INT pulse, 1 = ADC_DONE does not produce INT pulse
#define BQ25756E_CHARGER_MASK1_IAC_DPM    0x40  // Bit 6: IAC_DPM mask: 0 = IAC_DPM_FLAG produces INT pulse, 1 = IAC_DPM_FLAG does not produce INT pulse
#define BQ25756E_CHARGER_MASK1_VAC_DPM    0x20  // Bit 5: VAC_DPM mask: 0 = VAC_DPM_FLAG produces INT pulse, 1 = VAC_DPM_FLAG does not produce INT pulse
// Bit 4 is reserved.
#define BQ25756E_CHARGER_MASK1_WD         0x08  // Bit 3: WD mask: 0 = WD expiration produces INT pulse, 1 = WD expiration does not produce INT pulse
// Bit 2 is reserved.
#define BQ25756E_CHARGER_MASK1_CV_TMR     0x02  // Bit 1: CV timer mask: 0 = CV Timer expired rising edge produces INT pulse, 1 = CV Timer expired rising edge does not produce INT pulse
#define BQ25756E_CHARGER_MASK1_CHARGE     0x01  // Bit 0: Charge cycle mask: 0 = CHARGE_STAT change produces INT pulse, 1 = CHARGE_STAT change does not produce INT pulse

// -----------------------------------------------------------------------------
// REG_CHARGER_MASK_2 (0x29) - Charger Mask 2 Register
// -----------------------------------------------------------------------------
#define BQ25756E_CHARGER_MASK2_PG         0x80  // Bit 7: PG mask: 0 = PG toggle produces INT pulse, 1 = PG toggle does not produce INT pulse
// Bits 6-5 are reserved.
#define BQ25756E_CHARGER_MASK2_TS         0x10  // Bit 4: TS mask: 0 = TS_STAT change produces INT pulse, 1 = TS_STAT change does not produce INT pulse
#define BQ25756E_CHARGER_MASK2_REVERSE    0x08  // Bit 3: Reverse Mode mask: 0 = REVERSE_STAT toggle produces INT pulse, 1 = REVERSE_STAT toggle does not produce INT pulse
// Bit 2 is reserved.
#define BQ25756E_CHARGER_MASK2_FSW_SYNC   0x02  // Bit 1: FSW_SYNC mask: 0 = FSW_SYNC status change produces INT pulse, 1 = FSW_SYNC status change does not produce INT pulse
#define BQ25756E_CHARGER_MASK2_MPPT       0x01  // Bit 0: MPPT mask: 0 = MPPT_STAT rising edge produces INT pulse, 1 = MPPT_STAT rising edge does not produce INT pulse

// -----------------------------------------------------------------------------
// REG_FAULT_MASK (0x2A) - Fault Mask Register
// -----------------------------------------------------------------------------
#define BQ25756E_FAULT_MASK_VAC_UV    0x80  // Bit 7: Input under-voltage mask: 0 = Input under-voltage event produces INT pulse, 1 = Input under-voltage event does not produce INT pulse
#define BQ25756E_FAULT_MASK_VAC_OV    0x40  // Bit 6: Input over-voltage mask: 0 = Input over-voltage event produces INT pulse, 1 = Input over-voltage event does not produce INT pulse
#define BQ25756E_FAULT_MASK_IBAT_OCP  0x20  // Bit 5: Battery over-current mask: 0 = Battery over-current event produces INT pulse, 1 = Battery over-current event does not produce INT pulse
#define BQ25756E_FAULT_MASK_VBAT_OV   0x10  // Bit 4: Battery over-voltage mask: 0 = Battery over-voltage event produces INT pulse, 1 = Battery over-voltage event does not produce INT pulse
#define BQ25756E_FAULT_MASK_TSHUT     0x08  // Bit 3: Thermal shutdown mask: 0 = TSHUT event produces INT pulse, 1 = TSHUT event does not produce INT pulse
#define BQ25756E_FAULT_MASK_CHG_TMR   0x04  // Bit 2: Charge safety timer mask: 0 = Timer expired rising edge produces INT pulse, 1 = Timer expired rising edge does not produce INT pulse
#define BQ25756E_FAULT_MASK_DRV_OKZ   0x02  // Bit 1: DRV_SUP mask: 0 = DRV_SUP pin fault produces INT pulse, 1 = DRV_SUP pin fault does not produce INT pulse
// Bit 0 is reserved.

// -----------------------------------------------------------------------------
// REG_ADC_CONTROL (0x2B) - ADC Control Register
// -----------------------------------------------------------------------------
#define BQ25756E_ADC_CONTROL_ADC_EN       0x80  // Bit 7: ADC enable (0 = Disable, 1 = Enable)
#define BQ25756E_ADC_CONTROL_ADC_RATE     0x40  // Bit 6: ADC conversion rate (0 = Continuous, 1 = One-shot)
#define BQ25756E_ADC_CONTROL_ADC_SAMPLE   0x30  // Bits 5-4: ADC sample speed control: 
                                                // 00b = 15 bit effective resolution 01b = 14 bit effective resolution 10b = 13 bit effective resolution 11b = Reserved
#define BQ25756E_ADC_CONTROL_ADC_AVG      0x08  // Bit 3: ADC average control (0 = Single value, 1 = Running average)
#define BQ25756E_ADC_CONTROL_ADC_AVG_INIT 0x04  // Bit 2: ADC average initialization control: 0b = Start average using existing register value, 1b = Start average using new conversion
// Bits 1-0 are reserved.

// -----------------------------------------------------------------------------
// REG_ADC_CHANNEL_CONTROL (0x2C) - ADC Channel Control Register
// -----------------------------------------------------------------------------
#define BQ25756E_ADC_CHANNEL_IAC_ADC_DIS   0x80  // Bit 7: IAC ADC disable (0 = Enable, 1 = Disable)
#define BQ25756E_ADC_CHANNEL_IBAT_ADC_DIS  0x40  // Bit 6: IBAT ADC disable (0 = Enable, 1 = Disable)
#define BQ25756E_ADC_CHANNEL_VAC_ADC_DIS   0x20  // Bit 5: VAC ADC disable (0 = Enable, 1 = Disable)
#define BQ25756E_ADC_CHANNEL_VBAT_ADC_DIS  0x10  // Bit 4: VBAT ADC disable (0 = Enable, 1 = Disable)
// Bit 3 is reserved.
#define BQ25756E_ADC_CHANNEL_TS_ADC_DIS    0x04  // Bit 2: TS ADC disable (0 = Enable, 1 = Disable)
#define BQ25756E_ADC_CHANNEL_VFB_ADC_DIS   0x02  // Bit 1: VFB ADC disable (0 = Enable, 1 = Disable)
// Bit 0 is reserved.

// -----------------------------------------------------------------------------
// REG_IAC_ADC (0x2D) - IAC ADC Register - Read Only
// -----------------------------------------------------------------------------
#define BQ25756E_IAC_ADC_MASK  0xFFFF   // 16-bit IAC ADC reading with 5mΩ RAC_SNS: Reported as 2s complement
                                        // POR: 0mA(0h)
                                        // Format: 2s Complement
                                        // Range: -20000mA - 20000mA (9E58h-61A8h)
                                        // Clamped Low
                                        // Clamped High
                                        // Bit Step: 0.8mA

// -----------------------------------------------------------------------------
// REG_IBAT_ADC (0x2F) - IBAT ADC Register
// -----------------------------------------------------------------------------
#define BQ25756E_IBAT_ADC_MASK  0xFFFF  // 16-bit BAT ADC reading with 5mΩ RBAT_SNS: Reported as 2s complement
                                        // POR: 0mA (0h)
                                        // Format: 2s Complement
                                        // Range: -20000mA-20000mA (D8F0h-2710h)
                                        // Clamped Low
                                        // Clamped High
                                        // Bit Step: 2mA

// -----------------------------------------------------------------------------
// REG_VAC_ADC (0x31) - VAC ADC Register
// -----------------------------------------------------------------------------
#define BQ25756E_VAC_ADC_MASK  0xFFFF   // 16-bit VAC ADC reading: Reported as unsigned integer
                                        // POR: 0mV (0h)
                                        // Format: 2s Complement
                                        // Range: 0mV-60000mV (0h-7530h)
                                        // Clamped Low
                                        // Bit Step: 2mV

// -----------------------------------------------------------------------------
// REG_VBAT_ADC (0x33) - VBAT ADC Register
// -----------------------------------------------------------------------------
#define BQ25756E_VBAT_ADC_MASK  0xFFFF  // 16-bit VBAT ADC reading: Reported as unsigned integer
                                        // POR: 0mV (0h)
                                        // Format: 2s Complement
                                        // Range: 0mV-60000mV (0h-7530h)
                                        // Clamped Low
                                        // Bit Step: 2mV

// -----------------------------------------------------------------------------
// REG_TS_ADC (0x37) - TS ADC Register
// -----------------------------------------------------------------------------
#define BQ25756E_TS_ADC_MASK  0xFFFF    // 16-bit TS ADC reading as percentage of REGN: Reported as unsigned integer
                                        // POR: 0%(0h)
                                        // Range: 0% - 99.90234375% (0h-3FFh)
                                        // Clamped High
                                        // Bit Step: 0.09765625%

// -----------------------------------------------------------------------------
// REG_VFB_ADC (0x39) - VFB ADC Register
// -----------------------------------------------------------------------------
#define BQ25756E_VFB_ADC_MASK  0xFFFF   // 16-bit ADC reading for VFB (unsigned)
                                        // POR: 0mV (0h)
                                        // Range: 0mV-2047mV (0h-7FFh)
                                        // Clamped High
                                        // Bit Step: 1mV

// -----------------------------------------------------------------------------
// REG_GATE_DRIVER_STRENGTH_CONTROL (0x3B) - Gate Driver Strength Control
// -----------------------------------------------------------------------------
#define BQ25756E_BOOST_HS_DRV  0xC0  // Bits 7-6: Boost HS gate driver strength control
                                     // 00b = Fastest 01b = Faster 10b = Slower 11b = Slowest
#define BQ25756E_BUCK_HS_DRV   0x30  // Bits 5-4: Buck HS gate driver strength control
                                     // 00b = Fastest 01b = Faster 10b = Slower 11b = Slowest
#define BQ25756E_BOOST_LS_DRV  0x0C  // Bits 3-2: Boost LS gate driver strength control
                                     // 00b = Fastest 01b = Faster 10b = Slower 11b = Slowest
#define BQ25756E_BUCK_LS_DRV   0x03  // Bits 1-0: Buck LS gate driver strength control
                                     // 00b = Fastest 01b = Faster 10b = Slower 11b = Slowest

// -----------------------------------------------------------------------------
// REG_GATE_DRIVER_DEAD_TIME_CONTROL (0x3C) - Gate Driver Dead Time Control
// -----------------------------------------------------------------------------
#define BQ25756E_BOOST_DEAD_TIME_CONTROL  0x0F  
// Bit 7-4 are reserved
#define BQ25756E_BOOST_DEAD_TIME  0x0C  // Bits 3-2: Boost gate driver dead time control
                                        // 00b = 45ns 01b = 75ns 10b = 105ns 11b = 135ns
#define BQ25756E_BUCK_DEAD_TIME   0x03  // Bits 1-0: Buck gate driver dead time control
                                        // 00b = 45ns 01b = 75ns 10b = 105ns 11b = 135ns

// -----------------------------------------------------------------------------
// REG_PART_INFORMATION (0x3D) - Part Information Register
// -----------------------------------------------------------------------------
#define BQ25756E_PART_INFO        0x7F 
// Bit 7 is reserved
#define BQ25756E_PART_NUM         0x78  // Bits 6-3: Part number: 0110 - BQ25756E
#define BQ25756E_DEV_REV          0x07  // Bits 2-0: Device revision

// -----------------------------------------------------------------------------
// REG_REVERSE_MODE_BATTERY_DISCHARGE_CURRENT (0x62) - Reverse Mode Battery Discharge Current
// -----------------------------------------------------------------------------
#define BQ25756E_IBAT_REV_CONTROL               0xC2   //
#define BQ25756E_IBAT_REV               0xC0   // Bits 7-6: Reverse Mode battery discharge current control
                                               // 00b = 20A 01b = 15A 10b = 10A 11b = 5A
// Bit 5-2 are reserved
#define BQ25756E_EN_CONV_FAST_TRANSIENT 0x02   // Bit 1: Enable converter fast transient response
// Bit 0 is reserved

// Additional enums (e.g., for timer settings or MPPT modes) can be added here

// --- CONFIGURATION STRUCTURE ---
struct BQ25756E_Config {
    uint16_t chargeVoltageLimit;        // in mV; valid range and resolution as per datasheet
    uint16_t chargeCurrentLimit;        // in mA; fast charge current regulation limit
    uint16_t inputCurrentDPMLimit;      // in mA; input current DPM regulation limit
    uint16_t inputVoltageDPMLimit;      // in mV; input voltage regulation limit
    uint16_t prechargeCurrentLimit;     // in mA; precharge current regulation limit
    uint16_t terminationCurrentLimit;   // in mA; termination current limit
    bool terminationControlEnabled;     // Enable or disable termination current control
    uint8_t fastChargeThreshold;        // 2-bit termination threshold for fast charge: 00b = 30% x VFB_REG 01b = 55% x VFB_REG 10b = 66.7% x VFB_REG 11b = 71.4% x VFB_REG
    bool prechargeControlEnabled;       // Enable or disable precharge current control
    uint8_t topOffTimer;                // 2-bit top-off timer control: 00b = 0 min 01b = 5 min 10b = 10 min 11b = 15 min
    uint8_t watchdogTimer;              // 2-bit watchdog timer control: 00b = 40s 01b = 80s 10b = 160s 11b = 320s
    bool safetyTimerEnabled;            // Enable or disable charge safety timer
    uint8_t safetyTimer;                // 2-bit charge safety timer control: 00b = 5 min 01b = 10 min 10b = 15 min 11b = 20 min
    bool safetyTimerSpeed;              // Enable or disable fast charge speed during DPM
    uint8_t constantVoltageTimer;       // 4-bit constant voltage timer control: 0000b = 0 min 0001b = 5 min 0010b = 10 min ... 1111b = 75 min
    uint8_t autoRechargeThreshold;      // 2-bit auto-recharge threshold control: 00b = 0% 01b = 5% 10b = 10% 11b = 15%
    bool watchdogTimerResetEnabled;     // Enable or disable watchdog timer reset
    bool CEPinEnabled;                  // Enable or disable CE pin control
    bool ChargeBehaviorWatchdogExpired; // Control charge behavior when watchdog timer expires
    bool highZModeEnabled;              // Enable or disable high-Z mode
    bool batteryLoadEnabled;            // Enable or disable battery load
    bool chargeEnabled;                 // Enable or disable charging

    bool enableMPPT;                    // Enable or disable MPPT control
    bool verbose;                       // Enable verbose debug output

};

// --- BQ25756E CLASS ---
class BQ25756E {
private:
    // Hardware dependant settings
    uint8_t address;         // I2C address of the charger
    uint16_t switching_freq;  // Switching frequency of the charger in kHz
    uint8_t regulation_voltage;  // Regulation voltage of the charger in V
    uint16_t max_charge_current; // Maximum charge current in mA
    uint16_t max_input_current;  // Maximum input current in mA
    uint16_t min_voltage;        // Minimum voltage in mV
    uint16_t max_voltage;        // Maximum voltage in mV

    // Configuration settings, all in a single struct
    BQ25756E_Config config;

    // Debug port
    Stream* _debugPort = nullptr;

    // Private functions
    void chargPrint(const char* message);

public:
    // Constructor
    BQ25756E(uint8_t addr, uint16_t freq, uint16_t max_out_current, uint16_t max_in_current, uint16_t min_volt, uint16_t max_volt) : 
            address(addr),  switching_freq(freq), max_charge_current(max_out_current), max_input_current(max_in_current), 
            min_voltage(min_voltage), max_voltage(max_volt) {}

    // Initialization function
    void init(const BQ25756E_Config& cfg);

    // --- Helper Functions ---
    uint16_t getChargeVoltageLimitRegister();
    uint16_t getChargeVoltageLimit();

    uint16_t getChargeCurrentLimitRegister();
    uint16_t getChargeCurrentLimit();

    uint16_t getInputCurrentDPMLimitRegister();
    uint16_t getInputCurrentDPMLimit();

    uint16_t getInputVoltageDPMLimitRegister();
    uint16_t getInputVoltageDPMLimit();

    uint16_t getReverseModeInputCurrentLimitRegister();
    uint16_t getReverseModeInputCurrentLimit();

    uint16_t getReverseModeInputVoltageLimitRegister();
    uint16_t getReverseModeInputVoltageLimit();

    uint16_t getPrechargeCurrentLimitRegister();
    uint16_t getPrechargeCurrentLimit();

    uint16_t getTerminationCurrentLimitRegister();
    uint16_t getTerminationCurrentLimit();

    uint8_t  getPrechargeTerminationControl();
    uint8_t  getTimerControl();
    uint8_t  getThreeStageChageControl();
    uint8_t  getChargerControl();
    uint8_t  getPinControl();
    uint8_t  getPowerPathReverseModeControl();
    uint8_t  getMPPTControl();
    uint8_t  getTSChargingThresholdControl();
    uint8_t  getTSChargingRegionBehaviorControl();
    uint8_t  getTSReverseModeThresholdControl();
    uint8_t  getReverseUndervoltageControl();

    uint16_t getVACMaxPowerPointDetectedRegister();
    uint16_t getVACMaxPowerPointDetected();

    uint8_t  getChargerStatus1();
    uint8_t  getChargeCycleStatus();
    uint8_t  getChargerStatus2();
    uint8_t  getChargerStatus3();
    uint8_t  getFaultStatus();
    uint8_t  getChargerFlag1();
    uint8_t  getChargerFlag2();
    uint8_t  getFaultFlag();
    uint8_t  getChargerMask1();
    uint8_t  getChargerMask2();
    uint8_t  getFaultMask();
    uint8_t  getADCControl();
    uint8_t  getADCChannelControl();

    uint16_t getIACADCRegister();
    uint16_t getIACADC();

    uint16_t getIBATADCRegister();
    uint16_t getIBATADC();

    uint16_t getVACADCRegister();
    uint16_t getVACADC();

    uint16_t getVBATADCRegister();
    uint16_t getVBATADC();

    uint16_t getTSADCRegister();
    double   getTSADC();

    uint16_t getVFBADCRegister();
    uint16_t getVFBADC();

    uint8_t  getGateDriverStrengthControl();
    uint8_t  getGateDriverDeadTimeControl();
    uint8_t  getPartInformation();
    uint8_t  getReverseModeBatteryDischargeCurrent();

    // --- Control Functions ---
    void setChargeVoltageLimit(uint16_t voltage_mV);
    void setChargeCurrentLimit(uint16_t current_mA);
    void setInputCurrentLimit(uint16_t current_mA);
    void setInputVoltageDPM(uint16_t voltage_mV = 4200);
    void setPrechargeCurrentLimit(uint16_t current_mA);
    void setTerminationCurrentLimit(uint16_t current_mA);
    void configurePrechargeTermination(bool enable_termination_control, uint8_t threshold_fast_charge, bool enable_precharge_control);
    void configureTopOffTimer(uint8_t top_off_timer = 0b00);
    void configureWatchdogTimer(uint8_t watchdog_timer = 0b00);
    void configureChargeSafetyTimer(bool enable_safety_timer = false, uint8_t safety_timer = 0b00, bool speed_during_DPM = false);
    void configureConstantVoltageTimer(uint8_t CV_timer = 0b0000);
    void setAutoRechargeThreshold(uint8_t auto_recharge_threshold = 0b00);
    void configureWatchdogTimerReset(bool enable_watchdog_reset = false);
    void setCEpin(bool enable_CE_pin = true);
    void configureChargeBehaiorWatchdogExpires(bool disable_charge = false);
    void setHighZmode(bool enable_highZ = false);
    void enableControlledDischarge(bool enable_discharge = false);
    void enableCharge();
    void disableCharge();
    void enablePins(bool enable_ICHG = true, bool enableILIM_HIZ = true, bool enable_PG = true, bool enable_STAT = true);
    void resetRegisters();
    void configureVACLoad(bool enable_load = false);
    void setPFMMode(bool enable_PFM = false);
    void setReverseMode(bool enable_reverse = false);
    void setTSPinFunction(bool enable_TS = false);
    void configureADC(bool enable_ADC = true, bool one_shot = false, uint8_t sample_speed = 0b00, bool running_avg = true, bool init_avg = false);
    void configureADCChannel(bool enable_IAC = true, bool enable_IBAT = true, bool enable_VAC = true, bool enable_VBAT = true, bool enable_TS = true, bool enable_VFB = true);

    // --- Other Functions ---
    void printChargerConfig(bool initial_config = false);
    void setDebugStream(Stream* debugPort);
    void printByteAsBinary(uint8_t value);
    void print2BytesAsBinary(uint16_t value);
};

#endif // BQ25756E_H
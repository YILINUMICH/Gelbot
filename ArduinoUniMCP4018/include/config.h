// config.h — pin assignments and board constants for the SMA Variable Voltage Controller
//
// The board's signal headers (J3/J4/J6/J8/J9) are single-pin flying leads, so the
// Arduino Uno pins below are a wiring CHOICE. Re-wire and edit here if you move them.
//
// Source: doc/Schematic PDF_[No Variations].pdf (PN 008-300A-0003, Rev A)

#pragma once

#include <Arduino.h>

// ----------------------------------------------------------------------------
// Serial
// ----------------------------------------------------------------------------
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 115200
#endif

// ----------------------------------------------------------------------------
// Pin assignments (Arduino Uno)
// ----------------------------------------------------------------------------
// I2C to digipot (MCP4018): SCL=A5, SDA=A4 are fixed by the ATmega328P hardware TWI.
//   Board nets: J3 = MCU_SCL, J4 = MCU_SDA.

// SMA enable -> gate of low-side MOSFET Q1 (board net SMA_EN, header J9).
// HIGH = SMA current path enabled, LOW = disabled.
constexpr uint8_t PIN_SMA_EN = 8;        // D8

// INA296A1 current-sense amplifier output (board net ISNS_OUT, header J8).
constexpr uint8_t PIN_ISNS = A0;         // A0

// V_LDO sense via on-board R9/R11 (47k/47k) divider (header J6). Reads V_LDO / 2.
constexpr uint8_t PIN_VLDO_SENSE = A1;   // A1

// ----------------------------------------------------------------------------
// Analog / measurement constants
// ----------------------------------------------------------------------------
// ADC reference. Uno default analogReference is DEFAULT = AVcc (~5.0 V from USB/regulator).
// Measure your actual 5V rail and set this for best accuracy, or use the `vref` command.
constexpr float ADC_VREF_DEFAULT = 5.0f;
constexpr float ADC_COUNTS       = 1023.0f;  // 10-bit ADC full scale

// Current sense: INA296A1 fixed gain = 10 V/V (confirmed in doc/ina296a.pdf).
// Shunt R1 = 0.33 ohm. I_sma = V_isns / (INA_GAIN * RSHUNT).
constexpr float INA_GAIN = 10.0f;
constexpr float RSHUNT   = 0.33f;        // ohms

// V_LDO sense divider: R9 = R11 = 47k, midpoint = V_LDO * R11/(R9+R11) = V_LDO/2.
// So V_LDO = Vadc * (R9 + R11) / R11.
constexpr float R9_OHMS  = 47000.0f;
constexpr float R11_OHMS = 47000.0f;
constexpr float VLDO_DIVIDER = (R9_OHMS + R11_OHMS) / R11_OHMS;  // = 2.0

// Derived: maximum current measurable before the INA296 output saturates near its
// 5 V supply rail. I_max ~= (ADC_VREF) / (INA_GAIN * RSHUNT). Informational only.
// = 5.0 / (10 * 0.33) ~= 1.515 A (practically ~1.45 A allowing for output swing).

// ----------------------------------------------------------------------------
// Digipot (MCP4018) — 7-bit, 128 wiper steps, 100k (104) device.
// ----------------------------------------------------------------------------
constexpr uint8_t WIPER_MAX = 127;       // valid codes 0..127

// Settling time after changing the wiper before the LDO output is considered stable.
// Used during calibration; tune if your LDO output ring/settle differs.
constexpr uint16_t WIPER_SETTLE_MS = 50;

// Number of ADC samples averaged per measurement (noise reduction).
constexpr uint8_t ADC_AVG_SAMPLES = 16;

// ----------------------------------------------------------------------------
// Cycle logging
// ----------------------------------------------------------------------------
// Sampling interval used while running a `cycle` (CSV row cadence), milliseconds.
constexpr uint16_t LOG_INTERVAL_MS = 10;

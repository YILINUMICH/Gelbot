// MCP4018.h — minimal driver for the Microchip MCP4017/18/19 (and MCP40D17/18/19)
// 7-bit single I2C digital potentiometer / rheostat.
//
// Why a custom library:
//   The board is populated with an MCP4018 (PN MCP4018T-104E/LT, 100k), but the
//   datasheet shipped with the design (doc/MCP4017-18-19-Data-Sheet-DS20002152.pdf)
//   is actually the MCP40D family (DS20002152). The two variants differ in BOTH the
//   I2C address and the write framing:
//
//     * MCP4017/18/19  : slave address 0x2F, "simple" write = [addr+W][data]
//     * MCP40D17/18/19 : slave address 0x2E (MCP40D18 also 0x3E),
//                        write = [addr+W][command code 0x00][data]
//
//   Rather than guess which part is on the bench unit, this driver scans the bus,
//   identifies which address ACKs, and selects the matching write framing
//   automatically. That doubles as the "check the digipot connection" check.
//
//   Wiper register is 7-bit: valid codes 0..127. The MSb of the data byte is a
//   "don't care". Power-on default is mid-scale (code 64).

#pragma once

#include <Arduino.h>

class MCP4018 {
public:
    enum class Variant : uint8_t {
        Unknown = 0,
        MCP4018,    // address 0x2F, simple framing  [addr][data]
        MCP40D,     // address 0x2E/0x3E, command-code framing [addr][0x00][data]
    };

    // Candidate 7-bit I2C addresses, in scan order.
    static constexpr uint8_t ADDR_MCP4018  = 0x2F;  // MCP4017/18/19
    static constexpr uint8_t ADDR_MCP40D   = 0x2E;  // MCP40D17/18/19
    static constexpr uint8_t ADDR_MCP40D_2 = 0x3E;  // MCP40D18 alternate

    static constexpr uint8_t WIPER_MAX = 127;       // 7-bit, 128 steps

    MCP4018() = default;

    // Call after Wire.begin(). Probes the bus and locks onto the first device that
    // ACKs. Returns true if a digipot was found.
    bool begin();

    // Re-probe the bus (does not change cached wiper). Returns true if found.
    bool detect();

    // True if a device was found and is still ACKing right now.
    bool isConnected();

    // Set the wiper. `code` is clamped to 0..127. Returns true on a successful
    // I2C transfer. No-op-fails (returns false) if no device has been detected.
    bool setWiper(uint8_t code);

    // Accessors describing the detected device.
    bool     found()   const { return _found; }
    uint8_t  address() const { return _addr; }
    Variant  variant() const { return _variant; }
    const char* variantName() const;

    // Last value written via setWiper(), for reporting. 0xFF until first write.
    uint8_t lastCode() const { return _lastCode; }

private:
    // Returns true if `addr` ACKs an address-only probe on the I2C bus.
    static bool ackAt(uint8_t addr);

    bool    _found    = false;
    uint8_t _addr     = 0;
    Variant _variant  = Variant::Unknown;
    uint8_t _lastCode = 0xFF;
};

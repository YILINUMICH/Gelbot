// MCP4018.cpp — see MCP4018.h for protocol notes.

#include "MCP4018.h"
#include <Wire.h>

bool MCP4018::ackAt(uint8_t addr) {
    Wire.beginTransmission(addr);
    // endTransmission() with no payload performs an address-only probe and
    // returns 0 if the device ACKed the address.
    return (Wire.endTransmission() == 0);
}

bool MCP4018::detect() {
    _found   = false;
    _variant = Variant::Unknown;
    _addr    = 0;

    // Scan in order. Prefer the populated part (MCP4018 @ 0x2F) first.
    if (ackAt(ADDR_MCP4018)) {
        _addr = ADDR_MCP4018;
        _variant = Variant::MCP4018;
        _found = true;
    } else if (ackAt(ADDR_MCP40D)) {
        _addr = ADDR_MCP40D;
        _variant = Variant::MCP40D;
        _found = true;
    } else if (ackAt(ADDR_MCP40D_2)) {
        _addr = ADDR_MCP40D_2;
        _variant = Variant::MCP40D;
        _found = true;
    }
    return _found;
}

bool MCP4018::begin() {
    return detect();
}

bool MCP4018::isConnected() {
    if (!_found) return false;
    return ackAt(_addr);
}

bool MCP4018::setWiper(uint8_t code) {
    if (!_found) return false;
    if (code > WIPER_MAX) code = WIPER_MAX;   // clamp to 7-bit range

    Wire.beginTransmission(_addr);
    if (_variant == Variant::MCP40D) {
        // MCP40D framing: command code 0x00 selects the wiper register; a non-zero
        // command code is ignored by the device (no write performed).
        Wire.write((uint8_t)0x00);
    }
    Wire.write((uint8_t)(code & 0x7F));       // MSb is don't-care
    bool ok = (Wire.endTransmission() == 0);
    if (ok) _lastCode = code;
    return ok;
}

const char* MCP4018::variantName() const {
    switch (_variant) {
        case Variant::MCP4018: return "MCP4017/18/19";
        case Variant::MCP40D:  return "MCP40D17/18/19";
        default:               return "unknown";
    }
}

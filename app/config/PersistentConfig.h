#pragma once
#include "DeviceConfig.h"
#include "drivers/AT24Cxx.h"

namespace cfg {

// EEPROM-backed configuration store.
// load() must be called once in App::init(); save() is called on demand
// (e.g. when a CONFIG_WRITE command is received, or before power-down).
class PersistentConfig
{
public:
    static constexpr uint16_t EEPROM_ADDR = 0x0000; // base address in EEPROM

    explicit PersistentConfig(drv::AT24Cxx& eeprom);

    // Reads config from EEPROM.  If the magic sentinel is wrong (first boot
    // or corrupted), resets to defaults and saves.  Returns true if data
    // was read successfully (even if defaults were applied).
    bool load();

    // Writes current config to EEPROM.
    bool save();

    // Resets to compile-time defaults and saves.
    bool reset();

    DeviceConfig&       get()       { return _cfg; }
    const DeviceConfig& get() const { return _cfg; }

private:
    drv::AT24Cxx& _eeprom;
    DeviceConfig  _cfg{};
};

} // namespace cfg

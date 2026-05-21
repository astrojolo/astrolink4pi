#pragma once
#include "drivers/SHT30.h"
#include "drivers/MLX90641.h"
#include "drivers/TSL2561.h"
#include "drivers/INA219.h"
#include <cstdint>
#include <array>

namespace svc {

struct SensorData
{
    // SHT30
    float   ambientTempC  = 0.0f;
    float   humidityRH    = 0.0f;
    bool    envValid      = false;

    // TSL2561
    float   lux           = 0.0f;
    bool    luxValid      = false;

    // INA219
    float   busVoltageV   = 0.0f;
    float   currentA      = 0.0f;
    float   powerW        = 0.0f;
    bool    powerValid    = false;

    // MLX90641 (192 pixels, 16×12)
    std::array<float, drv::MLX90641::PIXEL_COUNT> irFrame{};
    float   ambientTa     = 0.0f;  // sensor Ta from MLX
    bool    irValid       = false;
};

// Non-blocking round-robin sensor polling.  Each poll() call advances one
// sensor, preventing I2C bus starvation of the main loop.
class SensorService
{
public:
    SensorService(drv::SHT30&     sht,
                  drv::MLX90641&  mlx,
                  drv::TSL2561&   tsl,
                  drv::INA219&    ina);

    bool init();

    // Call from main loop every iteration.  Each call reads at most one
    // sensor measurement (round-robin) to keep I2C latency bounded.
    void poll();

    const SensorData& data() const { return _data; }

private:
    drv::SHT30&    _sht;
    drv::MLX90641& _mlx;
    drv::TSL2561&  _tsl;
    drv::INA219&   _ina;

    SensorData _data{};
    uint8_t    _round = 0; // which sensor to read next
};

} // namespace svc

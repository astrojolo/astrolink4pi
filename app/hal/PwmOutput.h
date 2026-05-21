#pragma once
#include "main.h"
#include <cstdint>
#include <algorithm>

namespace hal {

// Wraps a single TIM PWM channel.  Assumes the timer was started by CubeMX
// (HAL_TIM_PWM_Start called here; calling init() a second time is safe).
class PwmOutput
{
public:
    PwmOutput(TIM_HandleTypeDef* htim, uint32_t channel)
        : _htim(htim), _channel(channel) {}

    void init()  { HAL_TIM_PWM_Start(_htim, _channel); }
    void deinit(){ HAL_TIM_PWM_Stop(_htim, _channel);  }

    // duty: 0.0 = off, 1.0 = full on
    void setDuty(float duty)
    {
        duty = std::clamp(duty, 0.0f, 1.0f);
        const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(_htim);
        __HAL_TIM_SET_COMPARE(_htim, _channel,
                              static_cast<uint32_t>(duty * static_cast<float>(arr)));
        _duty = duty;
    }

    void setDutyPercent(uint8_t pct)  { setDuty(pct / 100.0f); }
    float getDuty()       const        { return _duty; }
    uint8_t getDutyPercent() const     { return static_cast<uint8_t>(_duty * 100.0f + 0.5f); }

private:
    TIM_HandleTypeDef* _htim;
    uint32_t           _channel;
    float              _duty = 0.0f;
};

} // namespace hal

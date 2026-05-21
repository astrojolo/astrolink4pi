#pragma once
#include "main.h"
#include <cstdint>

namespace hal {

// Wraps UART_HandleTypeDef for blocking TX/RX.
// Used by TMC2209 (single-wire half-duplex) and by the host protocol layer.
class UartBus
{
public:
    explicit UartBus(UART_HandleTypeDef* huart) : _huart(huart) {}

    bool transmit(const uint8_t* data, uint16_t len, uint32_t timeoutMs = 10) const
    {
        return HAL_UART_Transmit(_huart,
                                 const_cast<uint8_t*>(data), len,
                                 timeoutMs) == HAL_OK;
    }

    bool receive(uint8_t* data, uint16_t len, uint32_t timeoutMs = 10) const
    {
        return HAL_UART_Receive(_huart, data, len, timeoutMs) == HAL_OK;
    }

    // Non-blocking single-byte peek; returns false if RX register empty.
    bool tryReceiveByte(uint8_t& byte) const
    {
        return HAL_UART_Receive(_huart, &byte, 1, 0) == HAL_OK;
    }

    void flushRx() const { __HAL_UART_FLUSH_DRREGISTER(_huart); }

    UART_HandleTypeDef* handle() const { return _huart; }

private:
    UART_HandleTypeDef* _huart;
};

} // namespace hal

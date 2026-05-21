#pragma once
#include "Frame.h"
#include "hal/UartBus.h"
#include <cstdint>
#include <functional>
#include <array>

namespace proto {

// Byte-by-byte RX state machine + routing table.
// poll() must be called from the main loop; it processes one complete frame
// per call (non-blocking if no byte is available).
class Dispatcher
{
public:
    // Returns true on success; reply frame is populated by the handler.
    using Handler = std::function<bool(const Frame& req, Frame& reply)>;

    explicit Dispatcher(hal::UartBus& uart);

    // Register a handler for a specific command.
    // Registering twice for the same cmd replaces the previous handler.
    void registerHandler(Cmd cmd, Handler handler);

    // Call from main loop.  Pulls bytes from UART, assembles frames, and
    // dispatches complete frames to registered handlers.  Sends ACK/NAK reply.
    void poll();

    // Explicit send helpers (also usable for unsolicited events)
    bool sendAck(Cmd cmd, const uint8_t* data = nullptr, uint8_t dataLen = 0);
    bool sendNak(Cmd cmd, Error err);
    bool sendEvent(uint8_t eventId, const uint8_t* data = nullptr, uint8_t dataLen = 0);

private:
    bool buildAndSendFrame(Cmd cmd, const uint8_t* payload, uint8_t payloadLen);
    bool processFrame(const Frame& frame);
    static uint8_t crc8(const uint8_t* data, uint8_t len);

    hal::UartBus& _uart;

    // ── RX state machine ─────────────────────────────────────────────────────
    enum class RxState { WAIT_START, WAIT_LEN, WAIT_CMD, PAYLOAD, WAIT_CRC };
    RxState  _rxState      = RxState::WAIT_START;
    Frame    _rxFrame      = {};
    uint8_t  _rxIdx        = 0;
    uint8_t  _rxExpected   = 0; // expected payload bytes remaining
    uint8_t  _rxRunningCrc = 0;

    // ── Handler table ─────────────────────────────────────────────────────────
    static constexpr uint8_t MAX_HANDLERS = 32;
    struct Entry { Cmd cmd; Handler fn; };
    std::array<Entry, MAX_HANDLERS> _handlers{};
    uint8_t _handlerCount = 0;
};

} // namespace proto

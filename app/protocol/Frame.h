#pragma once
#include <cstdint>

namespace proto {

// UART framing: [0xA5][LEN][CMD][PAYLOAD 0..LEN-1][CRC8]
// LEN = number of payload bytes (not including header/CRC).
// CRC8 covers all bytes from START through the last payload byte.

static constexpr uint8_t START_BYTE  = 0xA5u;
static constexpr uint8_t MAX_PAYLOAD = 64u;

struct Frame
{
    uint8_t cmd;
    uint8_t payload[MAX_PAYLOAD];
    uint8_t payloadLen;
};

enum class Cmd : uint8_t
{
    // ── Motor (focuser) ──────────────────────────────────────────────────────
    MOTOR_MOVE_ABS   = 0x10, // payload: int32_t position
    MOTOR_MOVE_REL   = 0x11, // payload: int32_t steps
    MOTOR_STOP       = 0x12,
    MOTOR_ABORT      = 0x13, // immediate disable
    MOTOR_GET_POS    = 0x14,
    MOTOR_SET_SPEED  = 0x15, // payload: float maxVelocity, float accel
    MOTOR_SET_CURRENT= 0x16, // payload: uint8_t irun, uint8_t ihold

    // ── Sensors ──────────────────────────────────────────────────────────────
    SENSOR_GET_ENV   = 0x20, // temperature + humidity (SHT30)
    SENSOR_GET_LUX   = 0x21, // TSL2561
    SENSOR_GET_POWER = 0x22, // INA219 bus/current/power
    SENSOR_GET_IR    = 0x23, // MLX90641 full frame

    // ── Power outputs ─────────────────────────────────────────────────────────
    POWER_DC_SET     = 0x30, // payload: uint8_t channel, uint8_t onOff
    POWER_DC_GET     = 0x31, // payload: uint8_t channel
    POWER_PWM_SET    = 0x32, // payload: uint8_t channel, uint8_t dutyPercent
    POWER_PWM_GET    = 0x33, // payload: uint8_t channel
    POWER_FAULTS_GET = 0x34,

    // ── Configuration ─────────────────────────────────────────────────────────
    CONFIG_READ      = 0x40,
    CONFIG_WRITE     = 0x41,
    CONFIG_RESET     = 0x42,

    // ── Scheduler ─────────────────────────────────────────────────────────────
    SCHED_ADD        = 0x50, // payload: ScheduledEvent struct
    SCHED_REMOVE     = 0x51, // payload: uint8_t index
    SCHED_GET        = 0x52, // payload: uint8_t index
    SCHED_GET_COUNT  = 0x53,

    // ── RTC ──────────────────────────────────────────────────────────────────
    RTC_SET          = 0x60, // payload: DateTime struct
    RTC_GET          = 0x61,

    // ── System ───────────────────────────────────────────────────────────────
    PING             = 0xF0,
    GET_VERSION      = 0xF1,
    RESET            = 0xF2,

    // ── Responses ────────────────────────────────────────────────────────────
    ACK              = 0x80, // payload: echo of cmd byte, then optional data
    NAK              = 0x81, // payload: echo of cmd byte, uint8_t errorCode
    EVENT            = 0x82, // unsolicited; payload: uint8_t eventId, data...
};

// Error codes used in NAK payloads
enum class Error : uint8_t
{
    NONE            = 0x00,
    UNKNOWN_CMD     = 0x01,
    BAD_LENGTH      = 0x02,
    BAD_CRC         = 0x03,
    OUT_OF_RANGE    = 0x04,
    HARDWARE_FAULT  = 0x05,
    NOT_READY       = 0x06,
};

} // namespace proto

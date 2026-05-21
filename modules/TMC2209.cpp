#include "TMC2209.h"
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Construction & initialisation
// ─────────────────────────────────────────────────────────────────────────────

TMC2209::TMC2209(const Config &config)
    : _config(config)
{
    if (_config.stepPulseNops == 0)
        _config.stepPulseNops = 50;
}

bool TMC2209::init()
{
    // Initial GPIO state: STEP low, DIR low, EN high (driver disabled)
    HAL_GPIO_WritePin(_config.stepPort,   _config.stepPin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(_config.dirPort,    _config.dirPin,    GPIO_PIN_RESET);
    HAL_GPIO_WritePin(_config.enablePort, _config.enablePin, GPIO_PIN_SET);
    _enabled = false;

    HAL_Delay(10);  // TMC2209 power-up settling

    // GCONF:
    //   bit 6  pdn_disable     = 1  → UART active, PDN pin tristated
    //   bit 7  mstep_reg_select= 1  → microstep resolution from CHOPCONF.MRES
    //   bit 8  multistep_filt  = 1  → recommended for StealthChop
    _gconf = (1u << 6) | (1u << 7) | (1u << 8);
    if (!writeRegister(Reg::GCONF, _gconf)) return false;

    // CHOPCONF reset value (TMC2209 datasheet default):
    //   TOFF=3, HSTRT=5, HEND=3, TBL=1, INTPOL=1 (interp. to 256), MRES=0 (256µstep)
    _chopconf = 0x10000053u;
    if (!writeRegister(Reg::CHOPCONF, _chopconf)) return false;

    // PWMCONF: StealthChop defaults (PWM_AUTOSCALE=1, PWM_AUTOGRAD=1)
    if (!writeRegister(Reg::PWMCONF, 0xC10D0024u)) return false;

    // TPOWERDOWN: ~2 s hold-to-power-down delay
    if (!writeRegister(Reg::TPOWERDOWN, 20u)) return false;

    // IHOLD_IRUN: default currents
    if (!setCurrentScaling(_irun, _ihold, _iholdDelay)) return false;

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Enable / Disable
// ─────────────────────────────────────────────────────────────────────────────

void TMC2209::enable()
{
    HAL_GPIO_WritePin(_config.enablePort, _config.enablePin, GPIO_PIN_RESET); // active-low
    _enabled = true;
}

void TMC2209::disable()
{
    stop();
    HAL_GPIO_WritePin(_config.enablePort, _config.enablePin, GPIO_PIN_SET);
    _enabled = false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Current control
// ─────────────────────────────────────────────────────────────────────────────

bool TMC2209::setCurrentScaling(uint8_t irun, uint8_t ihold, uint8_t iholdDelay)
{
    _irun       = irun       & 0x1Fu;
    _ihold      = ihold      & 0x1Fu;
    _iholdDelay = iholdDelay & 0x0Fu;

    const uint32_t val = ((uint32_t)_iholdDelay << 16) |
                         ((uint32_t)_irun        <<  8) |
                          (uint32_t)_ihold;
    return writeRegister(Reg::IHOLD_IRUN, val);
}

bool TMC2209::setRunCurrent(uint8_t irun)
{
    return setCurrentScaling(irun, _ihold, _iholdDelay);
}

bool TMC2209::setHoldCurrent(uint8_t ihold)
{
    return setCurrentScaling(_irun, ihold, _iholdDelay);
}

// ─────────────────────────────────────────────────────────────────────────────
// Chopper / mode configuration
// ─────────────────────────────────────────────────────────────────────────────

bool TMC2209::setMicrosteps(uint16_t microsteps)
{
    // MRES encoding: 256→0, 128→1, 64→2, 32→3, 16→4, 8→5, 4→6, 2→7, 1→8
    uint8_t mres;
    switch (microsteps)
    {
        case 256: mres = 0; break;
        case 128: mres = 1; break;
        case  64: mres = 2; break;
        case  32: mres = 3; break;
        case  16: mres = 4; break;
        case   8: mres = 5; break;
        case   4: mres = 6; break;
        case   2: mres = 7; break;
        case   1: mres = 8; break;
        default:  return false;
    }
    // CHOPCONF bits [27:24] = MRES; preserve INTPOL (bit 28) and the rest
    _chopconf = (_chopconf & ~(0x0Fu << 24)) | ((uint32_t)mres << 24);
    return writeRegister(Reg::CHOPCONF, _chopconf);
}

bool TMC2209::setSpreadCycle(bool enable)
{
    if (enable)
        _gconf |=  (1u << 2);   // en_spreadCycle = 1
    else
        _gconf &= ~(1u << 2);   // en_spreadCycle = 0 → StealthChop
    return writeRegister(Reg::GCONF, _gconf);
}

bool TMC2209::setDirectionInverted(bool invert)
{
    if (invert)
        _gconf |=  (1u << 3);   // shaft = 1
    else
        _gconf &= ~(1u << 3);
    return writeRegister(Reg::GCONF, _gconf);
}

bool TMC2209::setStealthChopThreshold(uint32_t tstep)
{
    return writeRegister(Reg::TPWMTHRS, tstep);
}

// ─────────────────────────────────────────────────────────────────────────────
// Motion commands
// ─────────────────────────────────────────────────────────────────────────────

void TMC2209::moveTo(int32_t absoluteSteps)
{
    if (absoluteSteps == _position && _state == State::IDLE) return;
    _target = absoluteSteps;
    _state  = State::MOVING;
}

void TMC2209::moveBy(int32_t relativeSteps)
{
    moveTo(_position + relativeSteps);
}

void TMC2209::stop()
{
    _target      = _position;
    _velocity    = 0.0f;
    _accumulator = 0.0f;
    _state       = State::IDLE;
}

// ─────────────────────────────────────────────────────────────────────────────
// Tick — trapezoidal velocity profile
//
// Call from a hardware timer ISR at exactly 1 / tickInterval Hz.
// Velocity is in steps/s; acceleration in steps/s².
// Direction changes are handled by decelerating to MIN_VELOCITY first.
// ─────────────────────────────────────────────────────────────────────────────

void TMC2209::tick()
{
    if (_state == State::IDLE || !_enabled) return;

    const int32_t stepsToGo    = _target - _position;
    const int32_t absToGo      = (stepsToGo >= 0) ? stepsToGo : -stepsToGo;
    const int8_t  newDirection = (stepsToGo >= 0) ? 1 : -1;

    if (absToGo == 0)
    {
        _state       = State::IDLE;
        _velocity    = 0.0f;
        _accumulator = 0.0f;
        return;
    }

    // Braking distance at current speed: v² / (2·a)
    const float brakeDist = (_velocity * _velocity) / (2.0f * _acceleration);

    if (newDirection != _direction && _velocity > MIN_VELOCITY)
    {
        // Direction reversal: decelerate before flipping DIR
        _velocity -= _acceleration * _config.tickInterval;
        if (_velocity < MIN_VELOCITY)
        {
            _velocity  = MIN_VELOCITY;
            _direction = newDirection;
        }
    }
    else if ((float)absToGo <= brakeDist)
    {
        // Within braking distance: decelerate
        _direction = newDirection;
        _velocity -= _acceleration * _config.tickInterval;
        if (_velocity < MIN_VELOCITY) _velocity = MIN_VELOCITY;
    }
    else
    {
        // Accelerate up to max speed
        _direction = newDirection;
        _velocity += _acceleration * _config.tickInterval;
        if (_velocity > _maxVelocity) _velocity = _maxVelocity;
    }

    // Update DIR pin once per tick, before any step pulses
    HAL_GPIO_WritePin(_config.dirPort, _config.dirPin,
                      (_direction > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Fractional-step accumulator
    _accumulator += _velocity * _config.tickInterval;

    while (_accumulator >= 1.0f)
    {
        stepPulse();
        _position    += _direction;
        _accumulator -= 1.0f;

        if (_position == _target)
        {
            _state       = State::IDLE;
            _velocity    = 0.0f;
            _accumulator = 0.0f;
            return;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// UART register access (single-wire: TX echo is consumed before reading reply)
// ─────────────────────────────────────────────────────────────────────────────

bool TMC2209::writeRegister(uint8_t reg, uint32_t value)
{
    uint8_t dg[8];
    dg[0] = SYNC_BYTE;
    dg[1] = _config.address;
    dg[2] = reg | 0x80u;              // Write flag
    dg[3] = (value >> 24) & 0xFFu;
    dg[4] = (value >> 16) & 0xFFu;
    dg[5] = (value >>  8) & 0xFFu;
    dg[6] =  value        & 0xFFu;
    dg[7] = calcCRC(dg, 7);

    if (HAL_UART_Transmit(_config.huart, dg, 8, UART_TIMEOUT) != HAL_OK)
        return false;

    // Consume TX echo (single-wire loopback)
    uint8_t echo[8];
    HAL_UART_Receive(_config.huart, echo, 8, UART_TIMEOUT);

    return true;
}

bool TMC2209::readRegister(uint8_t reg, uint32_t &value)
{
    // ── Send 4-byte read request ────────────────────────────────────────────
    uint8_t req[4];
    req[0] = SYNC_BYTE;
    req[1] = _config.address;
    req[2] = reg;                      // No write flag
    req[3] = calcCRC(req, 3);

    if (HAL_UART_Transmit(_config.huart, req, 4, UART_TIMEOUT) != HAL_OK)
        return false;

    // Consume echo of the request
    uint8_t echo[4];
    HAL_UART_Receive(_config.huart, echo, 4, UART_TIMEOUT);

    // ── Receive 8-byte reply ─────────────────────────────────────────────────
    // Reply format: [0x05][0xFF][reg|0x80][D3][D2][D1][D0][CRC]
    uint8_t resp[8] = {};
    if (HAL_UART_Receive(_config.huart, resp, 8, UART_TIMEOUT) != HAL_OK)
        return false;

    if (resp[0] != SYNC_BYTE)         return false;
    if (calcCRC(resp, 7) != resp[7])  return false;

    value = ((uint32_t)resp[3] << 24) |
            ((uint32_t)resp[4] << 16) |
            ((uint32_t)resp[5] <<  8) |
             (uint32_t)resp[6];
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Status
// ─────────────────────────────────────────────────────────────────────────────

uint32_t TMC2209::getDriverStatus()
{
    uint32_t val = 0;
    readRegister(Reg::DRVSTATUS, val);
    return val;
}

bool TMC2209::isStalled()
{
    const uint32_t s = getDriverStatus();
    // stst (bit 31) = 1 means standstill; SG_RESULT bits [9:0] = 0 means stall
    const bool standstill = (s >> 31) & 1u;
    const bool sg_zero    = (s & 0x1FFu) == 0u;
    return !standstill && sg_zero;
}

bool TMC2209::isOvertemp()
{
    // DRVSTATUS bit 25 = ot (overtemperature shutdown)
    return (getDriverStatus() >> 25) & 1u;
}

// ─────────────────────────────────────────────────────────────────────────────
// Private helpers
// ─────────────────────────────────────────────────────────────────────────────

void TMC2209::stepPulse()
{
    HAL_GPIO_WritePin(_config.stepPort, _config.stepPin, GPIO_PIN_SET);
    // Hold STEP high ≥ 100 ns (TMC2209 datasheet tSH).
    // 50 NOPs covers MCUs up to ~400 MHz; increase stepPulseNops for faster cores.
    for (volatile uint8_t i = 0; i < _config.stepPulseNops; i++) { __NOP(); }
    HAL_GPIO_WritePin(_config.stepPort, _config.stepPin, GPIO_PIN_RESET);
}

uint8_t TMC2209::calcCRC(const uint8_t *data, uint8_t len) const
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; ++i)
    {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; ++j)
        {
            if ((crc >> 7) ^ (b & 0x01u))
                crc = (crc << 1) ^ 0x07u;
            else
                crc <<= 1;
            b >>= 1;
        }
    }
    return crc;
}

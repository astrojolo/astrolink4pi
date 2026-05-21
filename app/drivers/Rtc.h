#pragma once
#include "main.h"
#include <cstdint>

namespace drv {

struct DateTime
{
    uint16_t year;
    uint8_t  month;    // 1–12
    uint8_t  day;      // 1–31
    uint8_t  hour;     // 0–23
    uint8_t  minute;   // 0–59
    uint8_t  second;   // 0–59
    uint8_t  weekday;  // 1=Monday … 7=Sunday
};

// Thin wrapper around the STM32 internal RTC peripheral via HAL.
class Rtc
{
public:
    explicit Rtc(RTC_HandleTypeDef* hrtc) : _hrtc(hrtc) {}

    bool     setDateTime(const DateTime& dt);
    DateTime getDateTime() const;

    // Seconds since 2000-01-01 00:00:00 (simple epoch for event scheduling)
    uint32_t getTimestamp() const;

private:
    RTC_HandleTypeDef* _hrtc;
};

} // namespace drv

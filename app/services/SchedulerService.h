#pragma once
#include "drivers/Rtc.h"
#include "drivers/AT24Cxx.h"
#include <array>
#include <cstdint>
#include <functional>

namespace svc {

// A scheduled event fires an action at a given time on selected weekdays.
struct ScheduledEvent
{
    uint8_t weekdayMask; // bit 0 = Mon … bit 6 = Sun; 0x7F = every day
    uint8_t hour;        // 0–23
    uint8_t minute;      // 0–59
    uint8_t actionId;    // maps to an Action registered via registerAction()
    bool    enabled;
};

// RTC-driven event table.  Events are persisted in EEPROM.
// tick() must be called from the main loop at least once per minute.
class SchedulerService
{
public:
    static constexpr uint8_t MAX_EVENTS  = 16;
    static constexpr uint8_t MAX_ACTIONS = 8;

    using Action = std::function<void()>;

    SchedulerService(drv::Rtc& rtc, drv::AT24Cxx& eeprom);

    void init();

    // Call from main loop.  Fires any event whose time matches the current
    // RTC minute (debounced — fires at most once per minute per event).
    void tick();

    // Register an action callback for a given id (0 … MAX_ACTIONS-1).
    void registerAction(uint8_t id, Action action);

    // Event table management
    bool    addEvent(const ScheduledEvent& event);  // returns false if table full
    bool    removeEvent(uint8_t index);
    bool    getEvent(uint8_t index, ScheduledEvent& out) const;
    uint8_t eventCount() const { return _eventCount; }

    void saveEvents();  // write _events[] to EEPROM
    void loadEvents();  // read _events[] from EEPROM

private:
    bool shouldFire(const ScheduledEvent& ev, const drv::DateTime& now) const;

    drv::Rtc&    _rtc;
    drv::AT24Cxx& _eeprom;

    std::array<ScheduledEvent, MAX_EVENTS> _events{};
    std::array<Action, MAX_ACTIONS>        _actions{};
    uint8_t _eventCount = 0;
    uint8_t _lastMinute = 0xFF; // sentinel — forces first evaluation

    static constexpr uint16_t EEPROM_SCHED_ADDR = 0x0100;
};

} // namespace svc

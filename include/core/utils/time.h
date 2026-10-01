#pragma once

#include "core/utils/units.h"
#include <cstdint>


namespace core {

/**
 * Platform-independent high-resolution monotonic timer.
 * ZERO vendor/VEX dependencies. Uses pluggable clock source hooked by platform/HAL layers.
 */
class Timer {
public:
    using ClockFn = uint64_t (*)();
    using DelayFn = void (*)(uint32_t ms);

    /// Register a platform-specific high-resolution microsecond clock source.
    static void set_clock_source(ClockFn fn);

    /// Register a platform-specific delay function.
    static void set_delay_provider(DelayFn fn);

    /// System time in microseconds.
    static uint64_t systemHighResolution();

    /// System time in milliseconds.
    static uint64_t system();

    /// System time in fractional seconds.
    static double system_seconds();

    /// Delay execution for specified milliseconds.
    static void delay_ms(uint32_t ms);

    /// Delay execution for specified microseconds.
    static void delay_us(uint64_t us);

    Timer();

    void reset();

    /// Returns elapsed time in seconds.
    double value() const;

    /// Returns elapsed time in seconds.
    double time_sec() const;

    /// Returns elapsed time in milliseconds.
    double time_msec() const;

    /// Returns elapsed time in milliseconds (VEX timer compatibility).
    double time() const { return time_msec(); }

    /// Returns elapsed time in specified units.
    double time(TimeUnits units) const {
        if (units == TimeUnits::Seconds) return time_sec();
        return time_msec();
    }


    /// Returns elapsed time in integer milliseconds.
    uint32_t time_ms() const;

private:
    uint64_t start_time_us;
};

inline void delay_ms(uint32_t ms) {
    Timer::delay_ms(ms);
}

inline void delay_us(uint64_t us) {
    Timer::delay_us(us);
}

} // namespace core

#include "core/utils/time.h"

namespace core {

static uint64_t default_clock_source() {
    static uint64_t simulated_us = 0;
    return (simulated_us += 10000); // fallback incremental 10ms
}

static void default_delay_provider(uint32_t ms) {
    // Busy wait fallback if no platform delay provider registered
    for (uint32_t i = 0; i < ms * 10000; ++i) {
        #if defined(__GNUC__) || defined(__clang__)
        asm volatile("" ::: "memory");
        #endif
    }
}

static Timer::ClockFn s_clock_source = default_clock_source;
static Timer::DelayFn s_delay_provider = default_delay_provider;

void Timer::set_clock_source(ClockFn fn) {
    if (fn) {
        s_clock_source = fn;
    }
}

void Timer::set_delay_provider(DelayFn fn) {
    if (fn) {
        s_delay_provider = fn;
    }
}

uint64_t Timer::systemHighResolution() {
    return s_clock_source();
}

uint64_t Timer::system() {
    return s_clock_source() / 1000;
}

double Timer::system_seconds() {
    return static_cast<double>(s_clock_source()) / 1000000.0;
}

void Timer::delay_ms(uint32_t ms) {
    s_delay_provider(ms);
}

void Timer::delay_us(uint64_t us) {
    s_delay_provider(static_cast<uint32_t>((us + 999) / 1000));
}

Timer::Timer() {
    reset();
}

void Timer::reset() {
    start_time_us = s_clock_source();
}

double Timer::value() const {
    return time_sec();
}

double Timer::time_sec() const {
    return static_cast<double>(s_clock_source() - start_time_us) / 1000000.0;
}

double Timer::time_msec() const {
    return static_cast<double>(s_clock_source() - start_time_us) / 1000.0;
}

uint32_t Timer::time_ms() const {
    return static_cast<uint32_t>(time_msec());
}

} // namespace core

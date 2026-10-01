#pragma once

#include "core/utils/time.h"
#include <atomic>

namespace mantle {

/**
 * Platform-independent spinlock mutex implementation.
 */
class Mutex {
private:
    std::atomic_flag flag = ATOMIC_FLAG_INIT;

public:
    Mutex() = default;
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    void lock() {
        while (flag.test_and_set(std::memory_order_acquire)) {
            core::delay_ms(1);
        }
    }

    void unlock() {
        flag.clear(std::memory_order_release);
    }

    bool try_lock() {
        return !flag.test_and_set(std::memory_order_acquire);
    }
};

using TaskFn = int (*)(void*);
using TaskSpawnerFn = void* (*)(TaskFn fn, void* arg);
using TaskStopperFn = void (*)(void* handle);

inline TaskSpawnerFn& get_task_spawner() {
    static TaskSpawnerFn s_spawner = nullptr;
    return s_spawner;
}

inline TaskStopperFn& get_task_stopper() {
    static TaskStopperFn s_stopper = nullptr;
    return s_stopper;
}

inline void set_task_spawner(TaskSpawnerFn spawner, TaskStopperFn stopper = nullptr) {
    get_task_spawner() = spawner;
    get_task_stopper() = stopper;
}

inline void* spawn_task(TaskFn fn, void* arg) {
    if (get_task_spawner()) {
        return get_task_spawner()(fn, arg);
    }
    return nullptr;
}

inline void stop_task(void* handle) {
    if (handle && get_task_stopper()) {
        get_task_stopper()(handle);
    }
}

} // namespace mantle

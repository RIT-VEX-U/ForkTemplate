#include "crust/mod.hpp"
#include "core/utils/time.h"
#include "mantle/rtos.h"
#include "mantle/commands/auto_command.h"
#include "mantle/initializer.h"
#include "mantle/subsystems/flywheel.h"
#include "vex.h"


namespace crust {

static uint64_t vex_clock_source() {
    return vex::timer::systemHighResolution();
}

static void vex_delay_provider(uint32_t ms) {
    vexDelay(ms);
}

static int crust_async_task(void *arg) {
    AutoCommand *cmd = static_cast<AutoCommand *>(arg);
    core::Timer tmr;
    while (1) {
        bool finished = cmd->run();
        if (finished) break;
        double t = static_cast<double>(tmr.time_ms()) / 1000.0;
        bool timed_out = t > cmd->timeout_seconds;
        bool doTimeout = timed_out && cmd->timeout_seconds > 0;
        if (cmd->true_to_end != nullptr) {
            doTimeout = doTimeout || cmd->true_to_end->test();
        }
        if (doTimeout) {
            cmd->on_timeout();
            break;
        }
        vexDelay(20);
    }
    delete cmd;
    return 0;
}

static void crust_async_spawner(AutoCommand *cmd) {
    new vex::task(crust_async_task, static_cast<void *>(cmd));
}

static size_t crust_timeout_runner(std::function<Selector::selector_t> selector, uint64_t microsec, unsigned int fallback, std::function<void(size_t)> cancel) {
    struct thread_args {
        std::function<Selector::selector_t> selector;
        size_t output;
    };
    uint64_t thread_start = vex::timer::systemHighResolution();
    thread_args targs = {selector, Selector::NO_SELECTION_INDEX};

    vex::thread sThread([](void* args) {
        ((thread_args*) args)->output = ((thread_args*) args)->selector();
    }, &targs);

    while(sThread.joinable()) {
        if(vex::timer::systemHighResolution() - thread_start > microsec) sThread.interrupt();
        else if(targs.output == Selector::NO_SELECTION_INDEX) {
            vexDelay(100);
            continue;
        }
        sThread.join();
    }

    if(targs.output == Selector::NO_SELECTION_INDEX) {
        if (cancel) cancel(fallback);
        return fallback;
    } else return targs.output;
}

static void* crust_task_spawner(int (*func)(void*), void* arg) {
    return new vex::task(func, arg);
}

static void crust_task_stopper(void* handle) {
    if (handle) {
        vex::task* t = static_cast<vex::task*>(handle);
        t->stop();
        delete t;
    }
}

void init() {
    core::Timer::set_clock_source(vex_clock_source);
    core::Timer::set_delay_provider(vex_delay_provider);
    mantle::set_task_spawner(crust_task_spawner, crust_task_stopper);
    Async::set_spawner(crust_async_spawner);
    Selector::set_timeout_runner(crust_timeout_runner);
    Flywheel::set_task_spawner(crust_task_spawner, crust_task_stopper);
}


const char* crust_version_string() {
    return "crust-v1.0.0";
}

} // namespace crust

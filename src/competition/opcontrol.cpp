#include "competition/opcontrol.h"
#include "robot-config.h"
#include <cstdarg>
#include <v5_api.h>
#include <stdarg.h>
// static constexpr std::uintptr_t JUMP_TABLE_START = 0x037fc000;
// static constexpr std::uintptr_t OFFSET_VSTRING = 0x00000684;
//
//
//
extern "C" {
int32_t vexBatteryVoltage() {
  // int32_t result;
  // __asm__ volatile (
  //     "ldr    r12, =0x037fca00  \n"
  //     "ldr    r12, [r12]        \n"
  //     "blx    r12               \n"
  //     "mov    %[out], r0        \n"
  //     : [out] "=r"(result)
  //     :
  //     : "r12", "r0", "lr"
  //   );
  // return result;
  return ((int32_t __attribute__((__pcs__("aapcs")))(*)(void))(*(uint32_t*)0x037fca00))();
}
}

extern "C" {
static inline void vexDisplay(int32_t line, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  ((void __attribute__((__pcs__("aapcs")))(*)(int32_t, const char*, va_list))(*(uint32_t*)0x037fc684))(line, fmt, args);

  // reinterpret_cast<void(*)(int32_t, const char*, va_list)>(0x037fc684)(line, fmt, args);
  va_end(args);

}
}

void opcontrol() {
  // vex::brain Brain;
  //
  int32_t voltage = vexBatteryVoltage();
  vexDisplayString(0, "%d", voltage);
  // vexDisplayString(1, "%d", vexBatteryVoltage());
  vexDisplay(5, "yay yay");

  while (true) {
    std::cout << vexBatteryVoltage() << std::endl;
    // printf("%d\n", vexBatteryVoltage());
    vexDelay(1000);
  }
}

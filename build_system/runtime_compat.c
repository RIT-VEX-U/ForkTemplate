/*
 * VEX startup normally replaces newlib's _impure_ptr with an OS-owned
 * struct _reent which is specific to VEX's newlib, and prevents certain features
 * like printf float from working.
 *
 * This change makes startup think that struct doesn't exist, preventing it from
 * replacing _impure_ptr, and allowing newlib to function properly.
 */
void* __wrap_vexSystemStdlibImpureDataAddr(void) { return 0; }

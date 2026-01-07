#ifndef STEPIT_MACRO_H_
#define STEPIT_MACRO_H_

namespace stepit {
extern const char *kConfigDir;
}  // namespace stepit

#ifdef STEPIT_FIX_GETTID
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)
#endif  // STEPIT_FIX_GETTID

#endif  // STEPIT_MACRO_H_

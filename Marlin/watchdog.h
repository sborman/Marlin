#ifndef _WATCHDOG_H_
#define _WATCHDOG_H_
#include "Marlin.h"

#ifdef USE_WATCHDOG

  // Intialize watch dog timer with a 1 second interrupt time
  void wd_init();

  // Pet the dog / reset watchdog.
  // This must be called at least every second after the first wd_init
  // or the microcontroller will go into emergency procedures.
  void wd_reset();

#else

  FORCE_INLINE void wd_init() {};
  FORCE_INLINE void wd_reset() {};

#endif

#endif // _WATCHDOG_H_

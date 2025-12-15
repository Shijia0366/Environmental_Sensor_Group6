#include <Arduino.h>
#include "config.h"

// Diagnostic mode
#if DIAG_MODE
void diag_setup();
void diag_loop();
void setup(){ diag_setup(); }
void loop() { diag_loop();  }

// Normal application mode
#else
void app_setup();
void app_loop();
void setup(){ app_setup(); }
void loop() { app_loop();  }
#endif

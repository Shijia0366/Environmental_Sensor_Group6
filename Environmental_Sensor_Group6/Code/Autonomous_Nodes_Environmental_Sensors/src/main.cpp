#include <Arduino.h>
#include "config.h"

// Diagnostic Mode
#if DIAG_MODE
void diag_setup();
void diag_loop();
void setup(){ diag_setup(); }
void loop() { diag_loop();  }

// Normal Application Mode
#else
void app_setup();
void app_loop();
void setup(){ app_setup(); }
void loop() { app_loop();  }
#endif
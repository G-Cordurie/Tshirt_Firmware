#ifndef AUTO_DIAG_H
#define AUTO_DIAG_H

#include <stdbool.h>

bool auto_diag_enter_check(void);
void auto_diag_run(void);
void auto_diag_set_status(uint8_t status);
void auto_diag_system_reset(void);

#endif
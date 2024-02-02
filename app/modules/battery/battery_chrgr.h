#ifndef BATTERY_CHRGR
#define BATTERY_CHRGR

#include <stdbool.h>

typedef enum chrgr_state_tag
{
    CHRGR_UNKNOWN_STATE = 0,
    CHRGR_DISCONNECTED_STATE,
    CHRGR_CONNECTED_STATE,
} chrgr_state_t;

typedef enum battery_chrg_state_tag
{
    CHRG_IDLE_STATE,
    CHRG_WAIT_STATE,
    CHRG_ONGOING_STATE,
    CHRG_COMPLETE_STATE,
} battery_chrg_state_t;

void          battery_chrgr_init(void);
void          battery_chrgr_enable(void);
chrgr_state_t battery_chrgr_state_get(void);
bool          battery_is_charging(void);
void          battery_chrgr_task(void);

#endif // BATTERY_CHRGR
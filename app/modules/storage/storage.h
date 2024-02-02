#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <string.h>

#include "ble_diagw.h"
#include "data_acq.h"
#include "flash_drv.h"
#include "md.pb.h"
#include "timer.h"

typedef enum mem_percent_mode_tag
{
    MEM_PERCENT_MODE_IDLE,
    MEM_PERCENT_MODE_FAST,
    MEM_PERCENT_MODE_SLOW,
} mem_percent_mode_t;

void storage_init(void);
void storage_percent_tmr_start(mem_percent_mode_t mode);
void storage_percent_tmr_stop(void);

void ring_store(notif_data_type_t type, uint8_t *data, uint8_t len);
void raw_data_ble_send(void);
void storage_mem_task(void);
void mem_dump_on_ble_cmd_callback(uint8_t *buf, uint16_t len);
void mem_dump_on_ble_disconnect_callback(ble_diagw_t *p_diagw);
void mem_dump_on_conn_param_update_callback(uint16_t conn_interval);
void mem_dump_on_ble_cmd_ack_callback(void);
bool mem_dump_in_progress(void);

#endif // STORAGE_H

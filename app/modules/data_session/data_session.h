#ifndef DATA_SESSION_H
#define DATA_SESSION_H

#include <stdint.h>

#include "flash_drv.h"

#define USER_ID_MAGIC_VAL ((uint16_t)0xA5A5)
#define USER_ID_LEN       6U

typedef enum data_idx_type_tag
{
    ecg_idx_type,
    imp_idx_type,
    breath_idx_type,
    temp_idx_type,
    acc_idx_type,
    data_idx_type_nbr,
} data_idx_type_t;

typedef struct data_session_tag
{
    uint32_t           crc;
    uint64_t           timestamp;
    uint64_t           user_id;
    uint32_t           data_idx[data_idx_type_nbr];
    flash_region_ctx_t data_region_ctx;
} data_session_t;

void     data_session_init(void);
void     data_session_update(uint64_t user_id, uint64_t timestamp);
void     data_session_store(void);
void     data_session_reset(void);
void     data_session_user_id_reset(void);
bool     data_session_user_is_connected(void);
void     data_session_user_disconnect(void);
void     data_session_get(const data_session_t *const data_session);
uint32_t data_session_sample_index_get(const data_idx_type_t type);
void     data_session_sample_index_set(const data_idx_type_t type, const uint32_t idx);
uint64_t data_session_sample_timestamp_get(const data_idx_type_t type);
void     data_session_timeout_tmr_start(void);
void     data_session_timeout_tmr_stop(void);

#endif // DATA_SESSION_H
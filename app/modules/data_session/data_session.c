#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "crc32.h"

#include "acc.h"
#include "app_ble_gap.h"
#include "breath.h"
#include "data_session.h"
#include "ecg.h"
#include "imp.h"
#include "pwr_mngr.h"
#include "spi_flash.h"
#include "storage.h"
#include "temp.h"
#include "timer.h"
#include "utils.h"

APP_TIMER_DEF(m_data_session_timeout_tmr_id);

#define DATA_SESSION_TIMEOUT_TMR_INTERVAL APP_TIMER_TICKS(30 * 1000)
#define DS_TIMESTAMP_OFFSET               (offsetof(data_session_t, timestamp))

static volatile data_session_t m_data_session;
static volatile bool           m_user_connected = false;

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static const uint32_t m_data_sampling_period_ms[] = {
    [ecg_idx_type] = ECG_SAMPLING_PERIOD_MS,   [imp_idx_type] = IMP_SAMPLING_PERIOD_MS, [breath_idx_type] = BREATH_SAMPLING_PERIOD_MS,
    [temp_idx_type] = TEMP_SAMPLING_PERIOD_MS, [acc_idx_type] = ACC_SAMPLING_PERIOD_MS,
};

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static void data_session_flash_load(data_session_t *const data_session)
{
    if (data_session == NULL)
    {
        return;
    }

    data_session_t tmp_data_session;
    memset((uint8_t *)&tmp_data_session, 0, sizeof(data_session_t));

    if (flash_page_read(MIN_DATA_SESSION_ADDR, (uint8_t *)&tmp_data_session, sizeof(data_session_t)) == NRF_SUCCESS)
    {
        uint32_t crc =
            crc32_compute((uint8_t *)&tmp_data_session + DS_TIMESTAMP_OFFSET, sizeof(data_session_t) - DS_TIMESTAMP_OFFSET, NULL);

        if (crc == tmp_data_session.crc)
        {
            memcpy((uint8_t *)data_session, (uint8_t *)&tmp_data_session, sizeof(data_session_t));
#if DEBUG
            INFO("Data Session loaded:");
            uint8_t buffer[21];
            uint64_to_string(data_session->timestamp, buffer, sizeof(buffer));
            INFO("\t\t Timestamp: %s", buffer);
            uint64_t user_id = (data_session->user_id >> 16) & 0xFFFFFFFFFFFF;
            INFO("\t\t User ID: %s", (uint8_t *)&user_id);
            INFO("\t\t Ext mem. addr: rd = %u (0x%X), wr = %u (0x%X)", data_session->data_region_ctx.rd_page_addr,
                 data_session->data_region_ctx.rd_page_addr, data_session->data_region_ctx.wr_page_addr,
                 data_session->data_region_ctx.wr_page_addr);
            INFO("\t\t Data index:");
            INFO("\t\t\t ECG idx: %u", data_session->data_idx[ecg_idx_type]);
            INFO("\t\t\t IMP idx: %u", data_session->data_idx[imp_idx_type]);
            INFO("\t\t\t BREATH idx: %u", data_session->data_idx[breath_idx_type]);
            INFO("\t\t\t TEMP idx: %u", data_session->data_idx[temp_idx_type]);
            INFO("\t\t\t ACC idx: %u", data_session->data_idx[acc_idx_type]);
#endif
            return;
        }

        INFO("[DATA_SESSION] > Function: %s, Invalid data session (CRC error) !", (uint32_t) __func__);
        return;
    }

    INFO("[%s], Flash access failed!", (uint32_t) __func__);
}

static void data_session_flash_store(data_session_t *const data_session)
{
    if (data_session == NULL)
    {
        return;
    }

    data_session->crc = crc32_compute((uint8_t *)data_session + DS_TIMESTAMP_OFFSET, sizeof(data_session_t) - DS_TIMESTAMP_OFFSET, NULL);

    if (flash_erase_block(MIN_DATA_SESSION_ADDR) == NRF_SUCCESS)
    {
        if (flash_page_write(MIN_DATA_SESSION_ADDR, (uint8_t *)data_session, sizeof(data_session_t)) == NRF_SUCCESS)
        {
#if DEBUG
            INFO("Data Session stored:");
            uint8_t buffer[21];
            uint64_to_string(data_session->timestamp, buffer, sizeof(buffer));
            INFO("\t\t Timestamp: %s", buffer);
            uint64_t user_id = (data_session->user_id >> 16) & 0xFFFFFFFFFFFF;
            INFO("\t\t User ID: %s", (uint8_t *)&user_id);
            INFO("\t\t Ext mem. addr: rd = %u (0x%X), wr = %u (0x%X)", data_session->data_region_ctx.rd_page_addr,
                 data_session->data_region_ctx.rd_page_addr, data_session->data_region_ctx.wr_page_addr,
                 data_session->data_region_ctx.wr_page_addr);
            INFO("\t\t Data index:");
            INFO("\t\t\t ECG idx: %u", data_session->data_idx[ecg_idx_type]);
            INFO("\t\t\t IMP idx: %u", data_session->data_idx[imp_idx_type]);
            INFO("\t\t\t BREATH idx: %u", data_session->data_idx[breath_idx_type]);
            INFO("\t\t\t TEMP idx: %u", data_session->data_idx[temp_idx_type]);
            INFO("\t\t\t ACC idx: %u", data_session->data_idx[acc_idx_type]);
#endif
            return;
        }
    }

    INFO("[%s], Flash access failed!", (uint32_t) __func__);
}

static void data_session_load(void)
{
    CRITICAL_REGION_ENTER();
    {
        data_session_flash_load((data_session_t *const)&m_data_session);
        flash_data_region_ctx_set(m_data_session.data_region_ctx);
    }
    CRITICAL_REGION_EXIT();
}

bool data_session_user_is_connected(void)
{
    return m_user_connected;
}

void data_session_user_disconnect(void)
{
    CRITICAL_REGION_ENTER();
    {
        if (m_user_connected)
        {
            m_user_connected = false;
        }
    }
    CRITICAL_REGION_EXIT();
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

void data_session_update(uint64_t user_id, uint64_t timestamp)
{
    if (timestamp && (timestamp > m_data_session.timestamp))
    {
        if (((uint16_t)m_data_session.user_id != USER_ID_MAGIC_VAL) || (((m_data_session.user_id >> 16) & 0xFFFFFFFFFFFF) == user_id))
        {
            CRITICAL_REGION_ENTER();
            {
                m_data_session.timestamp = timestamp;
                memset((uint8_t *)m_data_session.data_idx, 0, sizeof(m_data_session.data_idx));

                m_data_session.user_id = user_id << 16 | USER_ID_MAGIC_VAL;
                m_user_connected       = true;
            }
            CRITICAL_REGION_EXIT();

            data_session_timeout_tmr_stop();

#if DEBUG
            uint8_t buffer[21]; // 20 digits for the number + 1 for null-terminator
            uint64_to_string(m_data_session.timestamp, buffer, sizeof(buffer));
            INFO("[Data session] > Timestamp updated: %s", buffer);
            INFO("[Data session] > User ID: %s", (uint8_t *)&user_id);
#endif
        }
        else
        {
#if DEBUG
            uint64_t current_user_id = (m_data_session.user_id >> 16) & 0xFFFFFFFFFFFF;
            INFO("[Data session] > Unauthorized user ID: %s (!= %s) disconnecting...", (uint8_t *)&user_id, (uint8_t *)&current_user_id);
#endif
        }
    }
}

void data_session_get(const data_session_t *const data_session)
{
    CRITICAL_REGION_ENTER();
    {
        memcpy((uint8_t *)data_session, (uint8_t *)&m_data_session, sizeof(data_session_t));
    }
    CRITICAL_REGION_EXIT();
}

void data_session_store(void)
{
    CRITICAL_REGION_ENTER();
    {
        flash_data_region_ctx_get((flash_region_ctx_t *const)&m_data_session.data_region_ctx);
        data_session_flash_store((data_session_t *const)&m_data_session);
    }
    CRITICAL_REGION_EXIT();
}

void data_session_reset(void)
{
    CRITICAL_REGION_ENTER();
    {
        memset((uint8_t *)&m_data_session, 0, sizeof(data_session_t));
        m_user_connected = false;
        flash_data_region_ctx_reset();
        flash_data_region_ctx_get((flash_region_ctx_t *const)&m_data_session.data_region_ctx);
    }
    CRITICAL_REGION_EXIT();
    INFO("[DS] > Data session reset done.");
}

void data_session_user_id_reset(void)
{
    CRITICAL_REGION_ENTER();
    {
        m_data_session.user_id = 0;
        m_user_connected       = false;
    }
    CRITICAL_REGION_EXIT();
    INFO("[DS] > User ID reset done.");
}

uint32_t data_session_sample_index_get(const data_idx_type_t type)
{
    if ((type >= 0) && (type < data_idx_type_nbr))
    {
        return m_data_session.data_idx[type];
    }

    INFO("[%s] Invalid data type (%u) !", (uint32_t) __func__, type);

    return 0;
}

void data_session_sample_index_set(const data_idx_type_t type, const uint32_t idx)
{
    if ((type >= 0) && (type < data_idx_type_nbr))
    {
        m_data_session.data_idx[type] = idx;
        return;
    }

    INFO("[%s] Invalid data type (%u) !", (uint32_t) __func__, type);
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

uint64_t data_session_sample_timestamp_get(const data_idx_type_t type)
{
    uint64_t timestamp = 0;

    if ((type >= 0) && (type < data_idx_type_nbr))
    {
        CRITICAL_REGION_ENTER();
        {
            uint32_t idx                  = m_data_session.data_idx[type];
            timestamp                     = m_data_session.timestamp + (idx * m_data_sampling_period_ms[type]);
            m_data_session.data_idx[type] = ++idx;
        }
        CRITICAL_REGION_EXIT();
    }

    return timestamp;
}

/*--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------*/

static void data_session_timeout_tmr_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    ble_disconnect();
}

void data_session_init(void)
{
    memset((uint8_t *)&m_data_session, 0x00, sizeof(m_data_session));
    flash_data_region_ctx_get((flash_region_ctx_t *const)&m_data_session.data_region_ctx);
    data_session_load();
    ret_code_t err_code = app_timer_create(&m_data_session_timeout_tmr_id, APP_TIMER_MODE_SINGLE_SHOT, data_session_timeout_tmr_handler);
    APP_ERROR_CHECK(err_code);
}

void data_session_timeout_tmr_start(void)
{
    ret_code_t err_code = app_timer_start(m_data_session_timeout_tmr_id, DATA_SESSION_TIMEOUT_TMR_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

void data_session_timeout_tmr_stop(void)
{
    ret_code_t err_code = app_timer_stop(m_data_session_timeout_tmr_id);
    APP_ERROR_CHECK(err_code);
}
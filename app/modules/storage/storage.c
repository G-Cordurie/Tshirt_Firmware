#include "nrf_delay.h"
#include "nrf_soc.h"

#include "acc.h"
#include "adc.h"
#include "app_ble_gap.h"
#include "app_ble_mngr.h"
#include "battery.h"
#include "data_acq.h"
#include "data_session.h"
#include "flash_drv.h"
#include "md.pb.h"
#include "pb.h"
#include "pb_common.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "pwr_mngr.h"
#include "storage.h"
#include "timer.h"
#include "utils.h"

APP_TIMER_DEF(m_md_tmr_id);
APP_TIMER_DEF(m_md_status_check_tmr_id);
APP_TIMER_DEF(m_storage_percent_tmr_id);

#define BLE_DEFAULT_CONN_INTERVAL_MS         15U
#define TMR_BLE_NOTIF_INTERVAL_MS            BLE_DEFAULT_CONN_INTERVAL_MS
#define TMR_STATUS_CHECK_TIMEOUT_MS          (1U * 60000U)
#define TMR_CMD_BLE_SEND_TIMEOUT_MS          (5U * BLE_DEFAULT_CONN_INTERVAL_MS)
#define TMR_STORAGE_PERCENT_FAST_INTERVAL_MS (1000U)
#define TMR_STORAGE_PERCENT_FAST_TIMEOUT_S   (10U)
#define TMR_STORAGE_PERCENT_SLOW_INTERVAL_MS (30U * 1000U)

#define OS_BUF_SIZE                          (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OVERHEAD_LENGTH)
#define RAW_DATA_SIZE                        ((uint16_t)(sizeof(m_md_ctx.raw_data_buf) - (sizeof(m_md_ctx.raw_data_buf) % MEDIC_DATA_SLOT)))
#define DATA_CHUNK_ITEMS_COUNT               9U

#define MD_MIN_BLE_DATA_LEN                  (2 * MD_RAW_DATA_ITEM_SIZE) // Min required BLE data size = 48

#define MAX_ACQ_DATA                         128U
#define RING_SIZE                            5U
#define DATA_TYPE_LEN                        1U
#define MEDIC_DATA_SLOT                      ((uint8_t)(BLE_DIAGW_MAX_RAW_DATA_LEN + DATA_TYPE_LEN)) // MAX ECG DATA SIZE + 1 (DATA TYPE)

#define INCR_OFFSET(offset)                                                                                                                \
    do                                                                                                                                     \
    {                                                                                                                                      \
        if (++offset == MAX_ACQ_DATA)                                                                                                      \
        {                                                                                                                                  \
            offset = 0;                                                                                                                    \
        }                                                                                                                                  \
    } while (0)

typedef struct md_ctx_tag
{
    ret_code_t               ble_result;
    uint16_t                 current_pos;
    uint8_t                  raw_data_buf[FLASH_MAX_DATA_SIZE];
    uint8_t                  ostream_buf[OS_BUF_SIZE];
    volatile uint16_t        ble_max_data_len;
    volatile bool            ble_cmd_ack;
    pb_ostream_t             ostream;
    md_data_chunk_t          data_chunk;
    md_state_t               md_state;
    volatile md_ctrlpt_cmd_t md_ctrlpt_cmd;
    nrf_mutex_t              ble_cmd_lock;
    volatile bool            ble_notif_send_timeout;
    volatile uint16_t        ble_notif_interval_ms;
    volatile bool            md_status_check_timeout;
} md_ctx_t;

static volatile mem_percent_mode_t m_mem_percent_mode;
static volatile bool               m_mem_percent_tmr_timeout = false;
static volatile uint8_t            m_mem_percent_elapsed_time_s;

static md_ctx_t m_md_ctx = {
    .ble_result              = NRF_SUCCESS,
    .current_pos             = 0,
    .ble_max_data_len        = BLE_GATT_ATT_MTU_DEFAULT - OVERHEAD_LENGTH,
    .ble_cmd_ack             = false,
    .ostream                 = PB_OSTREAM_SIZING,
    .data_chunk              = MD_DATA_CHUNK_INIT_ZERO,
    .md_state                = MD_STATE_IDLE,
    .md_ctrlpt_cmd           = MD_CTRLPT_CMD_INIT_ZERO,
    .ble_notif_send_timeout  = false,
    .ble_notif_interval_ms   = BLE_DEFAULT_CONN_INTERVAL_MS,
    .md_status_check_timeout = false,
};

typedef struct acq_tag
{
    volatile uint8_t data[RING_SIZE][MAX_ACQ_DATA][BLE_DIAGW_MAX_RAW_DATA_LEN];
    volatile uint8_t rd_off[RING_SIZE];
    volatile uint8_t wr_off[RING_SIZE];
} acq_ctx_t;

static acq_ctx_t acq_ctx;

static const notif_data_type_t data_type_by_idx[RING_SIZE] = {
    [0] = ECG_TYPE, [1] = BREATH_TYPE, [2] = TEMP_TYPE, [3] = IMP_TYPE, [4] = ACC_TYPE};

static inline bool raw_data_ble_notif_enabled(void)
{
    return is_ble_connected() && (is_cccd_configured(ECG_TYPE) && is_cccd_configured(BREATH_TYPE) && is_cccd_configured(TEMP_TYPE) &&
                                  is_cccd_configured(IMP_TYPE) && is_cccd_configured(ACC_TYPE));
}

static ret_code_t flash_mem_percent_ble_send(void)
{
    uint8_t ostream_buf[MD_CTRLPT_CMD_SIZE] = {0};

    md_ctrlpt_cmd_t md_ctrlpt_cmd                 = MD_CTRLPT_CMD_INIT_ZERO;
    md_ctrlpt_cmd.which_cmd                       = MD_CTRLPT_CMD_MEM_PERCENT_TAG;
    md_ctrlpt_cmd.cmd.mem_percent.has_cmd_id      = true;
    md_ctrlpt_cmd.cmd.mem_percent.cmd_id          = MD_CMD_ID_MEM_PERCENT;
    md_ctrlpt_cmd.cmd.mem_percent.has_mem_percent = true;

    // Memory percent estimation on 5 Hours ≈ 4 * memory percent(20 Hours)
    uint32_t mem_percent                      = 4 * flash_data_region_percent();
    md_ctrlpt_cmd.cmd.mem_percent.mem_percent = (mem_percent >= 100) ? 0 : (100 - mem_percent);

    pb_ostream_t stream = pb_ostream_from_buffer(ostream_buf, MD_CTRLPT_CMD_SIZE);
    if (pb_encode(&stream, md_ctrlpt_cmd_fields, &md_ctrlpt_cmd))
    {
        return ble_diagw_mem_dump_ctrpt_indic_send(ostream_buf, stream.bytes_written);
    }

    return NRF_ERROR_INVALID_STATE;
}

static void raw_data_store(uint8_t type, uint8_t *buff, uint16_t size)
{
    static uint8_t  flash_buf[FLASH_MAX_DATA_SIZE];
    static uint16_t idx = 0;

    if (type <= DATA_TYPE_MIN || type >= DATA_TYPE_MAX)
    {
        return;
    }

    if ((size + DATA_TYPE_LEN) > MEDIC_DATA_SLOT) // size + type, can't be bigger than MEDIC_DATA_SLOT
    {
        INFO("[%s] size too large for data slot (%d)", __func__, size);
        return;
    }

    if (idx == 0)
    {
        memset(flash_buf, 0, sizeof(flash_buf));
    }

    flash_buf[idx] = type;
    memmove(&flash_buf[idx + 1], buff, size);

    idx += MEDIC_DATA_SLOT;

    // Wait for a full buffer before to write on flash memory.
    // MEDIC_DATA_SLOT is not necessarily a multiple of FLASH_MAX_DATA_SIZE so we have to substract the rest of FLASH_MAX_DATA_SIZE %
    // MEDIC_DATA_SLOT to get the right amount of data to store in the current page
    if (idx >= ((uint16_t)(sizeof(flash_buf) - (sizeof(flash_buf) % MEDIC_DATA_SLOT))))
    {
        idx = 0;
        UNUSED_RETURN_VALUE(flash_data_write(flash_buf, sizeof(flash_buf)));
    }
}

void ring_store(notif_data_type_t type, uint8_t *data, uint8_t len)
{
    if (len > 0 && len <= notif_max_len[type])
    {
        int8_t idx = -1;

        for (int8_t i = 0; i < (int8_t)RING_SIZE; i++)
        {
            if (data_type_by_idx[i] == type)
            {
                idx = i;
                break;
            }
        }

        if (idx >= 0 && idx < (int8_t)RING_SIZE)
        {
            memset((uint8_t *)acq_ctx.data[idx][acq_ctx.wr_off[idx]], 0, notif_max_len[type]);
            memcpy((uint8_t *)acq_ctx.data[idx][acq_ctx.wr_off[idx]], data, len);

            if (((acq_ctx.wr_off[idx] + 1) % MAX_ACQ_DATA) == acq_ctx.rd_off[idx])
            {
                INCR_OFFSET(acq_ctx.rd_off[idx]);
            }

            INCR_OFFSET(acq_ctx.wr_off[idx]);
        }
        else
        {
            INFO("[%s] Unexpected data index %d for type %s !", __func__, idx, data_type_name(type));
        }
    }
    else
    {
        INFO("[%s] Unexpected data size %u for type %s !", __func__, len, data_type_name(type));
    }
}

/**
 * @brief Read data stored in a ring buffer from the ADC callback and store it to flash memory or send it over BLE
 *
 */
void raw_data_ble_send(void)
{
    for (uint8_t i = 0; i < RING_SIZE; i++)
    {
        uint8_t rd_off;
        uint8_t wr_off;

        CRITICAL_REGION_ENTER();
        {
            rd_off = acq_ctx.rd_off[i];
            wr_off = acq_ctx.wr_off[i];
        }
        CRITICAL_REGION_EXIT();

        if (rd_off == wr_off)
        {
            continue;
        }

        notif_data_type_t type = data_type_by_idx[i];
        uint16_t          len  = notif_max_len[type];
        uint8_t          *data = (uint8_t *)acq_ctx.data[i][rd_off];

        if (raw_data_ble_notif_enabled())
        {
            if (ble_diagw_data_notif_send(type, data, len) != NRF_SUCCESS)
            {
                raw_data_store(type, data, len);
            }
        }
        else
        {
            raw_data_store(type, data, len);
        }

        CRITICAL_REGION_ENTER();
        {
            INCR_OFFSET(acq_ctx.rd_off[i]);
        }
        CRITICAL_REGION_EXIT();
    }
}

static inline bool mem_dump_ble_configured(void)
{
    return (is_ble_connected() && ((m_md_ctx.ble_max_data_len = ble_diagw_max_data_len_get()) >= MD_MIN_BLE_DATA_LEN) &&
            (is_cccd_configured(MEM_DUMP_TYPE)));
}

static ret_code_t flash_mem_dump(void)
{
    static uint32_t chunk_seq_nbr = 0;
    UNUSED_VARIABLE(chunk_seq_nbr);

    if (m_md_ctx.ble_result == NRF_SUCCESS)
    {
        if (m_md_ctx.current_pos == 0)
        {
            ret_code_t err_code =
                flash_data_read((uint8_t *const)m_md_ctx.raw_data_buf, sizeof(m_md_ctx.raw_data_buf), &m_md_ctx.data_chunk.block_addr);

            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }

            m_md_ctx.data_chunk.has_block_addr = true;
            m_md_ctx.data_chunk.data_count     = 0;
        }

        while (m_md_ctx.current_pos < RAW_DATA_SIZE)
        {
            if (!mem_dump_ble_configured())
            {
                return NRF_ERROR_INVALID_STATE;
            }

            data_type_t data_type = (data_type_t)m_md_ctx.raw_data_buf[m_md_ctx.current_pos];

            if (data_type > DATA_TYPE_MIN && data_type < DATA_TYPE_MAX)
            {
                uint16_t data_len           = notif_max_len[data_type];
                uint16_t residual_data_size = RAW_DATA_SIZE - (m_md_ctx.current_pos + 1);

                if ((data_len <= (MEDIC_DATA_SLOT - DATA_TYPE_LEN)) && (data_len <= residual_data_size))
                {
                    m_md_ctx.data_chunk.data[m_md_ctx.data_chunk.data_count].has_type = true;
                    m_md_ctx.data_chunk.data[m_md_ctx.data_chunk.data_count].type     = data_type;

                    uint64_t timestamp;
                    memcpy((uint8_t *)&timestamp, &m_md_ctx.raw_data_buf[m_md_ctx.current_pos + DATA_TYPE_LEN], sizeof(timestamp));
                    m_md_ctx.data_chunk.data[m_md_ctx.data_chunk.data_count].has_timestamp = true;
                    m_md_ctx.data_chunk.data[m_md_ctx.data_chunk.data_count].timestamp     = timestamp;

                    m_md_ctx.data_chunk.data[m_md_ctx.data_chunk.data_count].has_payload  = true;
                    m_md_ctx.data_chunk.data[m_md_ctx.data_chunk.data_count].payload.size = data_len - sizeof(timestamp);
                    memcpy(m_md_ctx.data_chunk.data[m_md_ctx.data_chunk.data_count].payload.bytes,
                           &m_md_ctx.raw_data_buf[m_md_ctx.current_pos + DATA_TYPE_LEN + TIMESTAMP_LEN], (data_len - sizeof(timestamp)));

                    m_md_ctx.data_chunk.data_count++;
                }
                else
                {
                    INFO("[%s] No enough data in raw data buffer or Invalid data length!", (uint32_t) __func__);
                }
            }
            else
            {
                INFO("[%s] Unknown raw data type (%d), skip data!", __func__, data_type);
            }

            bool         os_buf_overrun = false;
            pb_ostream_t stream         = PB_OSTREAM_SIZING;
            if (pb_encode(&stream, md_data_chunk_fields, &m_md_ctx.data_chunk))
            {
                if ((os_buf_overrun = stream.bytes_written > m_md_ctx.ble_max_data_len))
                {
                    if (m_md_ctx.data_chunk.data_count)
                    {
                        m_md_ctx.data_chunk.data_count--;
                    }

                    // INFO("Ouput stream overrun %u (max: %u)", stream.bytes_written, m_md_ctx.ble_max_data_len);
                }
            }
            else
            {
                INFO("[%s] Message size measuring error!", (uint32_t) __func__);
            }

            if ((m_md_ctx.data_chunk.data_count >= DATA_CHUNK_ITEMS_COUNT) || os_buf_overrun ||
                ((m_md_ctx.current_pos + MEDIC_DATA_SLOT) >= RAW_DATA_SIZE))
            {
                if (!os_buf_overrun)
                {
                    m_md_ctx.current_pos += MEDIC_DATA_SLOT;
                    if (m_md_ctx.current_pos >= RAW_DATA_SIZE)
                    {
                        m_md_ctx.current_pos = 0;
                    }
                }

                if (m_md_ctx.data_chunk.data_count)
                {
                    m_md_ctx.ostream               = pb_ostream_from_buffer(m_md_ctx.ostream_buf, m_md_ctx.ble_max_data_len);
                    bool enc_success               = pb_encode(&m_md_ctx.ostream, md_data_chunk_fields, &m_md_ctx.data_chunk);
                    m_md_ctx.data_chunk.data_count = 0;
                    if (enc_success)
                    {
                        chunk_seq_nbr = m_md_ctx.data_chunk.seq_number;
                        m_md_ctx.data_chunk.seq_number++;
                        break;
                    }
                    else
                    {
                        INFO("[%s] Encode error (Seq nbr: %u)!", (uint32_t) __func__, m_md_ctx.data_chunk.seq_number);
                    }
                }

                if (m_md_ctx.current_pos)
                {
                    continue;
                }

                return NRF_ERROR_INVALID_DATA;
            }

            m_md_ctx.current_pos += MEDIC_DATA_SLOT;
        }
    }

    m_md_ctx.ble_result = ble_diagw_mem_dump_notif_send(m_md_ctx.ostream_buf, m_md_ctx.ostream.bytes_written);

    if (m_md_ctx.ble_result == NRF_SUCCESS)
    {
        INFO("Data chunk sent (Block addr: %u, Chunk seq: %u, Size: %u)", m_md_ctx.data_chunk.block_addr, chunk_seq_nbr,
             m_md_ctx.ostream.bytes_written);
    }
    else
    {
        // INFO("[%s] Can not send data chunk (Block addr: %u, Chunk seq: %u) (BLE err: %d)!", (uint32_t) __func__,
        //      m_md_ctx.data_chunk.block_addr, chunk_seq_nbr, m_md_ctx.ble_result);
    }

    return m_md_ctx.ble_result;
}

void mem_dump_on_ble_cmd_callback(uint8_t *buf, uint16_t len)
{
    if (sd_mutex_acquire(&m_md_ctx.ble_cmd_lock) == NRF_SUCCESS)
    {
        md_ctrlpt_cmd_t md_ctrlpt_cmd = MD_CTRLPT_CMD_INIT_ZERO;
        pb_istream_t    stream        = pb_istream_from_buffer(buf, len);

        if (pb_decode(&stream, md_ctrlpt_cmd_fields, &md_ctrlpt_cmd))
        {
            switch (md_ctrlpt_cmd.which_cmd)
            {
            case MD_CTRLPT_CMD_START_TRANSFERT_TAG:
            {
                if (md_ctrlpt_cmd.cmd.start_transfert.cmd_id == MD_CMD_ID_START_TRANSFERT)
                {
                    m_md_ctx.md_ctrlpt_cmd = md_ctrlpt_cmd;
                }
            }
            break;

            case MD_CTRLPT_CMD_STOP_TRANSFERT_TAG:
            {
                if (md_ctrlpt_cmd.cmd.stop_transfert.cmd_id == MD_CMD_ID_STOP_TRANSFERT)
                {
                    m_md_ctx.md_ctrlpt_cmd = md_ctrlpt_cmd;
                }
            }
            break;

            case MD_CTRLPT_CMD_TRANSFERT_ERROR_TAG:
            {
                if (md_ctrlpt_cmd.cmd.transfert_error.cmd_id == MD_CMD_ID_TRANSFERT_ERROR)
                {
                    if (md_ctrlpt_cmd.cmd.transfert_error.err_type == MD_ERROR_TYPE_CHUNK_SEQ_NUMBER_ERR)
                    {
                        m_md_ctx.md_ctrlpt_cmd = md_ctrlpt_cmd;
                    }
                }
            }
            break;

            case MD_CTRLPT_CMD_INFO_REQ_TAG:
            {
                if (md_ctrlpt_cmd.cmd.info_req.cmd_id == MD_CMD_ID_INFO_REQUEST)
                {
                    m_md_ctx.md_ctrlpt_cmd = md_ctrlpt_cmd;
                }
            }
            break;

            default:
                break;
            }
        }
        sd_mutex_release(&m_md_ctx.ble_cmd_lock);
    }
}

static void mem_dump_notif_tmr_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    m_md_ctx.ble_notif_send_timeout = true;
}

static inline void mem_dump_notif_tmr_start(void)
{
    m_md_ctx.ble_notif_send_timeout = false;
    ret_code_t err_code             = app_timer_start(m_md_tmr_id, APP_TIMER_TICKS(m_md_ctx.ble_notif_interval_ms), NULL);
    APP_ERROR_CHECK(err_code);
}

static inline void mem_dump_notif_tmr_stop(void)
{
    ret_code_t err_code = app_timer_stop(m_md_tmr_id);
    APP_ERROR_CHECK(err_code);
}

static void mem_dump_status_check_tmr_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    m_md_ctx.md_status_check_timeout = true;
}

static inline void mem_dump_status_check_tmr_start(void)
{
    m_md_ctx.md_status_check_timeout = false;
    ret_code_t err_code              = app_timer_start(m_md_status_check_tmr_id, APP_TIMER_TICKS(TMR_STATUS_CHECK_TIMEOUT_MS), NULL);
    APP_ERROR_CHECK(err_code);
}

static inline void mem_dump_status_check_tmr_stop(void)
{
    ret_code_t err_code = app_timer_stop(m_md_status_check_tmr_id);
    APP_ERROR_CHECK(err_code);
}

static void storage_percent_tmr_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    m_mem_percent_tmr_timeout = true;

    if (m_mem_percent_mode == MEM_PERCENT_MODE_FAST)
    {
        if (++m_mem_percent_elapsed_time_s >= TMR_STORAGE_PERCENT_FAST_TIMEOUT_S)
        {
            storage_percent_tmr_stop();
            storage_percent_tmr_start(MEM_PERCENT_MODE_SLOW);
        }
    }
}

void storage_percent_tmr_start(mem_percent_mode_t mode)
{
    uint32_t timeout_ms = TMR_STORAGE_PERCENT_FAST_INTERVAL_MS;

    if (mode == MEM_PERCENT_MODE_FAST)
    {
        m_mem_percent_mode = MEM_PERCENT_MODE_FAST;
        timeout_ms         = TMR_STORAGE_PERCENT_FAST_INTERVAL_MS;
    }
    else if (mode == MEM_PERCENT_MODE_SLOW)
    {
        m_mem_percent_mode = MEM_PERCENT_MODE_SLOW;
        timeout_ms         = TMR_STORAGE_PERCENT_SLOW_INTERVAL_MS;
    }

    m_mem_percent_elapsed_time_s = 0;
    m_mem_percent_tmr_timeout    = false;
    ret_code_t err_code          = app_timer_start(m_storage_percent_tmr_id, APP_TIMER_TICKS(timeout_ms), NULL);
    APP_ERROR_CHECK(err_code);
}

void storage_percent_tmr_stop(void)
{
    m_mem_percent_mode        = MEM_PERCENT_MODE_IDLE;
    m_mem_percent_tmr_timeout = false;
    ret_code_t err_code       = app_timer_stop(m_storage_percent_tmr_id);
    APP_ERROR_CHECK(err_code);
}

static inline void mem_dump_ctx_init(void)
{
    m_md_ctx.data_chunk.has_cmd_id     = true;
    m_md_ctx.data_chunk.cmd_id         = MD_CMD_ID_DATA_CHUNK;
    m_md_ctx.data_chunk.has_seq_number = true;
    m_md_ctx.data_chunk.seq_number     = 0;
    m_md_ctx.data_chunk.data_count     = 0;

    m_md_ctx.current_pos      = 0;
    m_md_ctx.ble_result       = NRF_SUCCESS;
    m_md_ctx.ble_max_data_len = ble_diagw_max_data_len_get();

    mem_dump_notif_tmr_stop();
    m_md_ctx.ble_notif_send_timeout = false;
    mem_dump_status_check_tmr_stop();
    m_md_ctx.md_status_check_timeout = false;
}

static inline uint32_t flash_rd_page_addr_get(void)
{
    flash_region_ctx_t data_region_ctx;
    flash_data_region_ctx_get((flash_region_ctx_t *const)&data_region_ctx);
    return data_region_ctx.rd_page_addr;
}

static inline void flash_rd_page_addr_set(uint32_t rd_page_addr)
{
    flash_region_ctx_t data_region_ctx;
    flash_data_region_ctx_get((flash_region_ctx_t *const)&data_region_ctx);
    data_region_ctx.rd_page_addr = rd_page_addr;
    flash_data_region_ctx_set(data_region_ctx);
}

static inline void mem_dump_stop_transfert_cmd(void)
{
    mem_dump_notif_tmr_stop();
    m_md_ctx.ble_notif_send_timeout = false;
    mem_dump_status_check_tmr_stop();
    m_md_ctx.md_status_check_timeout = false;
}

static inline void mem_dump_transfert_err_cmd(void)
{
    if (m_md_ctx.md_ctrlpt_cmd.cmd.transfert_error.err_type == MD_ERROR_TYPE_CHUNK_SEQ_NUMBER_ERR)
    {
        CRITICAL_REGION_ENTER();
        {
            m_md_ctx.current_pos           = 0;
            m_md_ctx.ble_result            = NRF_SUCCESS;
            m_md_ctx.data_chunk.seq_number = m_md_ctx.md_ctrlpt_cmd.cmd.transfert_error.seq_number;
            flash_rd_page_addr_set(m_md_ctx.md_ctrlpt_cmd.cmd.transfert_error.block_addr);

            mem_dump_notif_tmr_stop();
            mem_dump_notif_tmr_start();
            mem_dump_status_check_tmr_stop();
            mem_dump_status_check_tmr_start();

            INFO("Mem. dump transfert error cmd rcvd! (Seq. number: %u, block addr: %u)", m_md_ctx.data_chunk.seq_number,
                 m_md_ctx.md_ctrlpt_cmd.cmd.transfert_error.block_addr);
        }
        CRITICAL_REGION_EXIT();
    }
}

// TODO: Blocking function! No side effect as we are in mem. dump state, but can be implemented as non blocking if needed.
static ret_code_t mem_dump_complete_cmd_ble_send(void)
{
    uint8_t ostream_buf[MD_CTRLPT_CMD_SIZE] = {0};

    md_ctrlpt_cmd_t md_ctrlpt_cmd                   = MD_CTRLPT_CMD_INIT_ZERO;
    md_ctrlpt_cmd.which_cmd                         = MD_CTRLPT_CMD_TRANSFERT_COMPLETE_TAG;
    md_ctrlpt_cmd.cmd.transfert_complete.has_cmd_id = true;
    md_ctrlpt_cmd.cmd.transfert_complete.cmd_id     = MD_CMD_ID_TRANSFERT_COMPLETE;

    pb_ostream_t stream = pb_ostream_from_buffer(ostream_buf, MD_CTRLPT_CMD_SIZE);
    if (pb_encode(&stream, md_ctrlpt_cmd_fields, &md_ctrlpt_cmd))
    {
        m_md_ctx.ble_cmd_ack = false;
        ret_code_t err_code  = ble_diagw_mem_dump_ctrpt_indic_send(ostream_buf, stream.bytes_written);

        if (err_code == NRF_SUCCESS)
        {
            volatile uint16_t elapsed_ms = 0;
            while (!m_md_ctx.ble_cmd_ack && elapsed_ms++ < (5 * m_md_ctx.ble_notif_interval_ms))
            {
                nrf_delay_ms(1);
            }

            if (m_md_ctx.ble_cmd_ack)
            {
                INFO("Mem. Dump complete command sent");
                return err_code;
            }

            INFO("[%s] Mem. Dump complete command confirmation timeout!", (uint32_t) __func__);
            return NRF_ERROR_TIMEOUT;
        }

        INFO("[%s] Mem. Dump complete command not sent!", (uint32_t) __func__);
        return err_code;
    }

    INFO("[%s] Command encode error!", (uint32_t) __func__);
    return NRF_ERROR_INVALID_DATA;
}

static ret_code_t mem_dump_info_resp_ble_send(void)
{
    uint8_t ostream_buf[MD_CTRLPT_CMD_SIZE] = {0};

    flash_region_ctx_t data_region_ctx;
    flash_data_region_ctx_get((flash_region_ctx_t *const)&data_region_ctx);

    md_ctrlpt_cmd_t md_ctrlpt_cmd                      = MD_CTRLPT_CMD_INIT_ZERO;
    md_ctrlpt_cmd.which_cmd                            = MD_CTRLPT_CMD_INFO_RESP_TAG;
    md_ctrlpt_cmd.cmd.info_resp.has_cmd_id             = true;
    md_ctrlpt_cmd.cmd.info_resp.cmd_id                 = MD_CMD_ID_INFO_RESPONSE;
    md_ctrlpt_cmd.cmd.info_resp.has_start_block_addr   = true;
    md_ctrlpt_cmd.cmd.info_resp.start_block_addr       = data_region_ctx.rd_page_addr;
    md_ctrlpt_cmd.cmd.info_resp.has_end_block_addr     = true;
    md_ctrlpt_cmd.cmd.info_resp.end_block_addr         = data_region_ctx.wr_page_addr - 1;
    md_ctrlpt_cmd.cmd.info_resp.has_total_block_nbr    = true;
    md_ctrlpt_cmd.cmd.info_resp.total_block_nbr        = flash_data_region_used_pages();
    md_ctrlpt_cmd.cmd.info_resp.has_mem_min_block_addr = true;
    md_ctrlpt_cmd.cmd.info_resp.mem_min_block_addr     = region_min_addr[MEDIC_REGION];
    md_ctrlpt_cmd.cmd.info_resp.has_mem_max_block_addr = true;
    md_ctrlpt_cmd.cmd.info_resp.mem_max_block_addr     = region_max_addr[MEDIC_REGION];

    pb_ostream_t stream = pb_ostream_from_buffer(ostream_buf, MD_CTRLPT_CMD_SIZE);
    if (pb_encode(&stream, md_ctrlpt_cmd_fields, &md_ctrlpt_cmd))
    {
        return ble_diagw_mem_dump_ctrpt_indic_send(ostream_buf, stream.bytes_written);
    }

    return NRF_ERROR_INVALID_STATE;
}

void storage_mem_task(void)
{
    if (m_md_ctx.md_state == MD_STATE_IDLE)
    {
        raw_data_ble_send();

        if (m_mem_percent_tmr_timeout)
        {
            m_mem_percent_tmr_timeout = false;
            flash_mem_percent_ble_send();
        }

        if (sd_mutex_acquire(&m_md_ctx.ble_cmd_lock) == NRF_SUCCESS)
        {
            if (m_md_ctx.md_ctrlpt_cmd.which_cmd == MD_CTRLPT_CMD_START_TRANSFERT_TAG)
            {
                medic_meas_stop();
                mem_dump_ctx_init();
                m_md_ctx.md_state = MD_STATE_PREPARE;
                pm_disallow_device_to_sleep();
                mem_dump_status_check_tmr_start();
                memset((uint8_t *)&m_md_ctx.md_ctrlpt_cmd, 0, sizeof(m_md_ctx.md_ctrlpt_cmd));
                battery_meas_start(); // Continue battery SOC monitoring during memory dumping
            }
            sd_mutex_release(&m_md_ctx.ble_cmd_lock);
        }
    }

    if (m_md_ctx.md_state == MD_STATE_PREPARE)
    {
        if (!m_md_ctx.md_status_check_timeout && mem_dump_ble_configured())
        {
            if (mem_dump_info_resp_ble_send() == NRF_SUCCESS)
            {
                INFO("[Storage] > Memory dump started...");
                mem_dump_status_check_tmr_stop();
                mem_dump_status_check_tmr_start();
                mem_dump_notif_tmr_start();
                m_md_ctx.md_state = MD_STATE_IN_PROGRESS;
            }
        }
        else
        {
            INFO("[Storage] > [MD_STATE_PREPARE] Memory dump aborted !");
            m_md_ctx.md_state = MD_STATE_COMPLETED;
        }
    }

    if (m_md_ctx.md_state == MD_STATE_IN_PROGRESS)
    {
        if (!m_md_ctx.md_status_check_timeout && mem_dump_ble_configured())
        {
            if (m_md_ctx.ble_notif_send_timeout)
            {
                ret_code_t err_code = flash_mem_dump();

                if (err_code == NRF_SUCCESS)
                {
                    mem_dump_status_check_tmr_stop();
                    mem_dump_status_check_tmr_start();
                }

                mem_dump_notif_tmr_start();

                if (err_code == NRF_ERROR_NO_MEM)
                {
                    if (mem_dump_complete_cmd_ble_send() == NRF_SUCCESS)
                    {
                        mem_dump_notif_tmr_stop();
                        m_md_ctx.ble_notif_send_timeout = false;
                        mem_dump_status_check_tmr_stop();
                        mem_dump_status_check_tmr_start();
                    }
                }
            }

            if (m_md_ctx.md_ctrlpt_cmd.which_cmd)
            {
                if (sd_mutex_acquire(&m_md_ctx.ble_cmd_lock) == NRF_SUCCESS)
                {
                    switch (m_md_ctx.md_ctrlpt_cmd.which_cmd)
                    {
                    case MD_CTRLPT_CMD_STOP_TRANSFERT_TAG:
                    {
                        INFO("[Storage] > Mem. dump stop transfert cmd rcvd");
                        mem_dump_stop_transfert_cmd();
                        m_md_ctx.md_state = MD_STATE_COMPLETED;
                    }
                    break;

                    case MD_CTRLPT_CMD_TRANSFERT_ERROR_TAG:
                    {
                        mem_dump_transfert_err_cmd();
                    }
                    break;

                    case MD_CTRLPT_CMD_INFO_REQ_TAG:
                    {
                        (void)mem_dump_info_resp_ble_send();
                    }
                    break;

                    default:
                        break;
                    }

                    memset((uint8_t *)&m_md_ctx.md_ctrlpt_cmd, 0, sizeof(m_md_ctx.md_ctrlpt_cmd));
                    sd_mutex_release(&m_md_ctx.ble_cmd_lock);
                }
            }
        }
        else
        {
            INFO("[Storage] > [MD_STATE_IN_PROGRESS] Memory dump aborted !");
            mem_dump_stop_transfert_cmd();
            m_md_ctx.md_state = MD_STATE_COMPLETED;
        }
    }

    if (m_md_ctx.md_state == MD_STATE_COMPLETED)
    {
        INFO("[Storage] > [MD_STATE_COMPLETED] Memory dump completed");
        flash_mem_percent_ble_send();
        pm_allow_device_to_sleep();
    }
}

void mem_dump_on_ble_disconnect_callback(ble_diagw_t *p_diagw)
{
    m_md_ctx.ble_max_data_len      = p_diagw->max_data_len;
    m_md_ctx.ble_notif_interval_ms = BLE_DEFAULT_CONN_INTERVAL_MS;
}

void mem_dump_on_conn_param_update_callback(uint16_t conn_interval_ms)
{
    m_md_ctx.ble_notif_interval_ms = conn_interval_ms;
}

void mem_dump_on_ble_cmd_ack_callback(void)
{
    // INFO("Confirmation on mem. dump ctrlpt rcvd");
    m_md_ctx.ble_cmd_ack = true;
}

bool mem_dump_in_progress(void)
{
    return m_md_ctx.md_state != MD_STATE_IDLE;
}

void storage_init(void)
{
    ret_code_t err_code;

    memset((uint8_t *)acq_ctx.rd_off, 0, sizeof(acq_ctx.rd_off));
    memset((uint8_t *)acq_ctx.wr_off, 0, sizeof(acq_ctx.wr_off));

    flash_init();

    err_code = sd_mutex_new(&m_md_ctx.ble_cmd_lock);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_md_tmr_id, APP_TIMER_MODE_SINGLE_SHOT, mem_dump_notif_tmr_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_md_status_check_tmr_id, APP_TIMER_MODE_SINGLE_SHOT, mem_dump_status_check_tmr_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_storage_percent_tmr_id, APP_TIMER_MODE_REPEATED, storage_percent_tmr_handler);
    APP_ERROR_CHECK(err_code);
}
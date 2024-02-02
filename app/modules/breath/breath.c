#if 0

#include <stdint.h>
#include <string.h>

#include "arm_math.h"

#include "nordic_common.h"
#include "nrf.h"

#include "data_acq.h"
#include "debug.h"
#include "timer.h"
#include "utils.h"

APP_TIMER_DEF(m_breath_auto_disc_tmr_id); // Autodisconnect Timer Instance

#define BREATH_AUTO_DISC_TMR_INTERVAL APP_TIMER_TICKS(5000)
#define BREATH_BUFFER_LEN             (20 * 32) // Buffer for autodisconnection

// Breathing signal can be quite low, but the variance is close to 0 when no one is wearing the tee-shirt. Fine tune if needed.
#define BREATH_DETECTION_THRESHOLD    16

static q15_t             breath_buffer[BREATH_BUFFER_LEN];
static volatile uint16_t m_breath_buffer_idx = 0;

// FIR coefficients for a low-pass filter with cut-off frequency of 1.8Hz and sampling rate of 20Hz, to get rid of noise in breathing data
// before computing its variance
static const q15_t BREATH_FILTER_COEFFS[] = {-105, -126, -89,  254,  1131, 2552, 4202, 5539, 6054,
                                             5539, 4202, 2552, 1131, 254,  -89,  -126, -105};

q15_t                             breath_filter_state[16 + 8]; // 16 coefficients, blocksize: 8
static const arm_fir_instance_q15 breath_filter = {16, breath_filter_state, (q15_t *)BREATH_FILTER_COEFFS};

bool       prefs_auto_disconnect        = true; // Prefs to auto disconnect or not
static int should_auto_disconnect_count = 0;    // TODO: not used and should be deleted!

/** Boolean to store if a device has connected during this session */
volatile bool device_was_connected = false;

/**@brief Store upper breath for 30 seconds
 *
 * @details Store ecg in a 30s array
 * to be use by auto disconnect
 */
void store_breath(int16_t adc_value)
{
    if (m_breath_buffer_idx >= BREATH_BUFFER_LEN)
    {
        // Wait for the timer to process previous buffer
        // This could lead to a small, unimportant data race
        return;
    }

    // Convert from [0, 4096] to Q1.15
    breath_buffer[m_breath_buffer_idx++] = (16 * adc_value) ^ 0x8000;
}

/**@brief Should we auto disconnect
 *
 * @details Return true if variance of the last 32 s of breath is almost zero
 */
static bool should_auto_disconnect()
{
    q15_t variance;

    if (m_breath_buffer_idx < BREATH_BUFFER_LEN)
    {
        // Not enough samples
        return false;
    }

    // Filter out high-frequency noise
    for (int idx = 0; idx < BREATH_BUFFER_LEN; idx += 8)
    {
        arm_fir_fast_q15(&breath_filter, &breath_buffer[idx], &breath_buffer[idx], 8);
    }

    arm_var_q15(breath_buffer, BREATH_BUFFER_LEN, &variance);

    /* DEBUG */
    /*
       uint8_t memory[20];
       memset(&memory, 0, sizeof(memory));
       memory[0] = 0x02;
       snprintf ((char *) &memory[1], 20, "B: %d %d", variance, device_was_connected);
       ble_diagw_data_notif_send(INFO_TYPE, &g_evt_ctx.m_diagw, memory, sizeof (memory));
       */
    /* DEBUG */

    // Reset the buffer to collect new samples
    m_breath_buffer_idx = 0;

    // Now if this variance is very very low we can deem no one
    // is breathing and ask for autodisconnection
    if (variance > BREATH_DETECTION_THRESHOLD)
    {
        return false;
    }

    return true;
}

/**@brief Function to read memory and resend stored data at regular interval.
 * This stop all other adc sampling while doing the measure
 */
static void breath_auto_disc_tmr_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    if ((prefs_auto_disconnect == false) && (is_ble_connected()))
    {
        return;
    }

    should_auto_disconnect_count++;

    if (device_was_connected == true)
    {
        // Do not autodisconnect if device has ever been connected
        // FIXME: when autodisconnection is stable we should delete this
        return;
    }

    if (should_auto_disconnect() == true)
    {
        // TODO: Auto disconnection should be reviewed since the hw filter has changed
        // pm_allow_device_to_sleep();
    }
}

/**
 * @brief Init array for auto disconnect with max amplitude
 *
 */
void init_auto_disconnect(void)
{
    m_breath_buffer_idx = 0;
    memset(breath_buffer, 0, sizeof(breath_buffer));
    memset(breath_filter_state, 0, sizeof(breath_filter_state));
}

void breath_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_create(&m_breath_auto_disc_tmr_id, APP_TIMER_MODE_REPEATED, breath_auto_disc_tmr_handler);
    APP_ERROR_CHECK(err_code);
}

void breath_tmr_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_breath_auto_disc_tmr_id, BREATH_AUTO_DISC_TMR_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    INFO("Breath auto-disc timer started");
}

void breath_tmr_stop(void)
{
    ret_code_t err_code;

    err_code = app_timer_stop(m_breath_auto_disc_tmr_id);
    APP_ERROR_CHECK(err_code);
    INFO("Breath auto-disc timer stopped");
}

void breath_on_ble_connect_callback(void)
{
    // Record that the device was connected. This will disable auto-shutdown
    device_was_connected = true;
}

void breath_on_ble_disconnect_callback(void)
{
    // Reset the movement detection on disconnect,  Unimportant race condition!
    m_breath_buffer_idx = 0;
}

#endif
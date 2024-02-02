#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "arm_math.h"

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "app_error.h"
#include "ble.h"
#include "ble_dfu.h"
#include "ble_srv_common.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_lesc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_mpu_lib.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_stack_guard.h"
#include "nrf_stack_info.h"

#include "acc.h"
#include "adg714.h"
#include "app_ble_gap.h"
#include "app_ble_mngr.h"
#include "app_ble_sm.h"
#include "app_config.h"
#include "auto_diag.h"
#include "battery.h"
#include "battery_chrgr.h"
#include "battery_soc.h"
#include "breath.h"
#include "clock.h"
#include "dac.h"
#include "data_acq.h"
#include "data_session.h"
#include "debug.h"
#include "ecg.h"
#include "imp.h"
#include "pwr_mngr.h"
#include "pwr_reg.h"
#include "rtc.h"
#include "spi_flash.h"
#include "storage.h"
#include "temp.h"
#include "timer.h"

#define DEAD_BEEF 0xDEADBEEF /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

static inline void stack_guard_init(void)
{
    APP_ERROR_CHECK(nrf_mpu_lib_init());
    APP_ERROR_CHECK(nrf_stack_guard_init());
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

static inline void bsp_init(void)
{
    led0_init();
    ecg_init();
    pwr_ldo_reg_disable();
    battery_switch_enable();
    battery_chrgr_init();
    dac_init();
    adg714_init();
    acc_spi_init();
    flash_spi_init();
}

static inline void app_modules_init(void)
{
    stack_guard_init();
    hfclk_clock_init();
    timer_init();
    rtc_init();
#ifdef DEBUG
    rtc_start();
#endif
    power_management_init();
    ble_stack_init();
    peer_manager_init();
    bsp_init();

    device_name_base36_set();
    storage_init();
    imp_stimulus_init();

#if APP_MODULE_ENABLED(AUTODIAG)
    if (auto_diag_enter_check())
    {
        auto_diag_run();
    }
#endif

    temp_calib_check();
    data_session_init();
    acc_init();
    ble_stack_config();
}

static void main_task(void)
{
    app_ble_task();
    storage_mem_task();
    battery_chrgr_task();
    data_acq_task();
    pwr_mngr_task();
}

/**
 * @brief Function for application main entry.
 *
 * @return int
 */
int main(void)
{
    log_init();
    INFO("Log Start !");

    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    ret_code_t err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);

    app_modules_init();

    if (battery_safe_boot_check() != NRF_SUCCESS)
    {
        system_shutdown(false);
        sleep_mode_enter();
    }

#if APP_MODULE_ENABLED(BLE_SEC)
    (void)advertising_start(false);
#else
    (void)advertising_start(true);
#endif

    INFO("HW %s init done (v%s)", HW_REV_STR, SW_REV_STR);

    // Enter main loop.
    for (;;)
    {
        main_task();
        idle_state_handle();
    }
}

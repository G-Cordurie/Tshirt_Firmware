#include <stdint.h>

#include "nrf_bootloader_info.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_power.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_saadc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_stack_guard.h"

#include "acc.h"
#include "adc.h"
#include "adg714.h"
#include "app_ble_gap.h"
#include "battery.h"
#include "battery_soc.h"
#include "boards.h"
#include "breath.h"
#include "clock.h"
#include "dac.h"
#include "data_session.h"
#include "debug.h"
#include "ecg.h"
#include "imp.h"
#include "pwr_mngr.h"
#include "pwr_reg.h"
#include "rtc.h"
#include "storage.h"

static volatile bool m_low_pwr_mode_flag = false; // Low power mode flag

static void pwr_mngr_sdh_state_observer(nrf_sdh_state_evt_t state, void *p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        // Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) = {
    .handler = pwr_mngr_sdh_state_observer,
};

static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
        INFO("Power management wants to reset to DFU mode.");
        break;

    case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
        INFO("[PWR_MNGR] > Go to system off...");
        return true;

    default:
        return true;
    }

    INFO("Power management allowed to reset to DFU mode.");
    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

void power_management_init(void)
{
    uint32_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void pm_allow_device_to_sleep(void)
{
    m_low_pwr_mode_flag = true;
}

void pm_disallow_device_to_sleep(void)
{
    m_low_pwr_mode_flag = false;
}

static bool pm_check_if_device_can_go_to_sleep(void)
{
    return m_low_pwr_mode_flag;
}

void system_shutdown(bool session_store)
{
    ble_disconnect();
    medic_meas_stop();
    battery_switch_disable();
    imp_meas_enable();

    if (session_store)
    {
        data_session_store();
    }

    battery_chrgr_enable();

    acc_spi_uninit();
    adg714_spi_uninit();
    dac_disconnect();
    flash_uninit();
    rtc_uninit();

    pwr_ldo_reg_disable();
    hfclk_clock_release();
    LED_OFF();
}

void sleep_mode_enter(void)
{
    ret_code_t err_code = nrf_stack_guard_deinit();
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_sense_input(INT2, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);

    // Disable SoftDevice. It is required to be able to write to GPREGRET2 register (SoftDevice API blocks it).
    // GPREGRET2 register holds the information about skipping CRC check on next boot.
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
}

void pwr_mngr_task(void)
{
    if (pm_check_if_device_can_go_to_sleep())
    {
        system_shutdown(true);
        sleep_mode_enter();
    }
}
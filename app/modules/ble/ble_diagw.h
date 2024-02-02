/**@file
 *
 * @defgroup ble_diagw ChronoLife DiagW Service
 * @{
 * @ingroup
 * @brief    ChronoLife DiagW Service implementation.
 *
 * * 713D0001-503E-4C75-BA94-3148F18D941E : ECG (200Hz : 20 Notifications / s)
 *   4 bytes : index of the notification
 *   10 values packed in 12 bits : 15 bytes
 * * 713D0002-503E-4C75-BA94-3148F18D941E : Breath (20Hz * 2 : 4 Notifications / s)
 *   4 bytes : index of the notification
 *   10 values packed in 12 bits : 15 bytes
 *
 * * 713D0003-503E-4C75-BA94-3148F18D941E : Accelerometer (50Hz : 5 Notifications / s)
 *    4 bytes : index of the notification
 *    10 values packed in 12 bits : 15 bytes
 *
 * * 713D0004-503E-4C75-BA94-3148F18D941E : Temperature (1Hz)
 *    4 bytes : index of the notification
 *    2 bytes : value of the first temperature sensor in Celcius * 10
 *    2 bytes : value of the second temperature sensor in Celcius * 10
 *
 * * 713D0005-503E-4C75-BA94-3148F18D941E : Impedance (1/600Hz)
 *    4 bytes : index of the notification
 *    2 bytes : measure between front left and left back
 *    2 bytes : measure between front left and right back
 *    2 bytes : measure between front right and left back
 *    2 bytes : measure between front right and right back
 *
 * * 713D0006-503E-4C75-BA94-3148F18D941E : Accelerometer (50Hz : 16~17 Notifications / s)
 *    4 bytes : index of the notification
 *    9 * 2 bytes values packed in 12 bits : 15 bytes
 *    x, y, z, x, y, z, x, y, z
 *
 * * 713D000f-503E-4C75-BA94-3148F18D941E : Setting Service (Write).
 *    1 bytes : command
 *        0x01: Put device in sleep mode with BLE Advertisement
 *        0x03: Set pref to disconnect on low temp measure
 *        0x04: Set pref to not disconnect on low temp measure
 *        0x05: Shutdowm ARM Cortex can be wake up by movement
 *        0x07: Set a new DiagW-xxxx number if never attributed
 *            Example : 0x07yyyyyyyy where yyyyyyyy is the serial number in hex it ll be displayed in the xxxx form in base36 as the device
 * name. 0x06: Instant impedance measure 0x0a: Read Temperature Calibration 0x0b: Write Temperature Calibration
 *
 * 713D0007-503E-4C75-BA94-3148F18D941E : Various information from device
 *  1 bytes : information type
 *      0x01 : device available memory percent type. Followed by one byte
 *
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_diagw_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_DIAGW_BLE_OBSERVER_PRIO,
 *                                   ble_diagw_on_ble_evt, &instance);
 *          @endcode
 */
#ifndef BLE_DIAGW_H
#define BLE_DIAGW_H

#include "app_ble_gap.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"
#include "sdk_config.h"
#include <stdbool.h>
#include <stdint.h>

#define BLE_DIAGW_BLE_OBSERVER_PRIO 2 // Priority of BLE Observer

/**@brief   Macro for defining a ble_diagw instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_DIAGW_DEF(_name)                                                                                                               \
    static volatile ble_diagw_t _name;                                                                                                     \
    NRF_SDH_BLE_OBSERVER(_name##_obs, BLE_DIAGW_BLE_OBSERVER_PRIO, ble_diagw_on_ble_evt, (ble_diagw_t *)&_name)

#define DIAGW_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN // UUID type for the DIAGW Service (vendor specific).

#define DIAGW_BASE_UUID                                                                                                                    \
    {                                                                                                                                      \
        {                                                                                                                                  \
            0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0x00, 0x00, 0x3D, 0x71                                 \
        }                                                                                                                                  \
    }

#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)

// Maximum length of data (in bytes) that can be transmitted to the peer by the diagw service module.
#define BLE_DIAGW_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OVERHEAD_LENGTH)
#else
#error NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

#define BLE_UUID_DIAGW_SERVICE                        0x0000 // The UUID of the DIAGW Service.
#define BLE_UUID_DIAGW_ECG_CHARACTERISTIC             0x0001 // 0x0024
#define BLE_UUID_DIAGW_BREATH_CHARACTERISTIC          0x0002 // 0x0027
#define BLE_UUID_DIAGW_TEMP_CHARACTERISTIC            0x0004 // 0x002A
#define BLE_UUID_DIAGW_IMP_CHARACTERISTIC             0x0005 // 0x002D
#define BLE_UUID_DIAGW_MEM_DUMP_CHARACTERISTIC        0x0003 // 0x0033
#define BLE_UUID_DIAGW_ACC_CHARACTERISTIC             0x0006 // 0x0030
#define BLE_UUID_DIAGW_MEM_DUMP_CTRLPT_CHARACTERISTIC 0x0007 // 0x0036
#define BLE_UUID_DIAGW_RX_CHARACTERISTIC              0x000F

#define TIMESTAMP_LEN                                 8U

#define ECG_DATA_LEN                                  15U
#define BREATH_DATA_LEN                               15U
#define TEMP_DATA_LEN                                 4U
#define IMP_DATA_LEN                                  8U
#define MEM_DUMP_DATA_LEN                             13U
#define ACC_DATA_LEN                                  14U
#define MEM_DUMP_CTRLPT_DATA_LEN                      16U
#define RX_DATA_LEN                                   16U

#define BLE_DIAGW_MAX_ECG_CHAR_LEN                    ((uint8_t)((ECG_DATA_LEN) + (TIMESTAMP_LEN)))
#define BLE_DIAGW_MAX_BREATH_CHAR_LEN                 ((uint8_t)((BREATH_DATA_LEN) + (TIMESTAMP_LEN)))
#define BLE_DIAGW_MAX_TEMP_CHAR_LEN                   ((uint8_t)((TEMP_DATA_LEN) + (TIMESTAMP_LEN)))
#define BLE_DIAGW_MAX_IMP_CHAR_LEN                    ((uint8_t)((IMP_DATA_LEN) + (TIMESTAMP_LEN)))
#define BLE_DIAGW_MAX_MEM_DUMP_CHAR_LEN               ((uint8_t)((MEM_DUMP_DATA_LEN) + (TIMESTAMP_LEN)))
#define BLE_DIAGW_MAX_ACC_CHAR_LEN                    ((uint8_t)((ACC_DATA_LEN) + (TIMESTAMP_LEN)))
#define BLE_DIAGW_MAX_MEM_DUMP_CTRLPT_CHAR_LEN        ((uint8_t)((MEM_DUMP_CTRLPT_DATA_LEN) + (TIMESTAMP_LEN)))
#define BLE_DIAGW_MAX_RX_CHAR_LEN                     ((uint8_t)((RX_DATA_LEN) + (TIMESTAMP_LEN)))

#define BLE_DIAGW_MAX_RAW_DATA_LEN                    BLE_DIAGW_MAX_ECG_CHAR_LEN

typedef enum notif_data_type_tag
{
    UNKNOWN_TYPE         = 0,
    ECG_TYPE             = 1,
    BREATH_TYPE          = 2,
    TEMP_TYPE            = 3,
    IMP_TYPE             = 4,
    MEM_DUMP_TYPE        = 5,
    ACC_TYPE             = 6,
    MEM_DUMP_CTRLPT_TYPE = 7,
    NOTIF_DATA_TYPE_SIZE
} notif_data_type_t;

typedef struct diagw_notif_tag
{
    union
    {
        struct
        {
            uint8_t ecg             : 1;
            uint8_t breath          : 1;
            uint8_t temp            : 1;
            uint8_t imp             : 1;
            uint8_t acc             : 1;
            uint8_t mem_dump        : 1;
            uint8_t mem_dump_ctrlpt : 1;
            uint8_t reserved        : 1;
        } items;
        uint8_t byte;
    } status;

} diagw_notif_t;

static const uint16_t notif_max_len[NOTIF_DATA_TYPE_SIZE] = {0,
                                                             BLE_DIAGW_MAX_ECG_CHAR_LEN,
                                                             BLE_DIAGW_MAX_BREATH_CHAR_LEN,
                                                             BLE_DIAGW_MAX_TEMP_CHAR_LEN,
                                                             BLE_DIAGW_MAX_IMP_CHAR_LEN,
                                                             BLE_DIAGW_MAX_MEM_DUMP_CHAR_LEN,
                                                             BLE_DIAGW_MAX_ACC_CHAR_LEN,
                                                             BLE_DIAGW_MAX_MEM_DUMP_CTRLPT_CHAR_LEN};

static const uint16_t notif_uuid[NOTIF_DATA_TYPE_SIZE] = {0,
                                                          BLE_UUID_DIAGW_ECG_CHARACTERISTIC,
                                                          BLE_UUID_DIAGW_BREATH_CHARACTERISTIC,
                                                          BLE_UUID_DIAGW_TEMP_CHARACTERISTIC,
                                                          BLE_UUID_DIAGW_IMP_CHARACTERISTIC,
                                                          BLE_UUID_DIAGW_MEM_DUMP_CHARACTERISTIC,
                                                          BLE_UUID_DIAGW_ACC_CHARACTERISTIC,
                                                          BLE_UUID_DIAGW_MEM_DUMP_CTRLPT_CHARACTERISTIC};

typedef enum
{
    BLE_DIAGW_EVT_RX_DATA,      // Data received.
    BLE_DIAGW_EVT_TX_RDY,       // Service is ready to accept new data to be transmitted.
    BLE_DIAGW_EVT_COMM_STARTED, // Notification has been enabled.
    BLE_DIAGW_EVT_COMM_STOPPED, // Notification has been disabled.
} ble_diagw_evt_type_t;         // DIAGW Service event types.

typedef struct ble_diagw_s ble_diagw_t; // Forward declaration of the ble_diagw_t type.

/**@brief   DIAGW Service @ref BLE_DIAGW_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_DIAGW_EVT_RX_DATA occurs.
 */
typedef struct
{
    uint8_t const *p_data; // A pointer to the buffer with received data.
    uint16_t       length; // Length of received data.
} ble_diagw_evt_rx_data_t;

/**@brief   DIAGW Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
    ble_diagw_evt_type_t type;    // Event type.
    ble_diagw_t         *p_diagw; // A pointer to the instance.
    union
    {
        ble_diagw_evt_rx_data_t rx_data; // @ref BLE_DIAGW_EVT_RX_DATA event data.
    } params;
} ble_diagw_evt_t;

typedef void (*ble_diagw_data_handler_t)(ble_diagw_evt_t *p_evt); // DIAGW Service event handler type.

/**@brief   DIAGW Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_diagw_init
 *          function.
 */
typedef struct
{
    ble_diagw_data_handler_t data_handler; // Event handler to be called for handling received data.
} ble_diagw_init_t;

/**@brief   DIAGW Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_diagw_s
{
    uint8_t                  uuid_type;                         // UUID type for DIAGW Service Base UUID.
    uint16_t                 service_handle;                    // Handle of DIAGW Service (as provided by the SoftDevice).
    ble_gatts_char_handles_t char_handle[NOTIF_DATA_TYPE_SIZE]; // Handles of DIAGW Characteristics (as provided by the SoftDevice).
    ble_gatts_char_handles_t rx_handle;
    volatile uint16_t        conn_handle;  // Current connection handle.
    ble_diagw_data_handler_t data_handler; // Event handler to be called for handling received data.
    volatile uint16_t        max_data_len;
    volatile diagw_notif_t   diagw_notif;
};

uint32_t     ble_diagw_init(ble_diagw_t *p_diagw, ble_diagw_init_t const *p_diagw_init);
void         ble_diagw_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);
uint32_t     ble_diagw_data_notif_send(notif_data_type_t dtype, uint8_t *buf, uint16_t len);
void         nrf_ble_diagw_on_gatt_evt(ble_diagw_t *p_diagw, nrf_ble_gatt_evt_t const *p_gatt_evt);
void         diagw_data_handler(ble_diagw_evt_t *p_evt);
uint16_t     ble_diagw_max_data_len_get(void);
ret_code_t   ble_diagw_mem_dump_ctrpt_indic_send(uint8_t *buf, uint16_t len);
ret_code_t   ble_diagw_mem_dump_notif_send(uint8_t *buf, uint16_t len);
bool         is_cccd_configured(notif_data_type_t type);
ble_diagw_t *ble_diagw_svc_instance_get(void);

#endif // BLE_DIAGW_H
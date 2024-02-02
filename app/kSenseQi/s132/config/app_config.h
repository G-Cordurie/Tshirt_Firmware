#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_MODULE_ENABLED(module) ((defined(module##_ENABLED) && (module##_ENABLED)) ? 1 : 0)

#define HW_TYPE                    HW_TYPE_0706

#define HW_REV_0706                0
#define HW_REV_0706B               1
#define HW_REV                     HW_REV_0706B

#define HW_REV_STR                 "0706cp"
#define MODEL_NUMBER_STR           "kSenseQi"

#define AUTODIAG_ENABLED           0
#define BLE_SEC_ENABLED            0
#define APP_BLE_NUS_ENABLED        0

#endif // APP_CONFIG_H
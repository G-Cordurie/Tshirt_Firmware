#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_MODULE_ENABLED(module) ((defined(module##_ENABLED) && (module##_ENABLED)) ? 1 : 0)

#define HW_TYPE                    HW_TYPE_Euro_Body

#define HW_REV_0202                0
#define HW_REV_0501                1
#define HW_REV_0201                2
#define HW_REV_0701                3
#define HW_REV_0704                4
#define HW_REV_0705                5
#define HW_REV_0705H               6
#define HW_REV_0705HM              7
#define HW_REV                     HW_REV_0102

#define HW_REV_STR                 "Euro_Body"
#define MODEL_NUMBER_STR           "Etincel"

#define AUTODIAG_ENABLED           0
#define BLE_SEC_ENABLED            0
#define APP_BLE_NUS_ENABLED        0

#endif // APP_CONFIG_H
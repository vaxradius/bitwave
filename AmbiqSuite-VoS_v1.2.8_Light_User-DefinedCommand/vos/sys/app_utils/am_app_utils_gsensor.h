//*****************************************************************************
//
//! @file am_app_utils_gsenor.h
//!
//! @brief RTOS printf functions for app layer use
//! 
//!
//
//*****************************************************************************
#ifndef AM_APP_UTILS_GSENSOR_H
#define AM_APP_UTILS_GSENSOR_H

#if USE_MAYA
#define GSENSOR_IOM     1

#define GET_I2CADDR(cfg)    \
(((cfg) & 0x000FFF00) >> (8 + 1))

#define LIS2DW_MEMS_I2C_ADDRESS         0x30  // 0x33
#define LIS2DW_WHO_AM_I                 0x0F  // device identification register
#define LIS2DW_CTRL0                    0x1E

#define AM_DEVICES_LIS2DW_CTRL_REG1        0x20
#define AM_DEVICES_LIS2DW_CTRL_REG2        0x21
#define AM_DEVICES_LIS2DW_CTRL_REG3        0x22
#define AM_DEVICES_LIS2DW_CTRL_REG4        0x23
#define AM_DEVICES_LIS2DW_CTRL_REG5        0x24
#define AM_DEVICES_LIS2DW_CTRL_REG6        0x25

#define X0_plus     150
#define X0_minus    -150
#define Y0_plus     150
#define Y0_minus    -150
#define Z_max       (-1000+200)
#define Z_min       (-1000-200)  

#define mg_per_bit          0.244 // +- 2G
#define GSENSOR_QUITE_MS    (20*1000)//(1*60*1000) //MS unit

extern void am_app_utils_gsensor_init(void);
extern void am_app_utils_gsensor_process(void);
extern void am_app_utils_gsensor_power_down(void);
extern void am_app_utils_gsensor_resetTick(void);

extern void am_app_util_gsensor_config(void);
extern void am_app_util_gsensor_button2_disble_handler(void);
extern void am_app_util_gsensor_powerdown(void);
#endif // USE_MAYA

#endif // AM_APP_UTILS_GSENSOR_H

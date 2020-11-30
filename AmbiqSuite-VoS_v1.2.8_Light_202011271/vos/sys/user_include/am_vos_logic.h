//*****************************************************************************
//
//! @file am_vos_logic.h
//!
//! @brief 
//! 
//!
//
//*****************************************************************************
#ifndef AM_VOS_LOGIC_H
#define AM_VOS_LOGIC_H

typedef enum logic_power_states{
    APP_LOGIC_POWERING_UP,
    APP_LOGIC_POWER_ON,
    APP_LOGIC_POWERING_DOWN,
    APP_LOGIC_POWER_OFF
}enum_logic_power_states_t;

extern void am_vos_logic_led_all_on(void);
extern void am_vos_logic_led_all_off(void);
extern void am_vos_logic_led_swirl(uint8_t type);
extern void am_vos_logic_button_init(void);
extern void am_vos_logic_button_process(void);
extern void am_vos_logic_system_power_off(void);
extern void am_vos_logic_cmd_led_on(int8_t i8CmdIndex);

void am_vos_logic_maya_hw_init(void);
uint32_t am_vos_logic_check_button_long(uint32_t gpio, uint32_t delay);

#endif // AM_VOS_LOGIC_H

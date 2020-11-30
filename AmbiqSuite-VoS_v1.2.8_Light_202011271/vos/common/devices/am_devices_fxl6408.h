
#ifndef AM_APP_UTILS_GPIO_FXL6408_H
#define AM_APP_UTILS_GPIO_FXL6408_H

typedef enum
{
    AM_FX6408_GPIO_OUTPUT_CLEAR,
    AM_FX6408_GPIO_OUTPUT_SET,
    AM_FX6408_GPIO_OUTPUT_TRISTATE_DISABLE,
    AM_FX6408_GPIO_OUTPUT_TRISTATE_ENABLE,
} am_device_fx6408_write_type_e;

#define GET_I2CADDR(cfg)    \
(((cfg) & 0x000FFF00) >> (8 + 1))

#define FLX6408_IOM             1
#define FXL6408_GPIOEXT_I2C_ADDRESS             0x86

#define AM_DEVICES_FXL6408_REG_CHIPID           0x01    // Device ID & Control
#define AM_DEVICES_FXL6408_REG_IODIR            0x03    // IO Direction
#define AM_DEVICES_FXL6408_REG_OUT_STATE        0x05    // Output State
#define AM_DEVICES_FXL6408_REG_OUT_HIGHZ        0x07    // Output High-Z
#define AM_DEVICES_FXL6408_REG_IN_DEFAULT       0x09    // Input default state
#define AM_DEVICES_FXL6408_REG_PULL_EN          0x0B    // Pull Enable
#define AM_DEVICES_FXL6408_REG_PULL_UPDOWN      0x0D    // Pull-Down or Up
#define AM_DEVICES_FXL6408_REG_IN_STATE         0x0F    // Input state
#define AM_DEVICES_FXL6408_REG_IRQ_MASK         0x11    // Interrupt Mask
#define AM_DEVICES_FXL6408_REG_IRQ_STATUS       0x13    // Interrupt Status

#define FXL6408_GPIO_DIR_INPUT                  0
#define FXL6408_GPIO_DIR_OUTPUT                 1

#define VM_AMIC_CTRL_THRES1                     0
#define VM_AMIC_CTRL_THRES2                     1
#define VM_AMIC_CTRL_GAIN                       2

void fxl6408_gpio_pinconfig(uint8_t ui8Pin, uint8_t ui8Dir);
void am_app_utils_fxl6408_init(void);
uint8_t fxl6408_gpio_state_read(uint8_t ui8Pin);
uint32_t fxl6408_gpio_state_write(uint8_t ui8Pin, am_device_fx6408_write_type_e eWriteType);

#endif

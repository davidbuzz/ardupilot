// /*
//  * This file is free software: you can redistribute it and/or modify it
//  * under the terms of the GNU General Public License as published by the
//  * Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * This file is distributed in the hope that it will be useful, but
//  * WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//  * See the GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License along
//  * with this program.  If not, see <http://www.gnu.org/licenses/>.
//  *
//  * Code by Andrew Tridgell and Siddharth Bharat Purohit
//  */
// #pragma once

// #include "AP_HAL_ESP32.h"

// #include "esp_types.h"

// #include "hal/gpio_hal.h" // for gpio_hal_context_t
// #include "hal/gpio_types.h" //gpio_port_t GPIO_PORT_0
// #include "soc/soc_caps.h"
// #include "hal/gpio_ll.h"

// #ifndef HAL_GPIO_LED_ON
// #define HAL_GPIO_LED_ON 0
// #endif

// #ifndef HAL_GPIO_LED_OFF
// #define HAL_GPIO_LED_OFF 1
// #endif

//     gpio_hal_context_t gpiohal;


// // chibios typedef/s, we'll keep consistent here
// // https://github.com/ChibiOS/ChibiOS/blob/stable_21.11.x/os/hal/ports/RP/LLD/GPIOv1/hal_pal_lld.h#L234
// typedef uint32_t ioline_t;
// // https://github.com/ChibiOS/ChibiOS/blob/master/os/hal/include/hal_pal.h#L148
// typedef void (*palcallback_t)(void *arg);
// #define PAL_ADUCM_OEN_INPUT             (0U << 2U)
// #define PAL_ADUCM_PUL_PULLUP            (1U << 3U)
// #define PAL_ADUCM_OCE_HIGHZ             (1U << 4U)
// #define PAL_MODE_INPUT_PULLDOWN         PAL_ADUCM_OCE_HIGHZ
// #define PAL_MODE_INPUT_PULLUP           (PAL_ADUCM_OEN_INPUT |              
//                                          PAL_ADUCM_PUL_PULLUP)
// #define PAL_ADUCM_OEN_OUTPUT            (1U << 2U)
// #define PAL_MODE_OUTPUT_PUSHPULL        PAL_ADUCM_OEN_OUTPUT

// #define PAL_ADUCM_PUL_FLOATING          (0U << 3U)
// #define PAL_MODE_INPUT                  (PAL_ADUCM_OEN_INPUT |              
//                                          PAL_ADUCM_PUL_FLOATING)

// /**
//  * @brief   Decodes a pad identifier from a line identifier.
//  */
// #define PAL_PAD(line)                                                       
//   ((uint32_t)((uint32_t)(line) & 0x0000000FU))

// #define PAL_PORT_BIT(n) ((ioportmask_t)(1U << (n)))
// // stm32:
// //#define PAL_PORT(line)   ((stm32_gpio_t *)(((uint32_t)(line)) & 0xFFFFFFF0U))
// // esp32:
// // https://github.com/espressif/esp-idf/blob/5faf116d26d1f171b6fc422a3a8c9c0b184bc65b/components/hal/esp32/include/hal/gpio_ll.h

// #define PAL_PORT(line)   ((gpio_config_t *)(((uint32_t)(line)) & 0xFFFFFFF0U))

// #define PAL_LOW 0

// #define pal_lld_setpadmode(port, pad, mode)                                 
//   do {                                                                      
//     (void)port;                                                             
//     (void)pad;                                                              
//     (void)mode;                                                             
//   } while (false)

// // chibios:
// #define palSetPadMode(port, pad, mode) pal_lld_setpadmode(port, pad, mode)

// #define palSetLineMode(line, mode)                                          
//   palSetPadMode(PAL_PORT(line), PAL_PAD(line), mode)


// #ifndef HAL_GPIO_PINS
// #define HAL_GPIO_PINS {}
// #endif

// /*
//   pin types for alternative configuration
//  */
// enum class PERIPH_TYPE : uint8_t {
//     UART_RX,
//     UART_TX,
//     I2C_SDA,
//     I2C_SCL,
//     OTHER,
//     GPIO,
// };

// class ESP32::GPIO : public AP_HAL::GPIO {
// public:
//     GPIO();
//     void    init() override;
//     void    pinMode(uint8_t pin, uint8_t output) override;
//     uint8_t read(uint8_t pin) override;
//     void    write(uint8_t pin, uint8_t value) override;
//     void    toggle(uint8_t pin) override;

//     /* Alternative interface: */
//     AP_HAL::DigitalSource* channel(uint16_t n) override;

//     /* Interrupt interface - fast, for RCOutput and SPI radios */
//     bool    attach_interrupt(uint8_t interrupt_num,
//                              AP_HAL::Proc p,
//                              INTERRUPT_TRIGGER_TYPE mode) override;

//     /* Interrupt interface - for AP_HAL::GPIO */
//     bool    attach_interrupt(uint8_t pin,
//                              irq_handler_fn_t fn,
//                              INTERRUPT_TRIGGER_TYPE mode) override;

//     /* return true if USB cable is connected */
//     bool    usb_connected(void) override;

//     void set_usb_connected() { _usb_connected = true; }

//     /* attach interrupt via ioline_t */
//     bool _attach_interrupt(ioline_t line, AP_HAL::Proc p, uint8_t mode);

//     /*
//       block waiting for a pin to change. A timeout of 0 means wait
//       forever. Return true on pin change, false on timeout
//      */
//     bool wait_pin(uint8_t pin, INTERRUPT_TRIGGER_TYPE mode, uint32_t timeout_us) override;

// #ifndef IOMCU_FW
//     // timer tick
//     void timer_tick(void) override;
// #endif

//     // check if a pin number is valid
//     bool valid_pin(uint8_t pin) const override;

//     // return servo channel associated with GPIO pin.  Returns true on success and fills in servo_ch argument
//     // servo_ch uses zero-based indexing
//     bool pin_to_servo_channel(uint8_t pin, uint8_t& servo_ch) const override;

//     /*
//       resolve an ioline to take account of alternative configurations
//      */
//     static ioline_t resolve_alt_config(ioline_t base, PERIPH_TYPE ptype, uint8_t instance);

// #if defined(STM32F7) || defined(STM32H7) || defined(STM32F4) || defined(STM32F3) || defined(STM32G4) || defined(STM32L4)
//     // allow for save and restore of pin settings
//     bool    get_mode(uint8_t pin, uint32_t &mode) override;
//     void    set_mode(uint8_t pin, uint32_t mode) override;
// #endif
    
// private:
//     bool _usb_connected;
//     bool _ext_started;

//     bool _attach_interruptI(ioline_t line, palcallback_t cb, void *p, uint8_t mode);
//     bool _attach_interrupt(ioline_t line, palcallback_t cb, void *p, uint8_t mode);
// #ifdef HAL_PIN_ALT_CONFIG
//     void setup_alt_config(void);
//     static uint8_t alt_config;
// #endif
// };

// class ESP32::DigitalSource : public AP_HAL::DigitalSource {
// public:
//     DigitalSource(ioline_t line);
//     void    mode(uint8_t output) override;
//     uint8_t read() override;
//     void    write(uint8_t value) override;
//     void    toggle() override;
// private:
//     ioline_t line;
// };

// #if HAL_WITH_IO_MCU
// class ESP32::IOMCU_DigitalSource : public AP_HAL::DigitalSource {
// public:
//     IOMCU_DigitalSource(uint8_t _pin);
//     void    write(uint8_t value) override;
//     void    toggle() override;
//     // IOMCU GPIO is write only
//     void    mode(uint8_t output) override {};
//     uint8_t    read() override { return 0; }
// private:
//     uint8_t pin;
// };
// #endif

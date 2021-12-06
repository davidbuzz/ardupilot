#ifndef _MCUCONF_H_
#define _MCUCONF_H_

//#include "stm32_tim.h"

#define MIMXRT1062_MCUCONF

/* The NXP USB stack uses more stack space than the default 256 byte of thread
 * working area for the test command can fit, so make some more room: */
#define SHELL_CMD_TEST_WA_SIZE              THD_WORKING_AREA_SIZE(1024)

// TEENSY :
// typedef struct {
//   __IO uint32_t DR;                                /**< GPIO data register, offset: 0x0 */
//   __IO uint32_t GDIR;                              /**< GPIO direction register, offset: 0x4 */
//   __I  uint32_t PSR;                               /**< GPIO pad status register, offset: 0x8 */
//   __IO uint32_t ICR1;                              /**< GPIO interrupt configuration register1, offset: 0xC */
//   __IO uint32_t ICR2;                              /**< GPIO interrupt configuration register2, offset: 0x10 */
//   __IO uint32_t IMR;                               /**< GPIO interrupt mask register, offset: 0x14 */
//   __IO uint32_t ISR;                               /**< GPIO interrupt status register, offset: 0x18 */
//   __IO uint32_t EDGE_SEL;                          /**< GPIO edge select register, offset: 0x1C */
//        uint8_t RESERVED_0[100];
//   __O  uint32_t DR_SET;                            /**< GPIO data register SET, offset: 0x84 */
//   __O  uint32_t DR_CLEAR;                          /**< GPIO data register CLEAR, offset: 0x88 */
//   __O  uint32_t DR_TOGGLE;                         /**< GPIO data register TOGGLE, offset: 0x8C */
// } GPIO_Type;

// see GPIO_PinInit/GPIO_PinWrite in fsl_gpio.h... this is COMPAT SHIM
typedef GPIO_Type * stm32_gpio_t;

// taken from https://github.com/steve-bate/ChibiOS-RPi/blob/master/os/hal/platforms/STM32/stm32.h
typedef struct {
  volatile uint16_t     CR1;
  uint16_t              _resvd0;
  volatile uint16_t     CR2;
  uint16_t              _resvd1;
  volatile uint16_t     SMCR;
  uint16_t              _resvd2;
  volatile uint16_t     DIER;
  uint16_t              _resvd3;
  volatile uint16_t     SR;
  uint16_t              _resvd4;
  volatile uint16_t     EGR;
  uint16_t              _resvd5;
  volatile uint16_t     CCMR1;
  uint16_t              _resvd6;
  volatile uint16_t     CCMR2;
  uint16_t              _resvd7;
  volatile uint16_t     CCER;
  uint16_t              _resvd8;
  volatile uint32_t     CNT;
  volatile uint16_t     PSC;
  uint16_t              _resvd9;
  volatile uint32_t     ARR;
  volatile uint16_t     RCR;
  uint16_t              _resvd10;
  volatile uint32_t     CCR[4];
  volatile uint16_t     BDTR;
  uint16_t              _resvd11;
  volatile uint16_t     DCR;
  uint16_t              _resvd12;
  volatile uint16_t     DMAR;
  uint16_t              _resvd13;
  volatile uint16_t     OR;
  uint16_t              _resvd14;
} stm32_tim_t;
//todo NXP calls them 'Qtimer' peripherals, as there's 4 timers in each, and i think the teensy has 4?(2?) of those, so 16? total timers.
// https://community.nxp.com/t5/i-MX-RT/XBAR-for-Qtimer/td-p/987224
/* "the QTIMER module contains four identical counter/timer groups. Each 16-bit counter/timer group contains a prescaler,
    a counter, a load register, a hold register, a capture register, two compare registers, two status and control 
    registers, and one control register.""
*/

//typedef stm32_tim_t xxxxx


typedef GPIO_Type * stm32_gpio_t;

//__TEENSY4__

#define GPIOA                           ((stm32_gpio_t *)GPIOA_BASE)
#define GPIOB                           ((stm32_gpio_t *)GPIOB_BASE)
#define GPIOC                           ((stm32_gpio_t *)GPIOC_BASE)
#define GPIOD                           ((stm32_gpio_t *)GPIOD_BASE)
#define GPIOE                           ((stm32_gpio_t *)GPIOE_BASE)
#define GPIOF                           ((stm32_gpio_t *)GPIOF_BASE)
#define GPIOG                           ((stm32_gpio_t *)GPIOG_BASE)
#define GPIOH                           ((stm32_gpio_t *)GPIOH_BASE)
#define GPIOI                           ((stm32_gpio_t *)GPIOI_BASE)
#define GPIOJ                           ((stm32_gpio_t *)GPIOJ_BASE)
#define GPIOK                           ((stm32_gpio_t *)GPIOK_BASE)

// we want 96 bits of unique info to be compatibvle with stm32's udid. see 'Fusemap Descriptions' table of the reference manual
// 0x4F0 = 32bit val => usb vid(16bit)/pid(16bit)
// 0x620 = 48bits =? ethernet MAC 1
// 0x630 = 48bits =? ethernet MAC 2
#define UDID_START 0x620

#define HAL_NO_FLASH_SUPPORT true

#define HAL_BOARD_INIT_HOOK_CALL 

#define STM32_DMA_SUPPORTS_DMAMUX FALSE

// see ../../modules/ChibiOS/../ChibiOS-Contrib/os/hal/ports/MIMXRT1062/LLD/DMAv1/stm32_dma.h:43:0: warning: "STM32_DMA_STREAMS" redefined
//#define STM32_DMA_STREAMS 4

//#define DISABLE_DSHOT true

//typedef stm32_dma_stream_t

//typedef void* stm32_dma_stream_t

// the avr-MEGA uses uint8_t as its pwm channel number... so can we.
// see also ChibiOS-Contrib/os/hal/ports/MIMXRT1062/LLD/TIMv1/hal_pwm_lld.h as its defined the same way there
typedef uint8_t pwmchannel_t;


//../../libraries/AP_HAL_ChibiOS/RCOutput.h:29:6: warning: "STM32_DMA_ADVANCED" is not defined [-Wundef]
#define STM32_DMA_ADVANCED          FALSE


// for modules/ChibiOS-Contrib/ext/mcux-sdk/devices/MIMXRT1062/fsl_device_registers.h
#define CPU_MIMXRT1062DVJ6A true

void stm32_clock_init(void);
void stm32_clock_init(void) {
//#warning buzz todo clock init
}

#endif /* _MCUCONF_H_ */
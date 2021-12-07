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

//__TEENSY4__.  this maps ardupilot's 'A','B' etc to Teensy's 1..2..3 while still calling it stm32_gpio_t not GPIO_Type

#define GPIOA                           ((stm32_gpio_t *)GPIO1_BASE)
#define GPIOB                           ((stm32_gpio_t *)GPIO2_BASE)
#define GPIOC                           ((stm32_gpio_t *)GPIO3_BASE)
#define GPIOD                           ((stm32_gpio_t *)GPIO4_BASE)
#define GPIOE                           ((stm32_gpio_t *)GPIO5_BASE)
#define GPIOF                           ((stm32_gpio_t *)GPIO6_BASE)
#define GPIOG                           ((stm32_gpio_t *)GPIO7_BASE)
#define GPIOH                           ((stm32_gpio_t *)GPIO8_BASE)
#define GPIOI                           ((stm32_gpio_t *)GPIO9_BASE)
#define GPIOJ                           ((stm32_gpio_t *)GPIO10_BASE)
#define GPIOK                           ((stm32_gpio_t *)GPIO11_BASE)

// we want 96 bits of unique info to be compatibvle with stm32's udid. see 'Fusemap Descriptions' table of the reference manual
// 0x4F0 = 32bit val => usb vid(16bit)/pid(16bit)
// 0x620 = 48bits =? ethernet MAC 1
// 0x630 = 48bits =? ethernet MAC 2
#define UDID_START 0x620

#define HAL_NO_FLASH_SUPPORT true

#define HAL_BOARD_INIT_HOOK_CALL 

//diables stm32_dmamux.h  and DSHOT and assorted other stuff calling dmaSetRequestSource
#define STM32_DMA_SUPPORTS_DMAMUX FALSE

// see ../../modules/ChibiOS/../ChibiOS-Contrib/os/hal/ports/MIMXRT1062/LLD/DMAv1/stm32_dma.h:43:0: warning: "STM32_DMA_STREAMS" redefined
//#define STM32_DMA_STREAMS 4

//#define DISABLE_DSHOT true

//typedef stm32_dma_stream_t

//typedef void* stm32_dma_stream_t

// stm32_dma_stream_tcopied from stm32 hack, needs mods
typedef struct {
  //DMA_Stream_TypeDef    *stream;        /**< @brief Associated DMA stream.  */
  volatile uint32_t     *ifcr;          /**< @brief Associated IFCR reg.    */
#if (STM32_DMA_SUPPORTS_DMAMUX == TRUE) || defined(__DOXYGEN__)
  //DMAMUX_Channel_TypeDef *mux;          /**< @brief Associated DMA mux.     */
#else
  uint8_t               dummy;          /**< @brief Filler.                 */
#endif
  uint8_t               shift;          /**< @brief Bits offset in xIFCR
                                             register.                      */
  uint8_t               selfindex;      /**< @brief Index to self in array. */
  uint8_t               vector;         /**< @brief Associated IRQ vector.  */
} stm32_dma_stream_t;

// used outside chibios,  in rc output, i think...  dmaSetRequestSource  ?
#define STM32_DMA_STREAM_ID(dma, stream) ((((dma) - 1U) * 8U) + (stream))


// the avr-MEGA uses uint8_t as its pwm channel number... so can we.
// see also ChibiOS-Contrib/os/hal/ports/MIMXRT1062/LLD/TIMv1/hal_pwm_lld.h as its defined the same way there
typedef uint8_t pwmchannel_t;


//../../libraries/AP_HAL_ChibiOS/RCOutput.h:29:6: warning: "STM32_DMA_ADVANCED" is not defined [-Wundef]
#define STM32_DMA_ADVANCED          FALSE


// for modules/ChibiOS-Contrib/ext/mcux-sdk/devices/MIMXRT1062/fsl_device_registers.h
#define CPU_MIMXRT1062DVJ6A true

#ifndef stm32_clock_init_h
#define stm32_clock_init_h
void stm32_clock_init(void);
//void stm32_clock_init(void) {
//#warning buzz todo clock init
//}
#endif

// buzz hack ripped off from stm32h7's stm32_dmamux.h, completely wrong for teensy. but at least defining them
//./hwdef.h:314:73: error: 'STM32_DMAMUX1_UART4_RX' was not declared in this scope
#define STM32_DMAMUX1_REQ_GEN0      1
#define STM32_DMAMUX1_REQ_GEN1      2
#define STM32_DMAMUX1_REQ_GEN2      3
#define STM32_DMAMUX1_REQ_GEN3      4
#define STM32_DMAMUX1_REQ_GEN4      5
#define STM32_DMAMUX1_REQ_GEN5      6
#define STM32_DMAMUX1_REQ_GEN6      7
#define STM32_DMAMUX1_REQ_GEN7      8
#define STM32_DMAMUX1_ADC1          9
#define STM32_DMAMUX1_ADC2          10
#define STM32_DMAMUX1_TIM1_CH1      11
#define STM32_DMAMUX1_TIM1_CH2      12
#define STM32_DMAMUX1_TIM1_CH3      13
#define STM32_DMAMUX1_TIM1_CH4      14
#define STM32_DMAMUX1_TIM1_UP       15
#define STM32_DMAMUX1_TIM1_TRIG     16
#define STM32_DMAMUX1_TIM1_COM      17
#define STM32_DMAMUX1_TIM2_CH1      18
#define STM32_DMAMUX1_TIM2_CH2      19
#define STM32_DMAMUX1_TIM2_CH3      20
#define STM32_DMAMUX1_TIM2_CH4      21
#define STM32_DMAMUX1_TIM2_UP       22
#define STM32_DMAMUX1_TIM3_CH1      23
#define STM32_DMAMUX1_TIM3_CH2      24
#define STM32_DMAMUX1_TIM3_CH3      25
#define STM32_DMAMUX1_TIM3_CH4      26
#define STM32_DMAMUX1_TIM3_UP       27
#define STM32_DMAMUX1_TIM3_TRIG     28
#define STM32_DMAMUX1_TIM4_CH1      29
#define STM32_DMAMUX1_TIM4_CH2      30
#define STM32_DMAMUX1_TIM4_CH3      31
#define STM32_DMAMUX1_TIM4_UP       32
#define STM32_DMAMUX1_I2C1_RX       33
#define STM32_DMAMUX1_I2C1_TX       34
#define STM32_DMAMUX1_I2C2_RX       35
#define STM32_DMAMUX1_I2C2_TX       36
#define STM32_DMAMUX1_SPI1_RX       37
#define STM32_DMAMUX1_SPI1_TX       38
#define STM32_DMAMUX1_SPI2_RX       39
#define STM32_DMAMUX1_SPI2_TX       40
#define STM32_DMAMUX1_USART1_RX     41
#define STM32_DMAMUX1_USART1_TX     42
#define STM32_DMAMUX1_USART2_RX     43
#define STM32_DMAMUX1_USART2_TX     44
#define STM32_DMAMUX1_USART3_RX     45
#define STM32_DMAMUX1_USART3_TX     46
#define STM32_DMAMUX1_TIM8_CH1      47
#define STM32_DMAMUX1_TIM8_CH2      48
#define STM32_DMAMUX1_TIM8_CH3      49
#define STM32_DMAMUX1_TIM8_CH4      50
#define STM32_DMAMUX1_TIM8_UP       51
#define STM32_DMAMUX1_TIM8_TRIG     52
#define STM32_DMAMUX1_TIM8_COM      53
#define STM32_DMAMUX1_RESERVED54    54
#define STM32_DMAMUX1_TIM5_CH1      55
#define STM32_DMAMUX1_TIM5_CH2      56
#define STM32_DMAMUX1_TIM5_CH3      57
#define STM32_DMAMUX1_TIM5_CH4      58
#define STM32_DMAMUX1_TIM5_UP       59
#define STM32_DMAMUX1_TIM5_TRIG     60
#define STM32_DMAMUX1_SPI3_RX       61
#define STM32_DMAMUX1_SPI3_TX       62
#define STM32_DMAMUX1_UART4_RX      63
#define STM32_DMAMUX1_UART4_TX      64
#define STM32_DMAMUX1_UART5_RX      65
#define STM32_DMAMUX1_UART5_TX      66
#define STM32_DMAMUX1_DAC1_CH1      67  /* Renamed to L4 name.*/
#define STM32_DMAMUX1_DAC1_CH2      68  /* Renamed to L4 name.*/
#define STM32_DMAMUX1_TIM6_UP       69
#define STM32_DMAMUX1_TIM7_UP       70
#define STM32_DMAMUX1_USART6_RX     71
#define STM32_DMAMUX1_USART6_TX     72
#define STM32_DMAMUX1_I2C3_RX       73
#define STM32_DMAMUX1_I2C3_TX       74
#define STM32_DMAMUX1_DCMI          75
#define STM32_DMAMUX1_CRYP_IN       76
#define STM32_DMAMUX1_CRYP_OUT      77
#define STM32_DMAMUX1_HASH_IN       78
#define STM32_DMAMUX1_UART7_RX      79
#define STM32_DMAMUX1_UART7_TX      80
#define STM32_DMAMUX1_UART8_RX      81
#define STM32_DMAMUX1_UART8_TX      82
#define STM32_DMAMUX1_SPI4_RX       83
#define STM32_DMAMUX1_SPI4_TX       84
#define STM32_DMAMUX1_SPI5_RX       85
#define STM32_DMAMUX1_SPI5_TX       86
#define STM32_DMAMUX1_SAI1_A        87
#define STM32_DMAMUX1_SAI1_B        88
#define STM32_DMAMUX1_SAI2_A        89
#define STM32_DMAMUX1_SAI2_B        90
#define STM32_DMAMUX1_SWPMI_RX      91
#define STM32_DMAMUX1_SQPMI_TX      92
#define STM32_DMAMUX1_SPDIFRX_DT    93
#define STM32_DMAMUX1_SPDIFRX_CS    94
#define STM32_DMAMUX1_HR_REQ1       95
#define STM32_DMAMUX1_HR_REQ2       96
#define STM32_DMAMUX1_HR_REQ3       97
#define STM32_DMAMUX1_HR_REQ4       98
#define STM32_DMAMUX1_HR_REQ5       99
#define STM32_DMAMUX1_HR_REQ6       100
#define STM32_DMAMUX1_DFSDM1_DMA0   101
#define STM32_DMAMUX1_DFSDM1_DMA1   102
#define STM32_DMAMUX1_DFSDM1_DMA2   103
#define STM32_DMAMUX1_DFSDM1_DMA3   104
#define STM32_DMAMUX1_TIM15_CH1     105
#define STM32_DMAMUX1_TIM15_UP      106
#define STM32_DMAMUX1_TIM15_TRIG    107
#define STM32_DMAMUX1_TIM15_COM     108
#define STM32_DMAMUX1_TIM16_CH1     109
#define STM32_DMAMUX1_TIM16_UP      110
#define STM32_DMAMUX1_TIM17_CH1     111
#define STM32_DMAMUX1_TIM17_UP      112
#define STM32_DMAMUX1_SAI3_A        113
#define STM32_DMAMUX1_SAI3_B        114
#define STM32_DMAMUX1_ADC3          115

#endif /* _MCUCONF_H_ */
/*
 generated hardware definitions from hwdef.dat - DO NOT EDIT
*/

#pragma once

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define MHZ (1000U*1000U)
#define KHZ (1000U)

// MCU type (ChibiOS define)
#define esp32_MCUCONF
#define classic

// crystal frequency
// UART used for stdout (printf)
#define HAL_USE_SDC FALSE
#define HAL_USE_USB TRUE
#define HAL_USE_SERIAL_USB TRUE
#define HAL_USE_HW_RNG FALSE
#define HAL_PROCESS_STACK_SIZE 0x1C00
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32
#define HAL_CHIBIOS_ARCH_FMUV3 1
#define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
#define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
#define HAL_STORAGE_SIZE 16384
#define HAL_HAVE_SAFETY_SWITCH 1
#define HAL_WITH_RAMTRON 1
#define HAL_HAVE_IMU_HEATER 1
#define HAL_OS_FATFS_IO 1
#define HAL_BATT_VOLT_PIN 2
#define HAL_BATT_CURR_PIN 3
#define HAL_BATT_VOLT_SCALE 10.1
#define HAL_BATT_CURR_SCALE 17.0
#define HAL_I2C_MAX_CLOCK 100000
#define BOARD_FLASH_SIZE 2048
#define CRT1_AREAS_NUMBER 1

// location of loaded firmware
#define FLASH_LOAD_ADDRESS 0x08004000
#define EXTERNAL_PROG_FLASH_MB 0
#define CRT1_RAMFUNC_ENABLE FALSE
#define HAL_CRASHDUMP_ENABLE 1

// memory regions
#define HAL_MEMORY_REGIONS {(void*)0x20000000, 0x00030000, 0x01 }, {(void*)0x10000000, 0x00010000, 0x02 }
#define HAL_CC_MEMORY_REGIONS {0x20000000, 0x20030000, CRASH_CATCHER_BYTE }, {0x10000000, 0x10010000, CRASH_CATCHER_BYTE }
#define HAL_MEMORY_TOTAL_KB 256
#define HAL_RAM0_START 0x20000000

// CPU serial number (12 bytes)
#define UDID_START UID_BASE


// APJ board ID (for bootloaders)
#define APJ_BOARD_ID 9

#ifndef HAL_ENABLE_THREAD_STATISTICS
#define HAL_ENABLE_THREAD_STATISTICS FALSE
#endif
    
#ifndef HAL_HAVE_HARDWARE_DOUBLE
#define HAL_HAVE_HARDWARE_DOUBLE 0
#endif
#define HAL_EXPECTED_SYSCLOCK 168000000
#define HAL_SPI_BUS_LIST HAL_SPI1_CONFIG


// SPI device table
#define HAL_SPI_DEVICE0  SPIDesc("mpu9250"        ,  0,  4, PAL_LINE(GPIOA,5U) , SPIDEV_MODE3,   4*MHZ,   8*MHZ)
#define HAL_SPI_DEVICE_LIST HAL_SPI_DEVICE0

#define HAL_WITH_SPI_MPU9250 1

// ADC config
#define ANALOG_VCC_5V_PIN 4
#define HAL_HAVE_BOARD_VOLTAGE 1
#define HAL_ANALOG_PINS { \
{  2,    3.30/4096 }, /* PA2 BATT_VOLTAGE_SENS */ \
{  3,    3.30/4096 }, /* PA3 BATT_CURRENT_SENS */ \
{  4,  2*3.30/4096 }, /* PA4 VDD_5V_SENS */ \
}

// GPIO config
#define HAL_GPIO_LINE_GPIO1 PAL_LINE(GPIOB,0U)
#define HAL_GPIO_LINE_GPIO2 PAL_LINE(GPIOB,1U)
#define HAL_GPIO_LINE_GPIO77 PAL_LINE(GPIOA,15U)
#define HAL_GPIO_PINS { \
{   1, true,  0, PAL_LINE(GPIOB,0U)}, /* PB0 EXTERN_GPIO1 OUTPUT */ \
{   2, true,  0, PAL_LINE(GPIOB,1U)}, /* PB1 EXTERN_GPIO2 OUTPUT */ \
{  77, true,  0, PAL_LINE(GPIOA,15U)}, /* PA15 TIM2_CH1 TIM2 AF1 */ \
}

// full pin define list
#define HAL_GPIO_PIN_BATT_CURRENT_SENS    PAL_LINE(GPIOA,3U)
#define HAL_GPIO_PIN_BATT_VOLTAGE_SENS    PAL_LINE(GPIOA,2U)
#define HAL_GPIO_PIN_BOOT1                PAL_LINE(GPIOB,2U)
#define HAL_GPIO_PIN_EXTERN_GPIO1         PAL_LINE(GPIOB,0U)
#define HAL_GPIO_PIN_EXTERN_GPIO2         PAL_LINE(GPIOB,1U)
#define HAL_GPIO_PIN_FMU_SW0              PAL_LINE(GPIOB,3U)
#define HAL_GPIO_PIN_JTCK_SWCLK           PAL_LINE(GPIOA,14U)
#define HAL_GPIO_PIN_JTMS_SWDIO           PAL_LINE(GPIOA,13U)
#define HAL_GPIO_PIN_MAG_CS               PAL_LINE(GPIOA,26U)
#define HAL_GPIO_PIN_MPU_CS               PAL_LINE(GPIOA,5U)
#define HAL_GPIO_PIN_OTG_FS_DM            PAL_LINE(GPIOA,11U)
#define HAL_GPIO_PIN_OTG_FS_DP            PAL_LINE(GPIOA,12U)
#define HAL_GPIO_PIN_SPI1_MISO            PAL_LINE(GPIOA,19U)
#define HAL_GPIO_PIN_SPI1_MOSI            PAL_LINE(GPIOA,23U)
#define HAL_GPIO_PIN_SPI1_SCK             PAL_LINE(GPIOA,18U)
#define HAL_GPIO_PIN_TIM2_CH1             PAL_LINE(GPIOA,15U)
#define HAL_GPIO_PIN_UART4_RX             PAL_LINE(GPIOA,1U)
#define HAL_GPIO_PIN_UART4_TX             PAL_LINE(GPIOA,0U)
#define HAL_GPIO_PIN_VBUS                 PAL_LINE(GPIOA,9U)
#define HAL_GPIO_PIN_VDD_5V_PERIPH_EN     PAL_LINE(GPIOA,8U)
#define HAL_GPIO_PIN_VDD_5V_SENS          PAL_LINE(GPIOA,4U)
#define HAL_GPIO_PIN_VDD_BRICK2_nVALID    PAL_LINE(GPIOB,7U)
#define HAL_GPIO_PIN_VDD_BRICK_nVALID     PAL_LINE(GPIOB,5U)

// peripherals enabled


// auto-generated DMA mapping from dma_resolver.py
#define STM32_ADC_ADC1_DMA_STREAM      STM32_DMA_STREAM_ID(2, 0)
#define STM32_ADC_ADC1_DMA_CHAN        0
#define STM32_SPI_SPI1_RX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 2)
#define STM32_SPI_SPI1_RX_DMA_CHAN     3
#define STM32_SPI_SPI1_TX_DMA_STREAM   STM32_DMA_STREAM_ID(2, 3)
#define STM32_SPI_SPI1_TX_DMA_CHAN     3
#define STM32_UART_UART4_RX_DMA_STREAM STM32_DMA_STREAM_ID(1, 2)
#define STM32_UART_UART4_RX_DMA_CHAN   4
#define STM32_UART_UART4_TX_DMA_STREAM STM32_DMA_STREAM_ID(1, 4)
#define STM32_UART_UART4_TX_DMA_CHAN   4

// Mask of DMA streams which are shared
#define SHARED_DMA_MASK 0


// generated UART DMA configuration lines
#define STM32_UART4_RX_DMA_CONFIG true, STM32_UART_UART4_RX_DMA_STREAM, STM32_UART_UART4_RX_DMA_CHAN
#define STM32_UART4_TX_DMA_CONFIG true, STM32_UART_UART4_TX_DMA_STREAM, STM32_UART_UART4_TX_DMA_CHAN


// generated SPI DMA configuration lines
#define STM32_SPI_SPI1_DMA_STREAMS STM32_SPI_SPI1_TX_DMA_STREAM, STM32_SPI_SPI1_RX_DMA_STREAM
#define HAL_PWM_COUNT 0

// Alarm PWM output config
#define HAL_PWM_ALARM \
        { /* pwmGroup */ \
          0,  /* Timer channel */ \
          { /* PWMConfig */ \
            1000000,    /* PWM clock frequency. */ \
            1000,    /* Initial PWM period 20ms. */ \
            NULL,  /* no callback */ \
            { /* Channel Config */ \
             {PWM_OUTPUT_ACTIVE_HIGH, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}, \
             {PWM_OUTPUT_DISABLED, NULL}  \
            }, \
            0, 0 \
          }, \
          &PWMD2 /* PWMDriver* */ \
        }

// PWM timer config

// PWM output config
#define HAL_PWM_GROUPS 


#ifndef HAL_USE_I2C
#define HAL_USE_I2C FALSE
#endif

// UART configuration
#define HAL_UARTA_DRIVER ChibiOS::UARTDriver uartADriver(0)
#define HAL_UARTB_DRIVER ChibiOS::UARTDriver uartBDriver(1)
#define HAL_UARTC_DRIVER ChibiOS::UARTDriver uartCDriver(2)
#define HAL_UARTD_DRIVER ChibiOS::UARTDriver uartDDriver(3)
#define HAL_UARTE_DRIVER ChibiOS::UARTDriver uartEDriver(4)
#define HAL_UARTF_DRIVER ChibiOS::UARTDriver uartFDriver(5)
#define HAL_UARTG_DRIVER Empty::UARTDriver uartGDriver
#define HAL_UARTH_DRIVER Empty::UARTDriver uartHDriver
#define HAL_UARTI_DRIVER Empty::UARTDriver uartIDriver
#define HAL_UARTJ_DRIVER Empty::UARTDriver uartJDriver
#define HAL_WITH_IO_MCU 0

#define HAL_OTG1_CONFIG {(BaseSequentialStream*) &SDU1, 1, true, false, 0, 0, false, 0, 0, 0}
#define HAL_UART4_CONFIG { (BaseSequentialStream*) &SD4, 4, false, -1, 0, -1, 0, 0}
#define HAL_USART2_CONFIG { (BaseSequentialStream*) &SD2, 2, false, -1, 0, -1, 0, 0}
#define HAL_USART3_CONFIG { (BaseSequentialStream*) &SD3, 3, false, -1, 0, -1, 0, 0}
#define HAL_UART8_CONFIG { (BaseSequentialStream*) &SD8, 8, false, -1, 0, -1, 0, 0}
#define HAL_UART7_CONFIG { (BaseSequentialStream*) &SD7, 7, false, -1, 0, -1, 0, 0}
#define HAL_UART_DEVICE_LIST HAL_OTG1_CONFIG,HAL_UART4_CONFIG,HAL_USART2_CONFIG,HAL_USART3_CONFIG,HAL_UART8_CONFIG,HAL_UART7_CONFIG

#define HAL_UART_NUM_SERIAL_PORTS 6
// USB configuration
#define HAL_USB_VENDOR_ID 0x1209
#define HAL_USB_PRODUCT_ID 0x5741
#define HAL_USB_STRING_MANUFACTURER "ArduPilot"
#define HAL_USB_STRING_PRODUCT "%BOARD%"
#define HAL_USB_STRING_SERIAL "%SERIAL%"


#define HAL_HAVE_AP_ROMFS_EMBEDDED_H 1

/*
* I/O ports initial setup, this configuration is established soon after reset
* in the initialization code.
* Please refer to the STM32 Reference Manual for details.
*/
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/* PORTA:
 PA0 UART4_TX UART4 AF8
 PA1 UART4_RX UART4 AF8
 PA2 BATT_VOLTAGE_SENS ADC1 ADC1_IN2
 PA3 BATT_CURRENT_SENS ADC1 ADC1_IN3
 PA4 VDD_5V_SENS ADC1 ADC1_IN4
 PA5 MPU_CS CS
 PA8 VDD_5V_PERIPH_EN OUTPUT
 PA9 VBUS INPUT
 PA11 OTG_FS_DM OTG1 AF10
 PA12 OTG_FS_DP OTG1 AF10
 PA13 JTMS-SWDIO SWD AF0
 PA14 JTCK-SWCLK SWD AF0
 PA15 TIM2_CH1 TIM2 AF1
 PA18 SPI1_SCK SPI1 AF5
 PA19 SPI1_MISO SPI1 AF10
 PA23 SPI1_MOSI SPI1 AF3
 PA26 MAG_CS CS
*/

#define VAL_GPIOA_MODER   (PIN_MODE_ALTERNATE(0U) | \
                           PIN_MODE_ALTERNATE(1U) | \
                           PIN_MODE_ANALOG(2U) | \
                           PIN_MODE_ANALOG(3U) | \
                           PIN_MODE_ANALOG(4U) | \
                           PIN_MODE_OUTPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U) | \
                           PIN_MODE_OUTPUT(8U) | \
                           PIN_MODE_INPUT(9U) | \
                           PIN_MODE_INPUT(10U) | \
                           PIN_MODE_ALTERNATE(11U) | \
                           PIN_MODE_ALTERNATE(12U) | \
                           PIN_MODE_ALTERNATE(13U) | \
                           PIN_MODE_ALTERNATE(14U) | \
                           PIN_MODE_ALTERNATE(15U) | \
                           PIN_MODE_INPUT(16U) | \
                           PIN_MODE_INPUT(17U) | \
                           PIN_MODE_ALTERNATE(18U) | \
                           PIN_MODE_ALTERNATE(19U) | \
                           PIN_MODE_INPUT(20U) | \
                           PIN_MODE_INPUT(21U) | \
                           PIN_MODE_INPUT(22U) | \
                           PIN_MODE_ALTERNATE(23U) | \
                           PIN_MODE_INPUT(24U) | \
                           PIN_MODE_INPUT(25U) | \
                           PIN_MODE_OUTPUT(26U) | \
                           PIN_MODE_INPUT(27U) | \
                           PIN_MODE_INPUT(28U) | \
                           PIN_MODE_INPUT(29U) | \
                           PIN_MODE_INPUT(30U) | \
                           PIN_MODE_INPUT(31U))

#define VAL_GPIOA_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U) | \
                           PIN_OTYPE_PUSHPULL(8U) | \
                           PIN_OTYPE_OPENDRAIN(9U) | \
                           PIN_OTYPE_PUSHPULL(10U) | \
                           PIN_OTYPE_PUSHPULL(11U) | \
                           PIN_OTYPE_PUSHPULL(12U) | \
                           PIN_OTYPE_PUSHPULL(13U) | \
                           PIN_OTYPE_PUSHPULL(14U) | \
                           PIN_OTYPE_PUSHPULL(15U) | \
                           PIN_OTYPE_PUSHPULL(16U) | \
                           PIN_OTYPE_PUSHPULL(17U) | \
                           PIN_OTYPE_PUSHPULL(18U) | \
                           PIN_OTYPE_PUSHPULL(19U) | \
                           PIN_OTYPE_PUSHPULL(20U) | \
                           PIN_OTYPE_PUSHPULL(21U) | \
                           PIN_OTYPE_PUSHPULL(22U) | \
                           PIN_OTYPE_PUSHPULL(23U) | \
                           PIN_OTYPE_PUSHPULL(24U) | \
                           PIN_OTYPE_PUSHPULL(25U) | \
                           PIN_OTYPE_PUSHPULL(26U) | \
                           PIN_OTYPE_PUSHPULL(27U) | \
                           PIN_OTYPE_PUSHPULL(28U) | \
                           PIN_OTYPE_PUSHPULL(29U) | \
                           PIN_OTYPE_PUSHPULL(30U) | \
                           PIN_OTYPE_PUSHPULL(31U))

#define VAL_GPIOA_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U) | \
                           PIN_OSPEED_MEDIUM(8U) | \
                           PIN_OSPEED_MEDIUM(9U) | \
                           PIN_OSPEED_MEDIUM(10U) | \
                           PIN_OSPEED_MEDIUM(11U) | \
                           PIN_OSPEED_MEDIUM(12U) | \
                           PIN_OSPEED_MEDIUM(13U) | \
                           PIN_OSPEED_MEDIUM(14U) | \
                           PIN_OSPEED_MEDIUM(15U) | \
                           PIN_OSPEED_MEDIUM(16U) | \
                           PIN_OSPEED_MEDIUM(17U) | \
                           PIN_OSPEED_MEDIUM(18U) | \
                           PIN_OSPEED_MEDIUM(19U) | \
                           PIN_OSPEED_MEDIUM(20U) | \
                           PIN_OSPEED_MEDIUM(21U) | \
                           PIN_OSPEED_MEDIUM(22U) | \
                           PIN_OSPEED_MEDIUM(23U) | \
                           PIN_OSPEED_MEDIUM(24U) | \
                           PIN_OSPEED_MEDIUM(25U) | \
                           PIN_OSPEED_MEDIUM(26U) | \
                           PIN_OSPEED_MEDIUM(27U) | \
                           PIN_OSPEED_MEDIUM(28U) | \
                           PIN_OSPEED_MEDIUM(29U) | \
                           PIN_OSPEED_MEDIUM(30U) | \
                           PIN_OSPEED_MEDIUM(31U))

#define VAL_GPIOA_PUPDR   (PIN_PUPDR_PULLUP(0U) | \
                           PIN_PUPDR_PULLUP(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_PULLUP(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_FLOATING(7U) | \
                           PIN_PUPDR_FLOATING(8U) | \
                           PIN_PUPDR_FLOATING(9U) | \
                           PIN_PUPDR_FLOATING(10U) | \
                           PIN_PUPDR_FLOATING(11U) | \
                           PIN_PUPDR_FLOATING(12U) | \
                           PIN_PUPDR_PULLUP(13U) | \
                           PIN_PUPDR_PULLDOWN(14U) | \
                           PIN_PUPDR_FLOATING(15U) | \
                           PIN_PUPDR_FLOATING(16U) | \
                           PIN_PUPDR_FLOATING(17U) | \
                           PIN_PUPDR_FLOATING(18U) | \
                           PIN_PUPDR_FLOATING(19U) | \
                           PIN_PUPDR_FLOATING(20U) | \
                           PIN_PUPDR_FLOATING(21U) | \
                           PIN_PUPDR_FLOATING(22U) | \
                           PIN_PUPDR_FLOATING(23U) | \
                           PIN_PUPDR_FLOATING(24U) | \
                           PIN_PUPDR_FLOATING(25U) | \
                           PIN_PUPDR_PULLUP(26U) | \
                           PIN_PUPDR_FLOATING(27U) | \
                           PIN_PUPDR_FLOATING(28U) | \
                           PIN_PUPDR_FLOATING(29U) | \
                           PIN_PUPDR_FLOATING(30U) | \
                           PIN_PUPDR_FLOATING(31U))

#define VAL_GPIOA_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U) | \
                           PIN_ODR_LOW(8U) | \
                           PIN_ODR_HIGH(9U) | \
                           PIN_ODR_HIGH(10U) | \
                           PIN_ODR_HIGH(11U) | \
                           PIN_ODR_HIGH(12U) | \
                           PIN_ODR_HIGH(13U) | \
                           PIN_ODR_HIGH(14U) | \
                           PIN_ODR_HIGH(15U) | \
                           PIN_ODR_HIGH(16U) | \
                           PIN_ODR_HIGH(17U) | \
                           PIN_ODR_HIGH(18U) | \
                           PIN_ODR_HIGH(19U) | \
                           PIN_ODR_HIGH(20U) | \
                           PIN_ODR_HIGH(21U) | \
                           PIN_ODR_HIGH(22U) | \
                           PIN_ODR_HIGH(23U) | \
                           PIN_ODR_HIGH(24U) | \
                           PIN_ODR_HIGH(25U) | \
                           PIN_ODR_HIGH(26U) | \
                           PIN_ODR_HIGH(27U) | \
                           PIN_ODR_HIGH(28U) | \
                           PIN_ODR_HIGH(29U) | \
                           PIN_ODR_HIGH(30U) | \
                           PIN_ODR_HIGH(31U))

#define VAL_GPIOA_AFRL    (PIN_AFIO_AF(0U, 8U) | \
                           PIN_AFIO_AF(1U, 8U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOA_AFRH    (PIN_AFIO_AF(8U, 0U) | \
                           PIN_AFIO_AF(9U, 0U) | \
                           PIN_AFIO_AF(10U, 0U) | \
                           PIN_AFIO_AF(11U, 10U) | \
                           PIN_AFIO_AF(12U, 10U) | \
                           PIN_AFIO_AF(13U, 0U) | \
                           PIN_AFIO_AF(14U, 0U) | \
                           PIN_AFIO_AF(15U, 1U) | \
                           PIN_AFIO_AF(16U, 0U) | \
                           PIN_AFIO_AF(17U, 0U) | \
                           PIN_AFIO_AF(18U, 5U) | \
                           PIN_AFIO_AF(19U, 10U) | \
                           PIN_AFIO_AF(20U, 0U) | \
                           PIN_AFIO_AF(21U, 0U) | \
                           PIN_AFIO_AF(22U, 0U) | \
                           PIN_AFIO_AF(23U, 3U) | \
                           PIN_AFIO_AF(24U, 0U) | \
                           PIN_AFIO_AF(25U, 0U) | \
                           PIN_AFIO_AF(26U, 0U) | \
                           PIN_AFIO_AF(27U, 0U) | \
                           PIN_AFIO_AF(28U, 0U) | \
                           PIN_AFIO_AF(29U, 0U) | \
                           PIN_AFIO_AF(30U, 0U) | \
                           PIN_AFIO_AF(31U, 0U))

/* PORTB:
 PB0 EXTERN_GPIO1 OUTPUT
 PB1 EXTERN_GPIO2 OUTPUT
 PB2 BOOT1 INPUT
 PB3 FMU_SW0 INPUT
 PB5 VDD_BRICK_nVALID INPUT
 PB7 VDD_BRICK2_nVALID INPUT
*/

#define VAL_GPIOB_MODER   (PIN_MODE_OUTPUT(0U) | \
                           PIN_MODE_OUTPUT(1U) | \
                           PIN_MODE_INPUT(2U) | \
                           PIN_MODE_INPUT(3U) | \
                           PIN_MODE_INPUT(4U) | \
                           PIN_MODE_INPUT(5U) | \
                           PIN_MODE_INPUT(6U) | \
                           PIN_MODE_INPUT(7U))

#define VAL_GPIOB_OTYPER  (PIN_OTYPE_PUSHPULL(0U) | \
                           PIN_OTYPE_PUSHPULL(1U) | \
                           PIN_OTYPE_PUSHPULL(2U) | \
                           PIN_OTYPE_PUSHPULL(3U) | \
                           PIN_OTYPE_PUSHPULL(4U) | \
                           PIN_OTYPE_PUSHPULL(5U) | \
                           PIN_OTYPE_PUSHPULL(6U) | \
                           PIN_OTYPE_PUSHPULL(7U))

#define VAL_GPIOB_OSPEEDR (PIN_OSPEED_MEDIUM(0U) | \
                           PIN_OSPEED_MEDIUM(1U) | \
                           PIN_OSPEED_MEDIUM(2U) | \
                           PIN_OSPEED_MEDIUM(3U) | \
                           PIN_OSPEED_MEDIUM(4U) | \
                           PIN_OSPEED_MEDIUM(5U) | \
                           PIN_OSPEED_MEDIUM(6U) | \
                           PIN_OSPEED_MEDIUM(7U))

#define VAL_GPIOB_PUPDR   (PIN_PUPDR_FLOATING(0U) | \
                           PIN_PUPDR_FLOATING(1U) | \
                           PIN_PUPDR_FLOATING(2U) | \
                           PIN_PUPDR_FLOATING(3U) | \
                           PIN_PUPDR_FLOATING(4U) | \
                           PIN_PUPDR_PULLUP(5U) | \
                           PIN_PUPDR_FLOATING(6U) | \
                           PIN_PUPDR_PULLUP(7U))

#define VAL_GPIOB_ODR     (PIN_ODR_HIGH(0U) | \
                           PIN_ODR_HIGH(1U) | \
                           PIN_ODR_HIGH(2U) | \
                           PIN_ODR_HIGH(3U) | \
                           PIN_ODR_HIGH(4U) | \
                           PIN_ODR_HIGH(5U) | \
                           PIN_ODR_HIGH(6U) | \
                           PIN_ODR_HIGH(7U))

#define VAL_GPIOB_AFRL    (PIN_AFIO_AF(0U, 0U) | \
                           PIN_AFIO_AF(1U, 0U) | \
                           PIN_AFIO_AF(2U, 0U) | \
                           PIN_AFIO_AF(3U, 0U) | \
                           PIN_AFIO_AF(4U, 0U) | \
                           PIN_AFIO_AF(5U, 0U) | \
                           PIN_AFIO_AF(6U, 0U) | \
                           PIN_AFIO_AF(7U, 0U))

#define VAL_GPIOB_AFRH    (0)


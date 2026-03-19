/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "hal.h"
#include "usbcfg.h"
#include "stm32_util.h"
#include "flash.h"
#include "watchdog.h"
#include "board.h"

#if defined(PIC02) || defined(RP2350) || PIC02_AVAILABLE
#define STM32_AVAILABLE FALSE 
#endif

/*
 * RP2350 (Pico2) note on XIP cache coherency after SWD flashing:
 *
 * When we program flash via SWD/OpenOCD then issue a SYSRESETREQ reset, the
 * RP2350 XIP cache can retain stale lines across the reset. If those stale
 * lines include the vector table, an external IRQ can dispatch to the *old*
 * handler address even though flash now contains the correct ISR entry.
 *
 * This shows up as (for example) I2C1 IRQ37 repeatedly landing in the default
 * DebugMon/_unhandled_exception handler, even though VectorD4 is present and
 * correctly placed in the vector table in flash.
 *
 * The ChibiOS RP2350 EFL driver invalidates the XIP cache by writing through
 * the XIP "maintenance" alias then re-enabling the cache. We reuse that same
 * mechanism here, but we must do it after C runtime init so the helper can
 * execute from RAM (the .ramtext section is copied into RAM as part of .data).
 */
#if PIC02_AVAILABLE == TRUE
#define RP2350_RAMFUNC __attribute__((noinline, section(".ramtext")))
static RP2350_RAMFUNC void rp2350_invalidate_xip_cache(void)
{
    /* Constants mirror modules/ChibiOS/os/hal/ports/RP/RP2350/hal_efl_lld.c */
    volatile uint8_t *maint = (volatile uint8_t *)0x18000000U;   /* RP_XIP_MAINTENANCE_BASE */
    volatile uint32_t *xip = (volatile uint32_t *)0x400C8000U;    /* RP_XIP_CTRL_BASE */

    /* 16KB cache, 8-byte cache line size. Writing any value invalidates the line. */
    for (uint32_t offset = 0U; offset < (16U * 1024U); offset += 8U) {
        maint[offset] = 0U;
    }

    __DSB();
    __ISB();

    /* Re-enable both Secure and Non-secure cache views. */
    xip[0] = (1U << 0) | (1U << 1); /* EN_SECURE | EN_NONSECURE */
}
#endif // PIC02_AVAILABLE

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#if STM32_AVAILABLE == TRUE
    /**
    * @brief   STM32 GPIO static initialization data.
    */
    #if defined(STM32F100_MCUCONF) || defined(STM32F103_MCUCONF) || defined(STM32F105_MCUCONF)
        const PALConfig pal_default_config =
        {
          {VAL_GPIOA_ODR, VAL_GPIOA_CRL, VAL_GPIOA_CRH},
          {VAL_GPIOB_ODR, VAL_GPIOB_CRL, VAL_GPIOB_CRH},
          {VAL_GPIOC_ODR, VAL_GPIOC_CRL, VAL_GPIOC_CRH},
          {VAL_GPIOD_ODR, VAL_GPIOD_CRL, VAL_GPIOD_CRH},
          {VAL_GPIOE_ODR, VAL_GPIOE_CRL, VAL_GPIOE_CRH},
        };

    #else //Other than STM32F1/F3 series, and excluding non-stm32 now too.

        /**
        * @brief   Type of STM32 GPIO port setup.
        */
        typedef struct {
          uint32_t              moder;
          uint32_t              otyper;
          uint32_t              ospeedr;
          uint32_t              pupdr;
          uint32_t              odr;
          uint32_t              afrl;
          uint32_t              afrh;
        } gpio_setup_t;

        /**
        * @brief   Type of STM32 GPIO initialization data.
        */
        typedef struct {
            #if STM32_HAS_GPIOA || defined(__DOXYGEN__)
              gpio_setup_t          PAData;
            #endif
            #if STM32_HAS_GPIOB || defined(__DOXYGEN__)
              gpio_setup_t          PBData;
            #endif
            #if STM32_HAS_GPIOC || defined(__DOXYGEN__)
              gpio_setup_t          PCData;
            #endif
            #if STM32_HAS_GPIOD || defined(__DOXYGEN__)
              gpio_setup_t          PDData;
            #endif
            #if STM32_HAS_GPIOE || defined(__DOXYGEN__)
              gpio_setup_t          PEData;
            #endif
            #if STM32_HAS_GPIOF || defined(__DOXYGEN__)
              gpio_setup_t          PFData;
            #endif
            #if STM32_HAS_GPIOG || defined(__DOXYGEN__)
              gpio_setup_t          PGData;
            #endif
            #if STM32_HAS_GPIOH || defined(__DOXYGEN__)
              gpio_setup_t          PHData;
            #endif
            #if STM32_HAS_GPIOI || defined(__DOXYGEN__)
              gpio_setup_t          PIData;
            #endif
            #if STM32_HAS_GPIOJ || defined(__DOXYGEN__)
              gpio_setup_t          PJData;
            #endif
            #if STM32_HAS_GPIOK || defined(__DOXYGEN__)
              gpio_setup_t          PKData;
            #endif
        } gpio_config_t;

        /**
        * @brief   STM32 GPIO static initialization data.
        */
        static const gpio_config_t gpio_default_config = {
            #if STM32_HAS_GPIOA
              {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR,
              VAL_GPIOA_ODR,   VAL_GPIOA_AFRL,   VAL_GPIOA_AFRH},
            #endif
            #if STM32_HAS_GPIOB
              {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR,
              VAL_GPIOB_ODR,   VAL_GPIOB_AFRL,   VAL_GPIOB_AFRH},
            #endif
            #if STM32_HAS_GPIOC
              {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR,
              VAL_GPIOC_ODR,   VAL_GPIOC_AFRL,   VAL_GPIOC_AFRH},
            #endif
            #if STM32_HAS_GPIOD
              {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR,
              VAL_GPIOD_ODR,   VAL_GPIOD_AFRL,   VAL_GPIOD_AFRH},
            #endif
            #if STM32_HAS_GPIOE
              {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR,
              VAL_GPIOE_ODR,   VAL_GPIOE_AFRL,   VAL_GPIOE_AFRH},
            #endif
            #if STM32_HAS_GPIOF
              {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR,
              VAL_GPIOF_ODR,   VAL_GPIOF_AFRL,   VAL_GPIOF_AFRH},
            #endif
            #if STM32_HAS_GPIOG
              {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR,
              VAL_GPIOG_ODR,   VAL_GPIOG_AFRL,   VAL_GPIOG_AFRH},
            #endif
            #if STM32_HAS_GPIOH
              {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR,
              VAL_GPIOH_ODR,   VAL_GPIOH_AFRL,   VAL_GPIOH_AFRH},
            #endif
            #if STM32_HAS_GPIOI
              {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR,
              VAL_GPIOI_ODR,   VAL_GPIOI_AFRL,   VAL_GPIOI_AFRH},
            #endif
            #if STM32_HAS_GPIOJ
              {VAL_GPIOJ_MODER, VAL_GPIOJ_OTYPER, VAL_GPIOJ_OSPEEDR, VAL_GPIOJ_PUPDR,
              VAL_GPIOJ_ODR,   VAL_GPIOJ_AFRL,   VAL_GPIOJ_AFRH},
            #endif
            #if STM32_HAS_GPIOK
              {VAL_GPIOK_MODER, VAL_GPIOK_OTYPER, VAL_GPIOK_OSPEEDR, VAL_GPIOK_PUPDR,
              VAL_GPIOK_ODR,   VAL_GPIOK_AFRL,   VAL_GPIOK_AFRH}
            #endif
        };
#endif //#if STM32_AVAILABLE

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
#if STM32_AVAILABLE == TRUE
    static void gpio_init(stm32_gpio_t *gpiop, const gpio_setup_t *config) {

      gpiop->OTYPER  = config->otyper;
      gpiop->OSPEEDR = config->ospeedr;
      gpiop->PUPDR   = config->pupdr;
      gpiop->ODR     = config->odr;
      gpiop->AFRL    = config->afrl;
      gpiop->AFRH    = config->afrh;
      gpiop->MODER   = config->moder;
    }
#endif

#if STM32_AVAILABLE == TRUE
    static void stm32_gpio_init(void) {

      /* Enabling GPIO-related clocks, the mask comes from the
        registry header file.*/
    #if defined(STM32H7)
    #if !EXT_FLASH_SIZE_MB // if we have external flash resetting GPIO might disable all comms with it
      rccResetAHB4(STM32_GPIO_EN_MASK);
    #endif
      rccEnableAHB4(STM32_GPIO_EN_MASK, true);
    #elif defined(STM32F3)
      rccResetAHB(STM32_GPIO_EN_MASK);
      rccEnableAHB(STM32_GPIO_EN_MASK, true);
    #elif defined(STM32G4) || defined(STM32L4) || defined(STM32L4PLUS)
      rccResetAHB2(STM32_GPIO_EN_MASK);
      rccEnableAHB2(STM32_GPIO_EN_MASK, true);
    #else
      rccResetAHB1(STM32_GPIO_EN_MASK);
      rccEnableAHB1(STM32_GPIO_EN_MASK, true);
    #endif

      /* Initializing all the defined GPIO ports.*/
    #if STM32_HAS_GPIOA
      gpio_init(GPIOA, &gpio_default_config.PAData);
    #endif
    #if STM32_HAS_GPIOB
      gpio_init(GPIOB, &gpio_default_config.PBData);
    #endif
    #if STM32_HAS_GPIOC
      gpio_init(GPIOC, &gpio_default_config.PCData);
    #endif
    #if STM32_HAS_GPIOD
      gpio_init(GPIOD, &gpio_default_config.PDData);
    #endif
    #if STM32_HAS_GPIOE
      gpio_init(GPIOE, &gpio_default_config.PEData);
    #endif
    #if STM32_HAS_GPIOF
      gpio_init(GPIOF, &gpio_default_config.PFData);
    #endif
    #if STM32_HAS_GPIOG
      gpio_init(GPIOG, &gpio_default_config.PGData);
    #endif
    #if STM32_HAS_GPIOH
      gpio_init(GPIOH, &gpio_default_config.PHData);
    #endif
    #if STM32_HAS_GPIOI
      gpio_init(GPIOI, &gpio_default_config.PIData);
    #endif
    #if STM32_HAS_GPIOJ
      gpio_init(GPIOJ, &gpio_default_config.PJData);
    #endif
    #if STM32_HAS_GPIOK
      gpio_init(GPIOK, &gpio_default_config.PKData);
    #endif
    }
#endif //#if STM32_AVAILABLE

#endif //!STM32F100_MCUCONF

#if PIC02_AVAILABLE == TRUE
void pico2_gpio_init(void) {
    /* Configure PWM GPIO pins to PWM alternate function (function 4).
     * HAL_PWM_GPIO_LINES is generated by chibios_hwdef.py when PWM pins are defined.
     * Use a local array so PAL_LINE (comma-operator form) is allowed as an initializer. */
    #if defined(HAL_PWM_GPIO_LINES)
    ioline_t lines[] = {HAL_PWM_GPIO_LINES};
    for (unsigned i = 0; i < sizeof(lines)/sizeof(lines[0]); i++) {
        palSetLineMode(lines[i], PAL_MODE_ALTERNATE_PWM);
    }
    #endif
}
#endif //PIC02_AVAILABLE

/**
 * @brief   Early initialization code.
 * @details This initialization must be performed just after stack setup
 *          and before any other initialization.
 *
 * You must not rely on: 1) BSS variables being cleared 2) DATA Variables being initialized 3) RAM functions to be in RAM
 * You can rely on: 1) const variables or tables 2) flash code 3) automatic variables
 */
void __early_init(void) {
    #if STM32_AVAILABLE == TRUE && !defined(STM32F1)
      stm32_gpio_init();
    #endif
    #if STM32_AVAILABLE == TRUE && !defined(HAL_XIP_ENABLED) || defined(HAL_FORCE_CLOCK_INIT)
      // if running from external flash then the clocks must not be reset - instead rely on the bootloader to setup
      stm32_clock_init();
    #endif
    #if defined(HAL_DISABLE_DCACHE)
      SCB_DisableDCache();
    #endif
    #if defined(STM32H7)
      // ensure ITCM and DTCM are enabled. These could be disabled by the px4
      // bootloader
      SCB->ITCMCR |= 1; // ITCM enable
      SCB->DTCMCR |= 1; // DTCM enable

        #ifdef STM32_NOCACHE_MPU_REGION_1
          // disable cache on configured regions so they can be used for DMA
          // this requires some coordination with the memory map in the MCU configuration script
          mpuConfigureRegion(STM32_NOCACHE_MPU_REGION_1,
                            STM32_NOCACHE_MPU_REGION_1_BASE,
                            MPU_RASR_ATTR_AP_RW_RW |
                            MPU_RASR_ATTR_NON_CACHEABLE |
                            STM32_NOCACHE_MPU_REGION_1_SIZE |
                            MPU_RASR_ENABLE);
        #endif
        #ifdef STM32_NOCACHE_MPU_REGION_2
          mpuConfigureRegion(STM32_NOCACHE_MPU_REGION_2,
                            STM32_NOCACHE_MPU_REGION_2_BASE,
                            MPU_RASR_ATTR_AP_RW_RW |
                            MPU_RASR_ATTR_NON_CACHEABLE |
                            STM32_NOCACHE_MPU_REGION_2_SIZE |
                            MPU_RASR_ENABLE);
        #endif
      #if defined(DUAL_CORE)
        stm32_disable_cm4_core(); // disable second core
      #endif
    #endif
    #if PIC02_AVAILABLE == TRUE
      /*
       * RP2350 note (Pico2/ArduPilot bootloader):
       * The application vector table lives at __vectors_base__ in flash
       * (typically app_flash_base + 0x80). If we don't explicitly point
       * VTOR at that table then external IRQs (for example I2C1 IRQ37)
       * are taken via the boot ROM vector table and can end up in the
       * default _unhandled_exception handler even though the real ISR
       * (for example VectorD4 for I2C1) is linked into the image.
       *
       * Set VTOR as early as possible so all peripheral IRQs land in the
       * correct ChibiOS handlers.
       */
      extern uint32_t __vectors_base__;

      SCB->VTOR = (uint32_t)&__vectors_base__;
      __DSB();
      __ISB();

      // pico2 specific early init can go here
      pico2_gpio_init();
    #endif
}

void __late_init(void) {

#if PIC02_AVAILABLE == TRUE
  /*
   * The bootloader leaves USB running (NVIC enabled, controller active).
   * Before halInit() zeroes the USB driver struct (USBD1.config=NULL),
   * we must prevent the USB ISR from firing, otherwise the first SOF
   * from the host hits usb_lld_serve_interrupt() with a NULL config
   * pointer, which reads ROM[12] (the ROM HardFault vector) and jumps
   * there, ending in the ROM debug BKPT trap at 0x000002f8.
   *
   * Fix: disable the USB NVIC IRQ and hold USB in peripheral reset
   * right here. usb_lld_start() will unreset and re-enable it later.
   */
  nvicDisableVector(RP_USBCTRL_IRQ_NUMBER);
  hal_lld_peripheral_reset(RESETS_ALLREG_USBCTRL);

  /*
   * Ensure the XIP cache does not hold stale vector table lines (or other
   * early-startup code) when the firmware was reflashed via SWD and reset
   * using SYSRESETREQ.
   *
   * This must run after the C runtime has initialized .data, because the
   * helper is located in .ramtext and executes from RAM.
   */
  rp2350_invalidate_xip_cache();

  /*
   * Relocate the vector table into SRAM.
   *
   * On the RP2350 we have observed external IRQs occasionally dispatching to
   * stale/default handlers after SWD reflashing + SYSRESETREQ reset, despite
   * the flash vector table containing the correct ISR entry.
   *
   * Moving the vector table into SRAM makes IRQ dispatch deterministic and
   * avoids any dependency on XIP flash caching behavior for vector fetches.
   *
   * The VTOR alignment requirement for Cortex-M33 is 128 bytes (TBLOFF[6:0]),
   * so we align the RAM vector table accordingly.
   *
   * The RP2350 port defines CORTEX_NUM_VECTORS=56 (see devices/RP2350/cmparams.h),
   * so the full table is 16 system exceptions + 56 external IRQ vectors.
   */
  extern uint32_t __vectors_base__;
  enum { RP2350_VECTOR_WORDS = 16U + 56U };
  static uint32_t rp2350_vectors[RP2350_VECTOR_WORDS] __attribute__((aligned(128)));
  const uint32_t *flash_vectors = (const uint32_t *)&__vectors_base__;

  for (uint32_t i = 0; i < RP2350_VECTOR_WORDS; i++) {
      rp2350_vectors[i] = flash_vectors[i];
  }

  SCB->VTOR = (uint32_t)rp2350_vectors;
  __DSB();
  __ISB();

  /*
   * RP2350 UART RX pads: at chip reset PADS_BANK0 has ISO=1 + PDE=1 + IE=0.
   * Setting 0x5A (PUE+IE+SCHMITT) prevents any early glitch on the RX lines
   * before UARTDriver::_begin() runs.  Neither halInit() / pal_lld_init()
   * nor sio_lld_start() configure per-pin GPIO CTRL or PADS registers; the
   * actual FUNCSEL=2 (UART) and permanent pull-up are applied later in
   * UARTDriver::_begin() via palSetLineMode(PAL_MODE_ALTERNATE_UART) followed
   * by palLineSetPushPull(PULLUP) before sioStart().
   */
  PADS_BANK0->GPIO[13] = 0x5AU;  /* GPIO13 = UART0_RX — early PUE+IE+SCHMITT */
  PADS_BANK0->GPIO[11] = 0x5AU;  /* GPIO11 = UART1_RX — early PUE+IE+SCHMITT */
#endif

  halInit();

#ifdef HAL_USB_PRODUCT_ID
  /*
   * Populate USB string descriptors AFTER halInit() so HAL state is fully
   * set up before we access OTP (serial number) and write vcom_strings[].
   * Must happen BEFORE chSysInit() starts the USB driver so the host
   * receives valid string descriptors during the first enumeration attempt.
   * This mirrors the approach used in the bootloader __late_init().
   */
  setup_usb_strings();
#endif

  chSysInit();

  /*
   * Start EFL driver for RP2350 QSPI flash read/write/erase support.
   * Must be called after halInit() which runs eflInit().
   */
  #if PIC02_AVAILABLE == TRUE && HAL_USE_EFL == TRUE
    eflStart(&EFLD1, NULL);
  #endif

  /*
   * Initialize RNG
   */
  #if HAL_USE_HW_RNG && defined(RNG)
    rccEnableAHB2(RCC_AHB2ENR_RNGEN, 0);
    RNG->CR |= RNG_CR_IE;
    RNG->CR |= RNG_CR_RNGEN;
  #endif

  #if STM32_AVAILABLE == TRUE
    stm32_watchdog_save_reason();
    #ifndef HAL_BOOTLOADER_BUILD
      stm32_watchdog_clear_reason();
    #endif
  #endif
  #if CH_CFG_USE_HEAP == TRUE
    malloc_init();
  #endif

  #if defined(STM32_AVAILABLE) && defined(HAL_FLASH_SET_NRST_MODE)
    // ensure NRST_MODE is set correctly
    stm32_flash_set_NRST_MODE(HAL_FLASH_SET_NRST_MODE);
  #endif
}

#if HAL_USE_SDC || defined(__DOXYGEN__)
    /**
    * @brief   SDC card detection.
    */
    bool sdc_lld_is_card_inserted(SDCDriver *sdcp) {
        (void)sdcp;
        return true;
    }

    /**
    * @brief   SDC card write protection detection.
    */
    bool sdc_lld_is_write_protected(SDCDriver *sdcp) {

      (void)sdcp;
      return false;
    }
#endif /* HAL_USE_SDC */

#if HAL_USE_MMC_SPI || defined(__DOXYGEN__)
    /**
    * @brief   MMC_SPI card detection.
    */
    bool mmc_lld_is_card_inserted(MMCDriver *mmcp) {
      (void)mmcp;
      /* TODO: Fill the implementation.*/
      return true;
    }

    /**
    * @brief   MMC_SPI card write protection detection.
    */
    bool mmc_lld_is_write_protected(MMCDriver *mmcp) {
      (void)mmcp;
      /* TODO: Fill the implementation.*/
      return false;
    }
#endif

/**
 * @brief   Board-specific initialization code.
 * @todo    Add your board-specific code, if any.
 */
void boardInit(void) {
  HAL_BOARD_INIT_HOOK_CALL;
}

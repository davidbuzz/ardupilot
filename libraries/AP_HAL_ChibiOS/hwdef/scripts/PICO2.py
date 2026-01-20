#!/usr/bin/env python3

# derived from 405

# flake8: noqa
'''
these tables are NOT generated from the STM32 datasheets for theSTM32F40x
'''

# additional build information for ChibiOS
build = {
    "CHIBIOS_STARTUP_MK"  : "os/common/startup/ARMCMx/compilers/GCC/mk/startup_rp2350.mk",
    "CHIBIOS_PLATFORM_MK" : "os/hal/ports/RP/RP2350/platform.mk"
    }

# 2350 official / nominal core speed to 150Mhz, but its trivially overclockable to 168Mhz like the F4 series

# a 4M onboard flash:
# #define PICO_FLASH_SIZE_BYTES (4 * 1024 * 1024)
# a 16M onboard flash:
# #define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)

# 2.2 Address map - Table 8. Address Map  /    explain
# Bus Segment 	 / Base Address
# ROM 				0x00000000
# XIP 				0x10000000            / eXecute In Place from fast nor flash,  and XIP cache, saves ram, but slower than internal memory. upto 16mb
# SRAM 				0x20000000            / 520k sram
# APB Peripherals 	0x40000000
# AHB Peripherals 	0x50000000
# Core-local Peripherals (SIO) 0xd0000000
# Cortex-M33 private registers 0xe0000000


# MCU parameters
mcu = {
    # ram map, as list of (address, size-kb, flags)
    # flags of 1 means DMA-capable
    # flags of 2 means faster memory for CPU intensive work
    'RAM_MAP' : [
        # ROM 				0x00000000
        # XIP 				0x10000000
        (0x20000000, 512, 1), # main memory, DMA safe, 512k
    ],

    'EXPECTED_CLOCK' : 150000000,  # 168000000 ?

    'DEFINES' : {
        'STM32F4' : '1',
    }

}

#     Table 1427
# Name QFN-60-Number QFN-80-Number Type Power-Domain Reset-State Description
# GPIO0 2 77 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO1 3 78 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO2 4 79 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO3 5 80 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO4 7 1 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO5 8 2 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO6 9 3 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO7 10 4 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO8 12 6 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO9 13 7 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO10 14 8 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO11 15 9 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO12 16 11 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO13 17 12 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO14 18 13 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO15 19 14 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO16 27 16 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO17 28 17 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO18 29 18 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO19 31 19 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO20 32 20 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO21 33 21 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO22 34 22 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO23 35 23 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO24 36 25 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO25 37 26 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO26_ADC0 40 - Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input
# GPIO27_ADC1 41 - Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input
# GPIO28_ADC2 42 - Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input
# GPIO29_ADC3 43 - Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input
# GPIO26 - 27 Digital-IO-(FT) IOVDD Pull-Down User-IO   gpio26-39 are qfn-80 only
# GPIO27 - 28 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO28 - 36 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO29 - 37 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO30 - 38 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO31 - 39 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO32 - 40 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO33 - 42 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO34 - 43 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO35 - 44 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO36 - 45 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO37 - 46 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO38 - 47 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO39 - 48 Digital-IO-(FT) IOVDD Pull-Down User-IO
# GPIO40_ADC0 - 49 Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input  qfn-80 only from here down
# GPIO41_ADC1 - 52 Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input  
# GPIO42_ADC2 - 53 Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input 
# GPIO43_ADC3 - 54 Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input
# GPIO44_ADC4 - 55 Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input
# GPIO45_ADC5 - 56 Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input
# GPIO46_ADC6 - 57 Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input
# GPIO47_ADC7 - 58 Digital-IO/Analogue IOVDD/ADC_AVDD Pull-Down User-IO-or-ADC-input

# Table 1428 
# QSPI_SD3 55 70 Digital-IO QSPI_IOVDD Pull-Up QSPI_data
# QSPI_SCLK 56 71 Digital-IO QSPI_IOVDD Pull-Down QSPI_clock
# QSPI_SD0 57 72 Digital-IO QSPI_IOVDD Pull-Down QSPI_data
# QSPI_SD2 58 73 Digital-IO QSPI_IOVDD Pull-Up QSPI_data
# QSPI_SD1 59 74 Digital-IO QSPI_IOVDD Pull-Down QSPI_data
# QSPI_SS 60 75 Digital-IO QSPI_IOVDD Pull-Up QSPI_chip_select/USB_BOOTSEL

# Table 1429. Crystal oscillator pins 
# Name QFN-60-Number QFN-80-Number Type Power-Domain Description
# XIN 21 30 Analogue (XOSC) IOVDD Crystal oscillator. XIN may also be driven by a square wave.
# XOUT 22 31 Analogue (XOSC) IOVDD Crystal oscillator.

# Table 1430. Miscellaneous pins 
# Name QFN-60-Number QFN-80-Number Type Power-Domain Reset-State Description
# RUN 26 35 Digital-In-(FT) IOVDD Pull-Up Chip enable / reset_n
# SWCLK 24 33 Digital-In-(FT) IOVDD Pull-Up Serial_Wire_Debug_clock
# SWDIO 25 34 Digital-IO-(FT) IOVDD Pull-Up Serial_Wire_Debug_data

# Table 1431. USB pins 
# Name QFN-60-Number QFN-80-Number Type Power-Domain Description
# USB_DP 52 67 USB IO USB_OTP_VDD USB Data +ve.
# USB_DM 51 66 USB IO USB_OTP_VDD USB Data -ve. 

# Table 1432. Power supply pins 
# Name QFN-60-Number(s) QFN-80-Number(s) Description
# DVDD 6, 23, 39 10, 32, 51 Core supply
# IOVDD 11, 20, 30, 38, 45, 54 5, 15, 24, 29, 41, 50, 60, 76 IO supply
# QSPI_IOVDD 54 69 QSPI IO supply
# USB_OTP_VDD 53 68 USB & OTP supply
# ADC_AVDD 44 59 ADC supply
# VREG_AVDD 46 61 Voltage regulator analogue supply
# VREG_PGND 47 62 Voltage regulator ground
# VREG_LX 48 63 Voltage regulator switching output (connect to inductor)
# VREG_VIN 49 64 Voltage regulator input supply
# VREG_FB 50 65 Voltage regulator feedback input
# GND - - Ground connection via central exposed pad

# we'll commect the normal pin names to the alternate function map too.
AltFunction_map = {
}
	
# q60 uses the QFN-60-Number - these have weird offsets, do not "fix", this is how they are numbered in the datasheet for realz.
regular_pins_q60 = {
"GPIO0" : 2, "GPIO1" : 3,"GPIO2" : 4,"GPIO3" : 5,"GPIO4" : 7,"GPIO5" : 8,"GPIO6" : 9,"GPIO7" : 10,"GPIO8" : 12,"GPIO9" : 13,"GPIO10" : 14,
"GPIO11" : 15,"GPIO12" : 16,"GPIO13" : 17,"GPIO14" : 18,"GPIO15" : 19,"GPIO16" : 27,"GPIO17" : 28,"GPIO18" : 29,
"GPIO19" : 31,"GPIO20" : 32,"GPIO21" : 33,"GPIO22" : 34,"GPIO23" : 35,"GPIO24" : 36,"GPIO25" : 37,
"GPIO26_ADC0" : 40,"GPIO27_ADC1" : 41,"GPIO28_ADC2" : 42,"GPIO29_ADC3" : 43,
"QSPI_SD3" : 55,"QSPI_SCLK" : 56,"QSPI_SD0" : 57,"QSPI_SD2" : 58,"QSPI_SD1" : 59,"QSPI_SS" : 60,
"XIN" : 21,"XOUT" : 22,
"RUN" : 26,"SWCLK" : 24,"SWDIO" : 25,
"USB_DP" : 52,"USB_DM" : 51,
}
for k,v in regular_pins_q60.items():
	AltFunction_map[f"P{v}:{k}"] = v+100  # offset by 100 to avoid clashes


 # q80 uses the QFN-80-Number - these have weird offsets, do not "fix", this is how they are numbered in the datasheet for realz.
regular_pins_q80 = {
"GPIO0" : 77,"GPIO1" : 78,"GPIO2" : 79,"GPIO3" : 80,"GPIO4" : 1,"GPIO5" : 2,"GPIO6" : 3,"GPIO7" : 4,"GPIO8" : 6,"GPIO9" : 7,"GPIO10" : 8,
"GPIO11" : 9,"GPIO12" : 11,"GPIO13" : 12,"GPIO14" : 13,"GPIO15" : 14,"GPIO16" : 16,"GPIO17" : 17,"GPIO18" : 18,		
"GPIO19" : 19,"GPIO20" : 20,"GPIO21" : 21,"GPIO22" : 22,"GPIO23" : 23,"GPIO24" : 25,"GPIO25" : 26, "GPIO26" : 27,
"GPIO27" : 28,"GPIO28" : 36,"GPIO29" : 37,"GPIO30" : 38,"GPIO31" : 39,"GPIO32" : 40,"GPIO33" : 42,
"GPIO34" : 43,"GPIO35" : 44,"GPIO36" : 45,"GPIO37" : 46,"GPIO38" : 47,"GPIO39" : 48,
"GPIO40_ADC0" : 49,"GPIO41_ADC1" : 52,"GPIO42_ADC2" : 53,"GPIO43_ADC3" : 54,"GPIO44_ADC4" : 55,"GPIO45_ADC5" : 56,"GPIO46_ADC6" : 57,"GPIO47_ADC7" : 58,
"QSPI_SD3" : 70,"QSPI_SCLK" : 71,"QSPI_SD0" : 72,"QSPI_SD2" : 73,"QSPI_SD1" : 74,"QSPI_SS" : 75,
"XIN" : 30,"XOUT" : 31,
"RUN" : 35,"SWCLK" : 33,"SWDIO" : 34,
"USB_DP" : 67,"USB_DM" : 66,
}
for k,v in regular_pins_q80.items():
	AltFunction_map[f"P{v}:{k}"] = v+200  # offset by 200 to avoid clashes

# 
#1.2.3. GPIO functions (Bank 0)
_AltFunction_map = {
	# format is PIN:FUNCTION : AFNUM
	# extracted from datasheet with human and convert_rp2350_gpio.py not 100% automated, but accurate. 
    # buzz made 'SPI1_TX' equivalent to 'SPI1_MOSI' etc for consistency with stm32 naming and _RX => _MISO
"P0:I2C0_SDA"	:	3,
"P0:PIO0"	:	6,
"P0:PIO1"	:	7,
"P0:PIO2"	:	8,
"P0:PWM0_A"	:	4,
"P0:QMI_CS1n"	:	9,
"P0:SIO"	:	5,
"P0:SPI0_RX"	:	1,
"P0:UART0_TX"	:	2,
"P0:USB_OVCUR_DET"	:	10,
"P10:I2C1_SDA"	:	3,
"P10:PIO0"	:	6,
"P10:PIO1"	:	7,
"P10:PIO2"	:	8,
"P10:PWM5_A"	:	4,
"P10:SIO"	:	5,
"P10:SPI1_SCK"	:	1,
"P10:UART1_CTS"	:	2,
"P10:UART1_TX"	:	11,
"P10:USB_VBUS_DET"	:	10,
"P11:I2C1_SCL"	:	3,
"P11:PIO0"	:	6,
"P11:PIO1"	:	7,
"P11:PIO2"	:	8,
"P11:PWM5_B"	:	4,
"P11:SIO"	:	5,
"P11:SPI1_TX"	:	1,
"P11:SPI1_MOSI"	:	1,
"P11:UART1_RTS"	:	2,
"P11:UART1_RX"	:	11,
"P11:USB_VBUS_EN"	:	10,
"P12:CLOCK_GPIN0"	:	9,
"P12:HSTX"	:	0,
"P12:I2C0_SDA"	:	3,
"P12:PIO0"	:	6,
"P12:PIO1"	:	7,
"P12:PIO2"	:	8,
"P12:PWM6_A"	:	4,
"P12:SIO"	:	5,
"P12:SPI1_RX"	:	1,
"P12:SPI1_MISO"	:	1,
"P12:UART0_TX"	:	2,
"P12:USB_OVCUR_DET"	:	10,
"P13:CLOCK_GPOUT0"	:	9,
"P13:HSTX"	:	0,
"P13:I2C0_SCL"	:	3,
"P13:PIO0"	:	6,
"P13:PIO1"	:	7,
"P13:PIO2"	:	8,
"P13:PWM6_B"	:	4,
"P13:SIO"	:	5,
"P13:SPI1_CSn"	:	1,
"P13:UART0_RX"	:	2,
"P13:USB_VBUS_DET"	:	10,
"P14:CLOCK_GPIN1"	:	9,
"P14:HSTX"	:	0,
"P14:I2C1_SDA"	:	3,
"P14:PIO0"	:	6,
"P14:PIO1"	:	7,
"P14:PIO2"	:	8,
"P14:PWM7_A"	:	4,
"P14:SIO"	:	5,
"P14:SPI1_SCK"	:	1,
"P14:UART0_CTS"	:	2,
"P14:UART0_TX"	:	11,
"P14:USB_VBUS_EN"	:	10,
"P15:CLOCK_GPOUT1"	:	9,
"P15:HSTX"	:	0,
"P15:I2C1_SCL"	:	3,
"P15:PIO0"	:	6,
"P15:PIO1"	:	7,
"P15:PIO2"	:	8,
"P15:PWM7_B"	:	4,
"P15:SIO"	:	5,
"P15:SPI1_TX"	:	1,
"P15:SPI1_MOSI"	:	1,
"P15:UART0_RTS"	:	2,
"P15:UART0_RX"	:	11,
"P15:USB_OVCUR_DET"	:	10,
"P16:HSTX"	:	0,
"P16:I2C0_SDA"	:	3,
"P16:PIO0"	:	6,
"P16:PIO1"	:	7,
"P16:PIO2"	:	8,
"P16:PWM0_A"	:	4,
"P16:SIO"	:	5,
"P16:SPI0_RX"	:	1,
"P16:UART0_TX"	:	2,
"P16:USB_VBUS_DET"	:	10,
"P17:HSTX"	:	0,
"P17:I2C0_SCL"	:	3,
"P17:PIO0"	:	6,
"P17:PIO1"	:	7,
"P17:PIO2"	:	8,
"P17:PWM0_B"	:	4,
"P17:SIO"	:	5,
"P17:SPI0_CSn"	:	1,
"P17:UART0_RX"	:	2,
"P17:USB_VBUS_EN"	:	10,
"P18:HSTX"	:	0,
"P18:I2C1_SDA"	:	3,
"P18:PIO0"	:	6,
"P18:PIO1"	:	7,
"P18:PIO2"	:	8,
"P18:PWM1_A"	:	4,
"P18:SIO"	:	5,
"P18:SPI0_SCK"	:	1,
"P18:UART0_CTS"	:	2,
"P18:UART0_TX"	:	11,
"P18:USB_OVCUR_DET"	:	10,
"P19:HSTX"	:	0,
"P19:I2C1_SCL"	:	3,
"P19:PIO0"	:	6,
"P19:PIO1"	:	7,
"P19:PIO2"	:	8,
"P19:PWM1_B"	:	4,
"P19:QMI_CS1n"	:	9,
"P19:SIO"	:	5,
"P19:SPI0_TX"	:	1,
"P19:UART0_RTS"	:	2,
"P19:UART0_RX"	:	11,
"P19:USB_VBUS_DET"	:	10,
"P1:I2C0_SCL"	:	3,
"P1:PIO0"	:	6,
"P1:PIO1"	:	7,
"P1:PIO2"	:	8,
"P1:PWM0_B"	:	4,
"P1:SIO"	:	5,
"P1:SPI0_CSn"	:	1,
"P1:TRACECLK"	:	9,
"P1:UART0_RX"	:	2,
"P1:USB_VBUS_DET"	:	10,
"P20:CLOCK_GPIN0"	:	9,
"P20:I2C0_SDA"	:	3,
"P20:PIO0"	:	6,
"P20:PIO1"	:	7,
"P20:PIO2"	:	8,
"P20:PWM2_A"	:	4,
"P20:SIO"	:	5,
"P20:SPI0_RX"	:	1,
"P20:UART1_TX"	:	2,
"P20:USB_VBUS_EN"	:	10,
"P21:CLOCK_GPOUT0"	:	9,
"P21:I2C0_SCL"	:	3,
"P21:PIO0"	:	6,
"P21:PIO1"	:	7,
"P21:PIO2"	:	8,
"P21:PWM2_B"	:	4,
"P21:SIO"	:	5,
"P21:SPI0_CSn"	:	1,
"P21:UART1_RX"	:	2,
"P21:USB_OVCUR_DET"	:	10,
"P22:CLOCK_GPIN1"	:	9,
"P22:I2C1_SDA"	:	3,
"P22:PIO0"	:	6,
"P22:PIO1"	:	7,
"P22:PIO2"	:	8,
"P22:PWM3_A"	:	4,
"P22:SIO"	:	5,
"P22:SPI0_SCK"	:	1,
"P22:UART1_CTS"	:	2,
"P22:UART1_TX"	:	11,
"P22:USB_VBUS_DET"	:	10,
"P23:CLOCK_GPOUT1"	:	9,
"P23:I2C1_SCL"	:	3,
"P23:PIO0"	:	6,
"P23:PIO1"	:	7,
"P23:PIO2"	:	8,
"P23:PWM3_B"	:	4,
"P23:SIO"	:	5,
"P23:SPI0_TX"	:	1,
"P23:UART1_RTS"	:	2,
"P23:UART1_RX"	:	11,
"P23:USB_VBUS_EN"	:	10,
"P24:CLOCK_GPOUT2"	:	9,
"P24:I2C0_SDA"	:	3,
"P24:PIO0"	:	6,
"P24:PIO1"	:	7,
"P24:PIO2"	:	8,
"P24:PWM4_A"	:	4,
"P24:SIO"	:	5,
"P24:SPI1_RX"	:	1,
"P24:SPI1_MISO"	:	1,
"P24:UART1_TX"	:	2,
"P24:USB_OVCUR_DET"	:	10,
"P25:CLOCK_GPOUT3"	:	9,
"P25:I2C0_SCL"	:	3,
"P25:PIO0"	:	6,
"P25:PIO1"	:	7,
"P25:PIO2"	:	8,
"P25:PWM4_B"	:	4,
"P25:SIO"	:	5,
"P25:SPI1_CSn"	:	1,
"P25:UART1_RX"	:	2,
"P25:USB_VBUS_DET"	:	10,
"P26:I2C1_SDA"	:	3,
"P26:PIO0"	:	6,
"P26:PIO1"	:	7,
"P26:PIO2"	:	8,
"P26:PWM5_A"	:	4,
"P26:SIO"	:	5,
"P26:SPI1_SCK"	:	1,
"P26:UART1_CTS"	:	2,
"P26:UART1_TX"	:	11,
"P26:USB_VBUS_EN"	:	10,
"P27:I2C1_SCL"	:	3,
"P27:PIO0"	:	6,
"P27:PIO1"	:	7,
"P27:PIO2"	:	8,
"P27:PWM5_B"	:	4,
"P27:SIO"	:	5,
"P27:SPI1_TX"	:	1,
"P27:SPI1_MOSI"	:	1,
"P27:UART1_RTS"	:	2,
"P27:UART1_RX"	:	11,
"P27:USB_OVCUR_DET"	:	10,
"P28:I2C0_SDA"	:	3,
"P28:PIO0"	:	6,
"P28:PIO1"	:	7,
"P28:PIO2"	:	8,
"P28:PWM6_A"	:	4,
"P28:SIO"	:	5,
"P28:SPI1_RX"	:	1,
"P28:SPI1_MISO"	:	1,
"P28:UART0_TX"	:	2,
"P28:USB_VBUS_DET"	:	10,
"P29:I2C0_SCL"	:	3,
"P29:PIO0"	:	6,
"P29:PIO1"	:	7,
"P29:PIO2"	:	8,
"P29:PWM6_B"	:	4,
"P29:SIO"	:	5,
"P29:SPI1_CSn"	:	1,
"P29:UART0_RX"	:	2,
"P29:USB_VBUS_EN"	:	10,
"P2:I2C1_SDA"	:	3,
"P2:PIO0"	:	6,
"P2:PIO1"	:	7,
"P2:PIO2"	:	8,
"P2:PWM1_A"	:	4,
"P2:SIO"	:	5,
"P2:SPI0_SCK"	:	1,
"P2:TRACEDATA0"	:	9,
"P2:UART0_CTS"	:	2,
"P2:UART0_TX"	:	11,
"P2:USB_VBUS_EN"	:	10,
"P30:I2C1_SDA"	:	3,
"P30:PIO0"	:	6,
"P30:PIO1"	:	7,
"P30:PIO2"	:	8,
"P30:PWM7_A"	:	4,
"P30:SIO"	:	5,
"P30:SPI1_SCK"	:	1,
"P30:UART0_CTS"	:	2,
"P30:UART0_TX"	:	11,
"P30:USB_OVCUR_DET"	:	10,
"P31:I2C1_SCL"	:	3,
"P31:PIO0"	:	6,
"P31:PIO1"	:	7,
"P31:PIO2"	:	8,
"P31:PWM7_B"	:	4,
"P31:SIO"	:	5,
"P31:SPI1_TX"	:	1,
"P31:SPI1_MOSI"	:	1,
"P31:UART0_RTS"	:	2,
"P31:UART0_RX"	:	11,
"P31:USB_VBUS_DET"	:	10,
"P32:I2C0_SDA"	:	3,
"P32:PIO0"	:	6,
"P32:PIO1"	:	7,
"P32:PIO2"	:	8,
"P32:PWM8_A"	:	4,
"P32:SIO"	:	5,
"P32:SPI0_RX"	:	1,
"P32:UART0_TX"	:	2,
"P32:USB_VBUS_EN"	:	10,
"P33:I2C0_SCL"	:	3,
"P33:PIO0"	:	6,
"P33:PIO1"	:	7,
"P33:PIO2"	:	8,
"P33:PWM8_B"	:	4,
"P33:SIO"	:	5,
"P33:SPI0_CSn"	:	1,
"P33:UART0_RX"	:	2,
"P33:USB_OVCUR_DET"	:	10,
"P34:I2C1_SDA"	:	3,
"P34:PIO0"	:	6,
"P34:PIO1"	:	7,
"P34:PIO2"	:	8,
"P34:PWM9_A"	:	4,
"P34:SIO"	:	5,
"P34:SPI0_SCK"	:	1,
"P34:UART0_CTS"	:	2,
"P34:UART0_TX"	:	11,
"P34:USB_VBUS_DET"	:	10,
"P35:I2C1_SCL"	:	3,
"P35:PIO0"	:	6,
"P35:PIO1"	:	7,
"P35:PIO2"	:	8,
"P35:PWM9_B"	:	4,
"P35:SIO"	:	5,
"P35:SPI0_TX"	:	1,
"P35:UART0_RTS"	:	2,
"P35:UART0_RX"	:	11,
"P35:USB_VBUS_EN"	:	10,
"P36:I2C0_SDA"	:	3,
"P36:PIO0"	:	6,
"P36:PIO1"	:	7,
"P36:PIO2"	:	8,
"P36:PWM10_A"	:	4,
"P36:SIO"	:	5,
"P36:SPI0_RX"	:	1,
"P36:UART1_TX"	:	2,
"P36:USB_OVCUR_DET"	:	10,
"P37:I2C0_SCL"	:	3,
"P37:PIO0"	:	6,
"P37:PIO1"	:	7,
"P37:PIO2"	:	8,
"P37:PWM10_B"	:	4,
"P37:SIO"	:	5,
"P37:SPI0_CSn"	:	1,
"P37:UART1_RX"	:	2,
"P37:USB_VBUS_DET"	:	10,
"P38:I2C1_SDA"	:	3,
"P38:PIO0"	:	6,
"P38:PIO1"	:	7,
"P38:PIO2"	:	8,
"P38:PWM11_A"	:	4,
"P38:SIO"	:	5,
"P38:SPI0_SCK"	:	1,
"P38:UART1_CTS"	:	2,
"P38:UART1_TX"	:	11,
"P38:USB_VBUS_EN"	:	10,
"P39:I2C1_SCL"	:	3,
"P39:PIO0"	:	6,
"P39:PIO1"	:	7,
"P39:PIO2"	:	8,
"P39:PWM11_B"	:	4,
"P39:SIO"	:	5,
"P39:SPI0_TX"	:	1,
"P39:UART1_RTS"	:	2,
"P39:UART1_RX"	:	11,
"P39:USB_OVCUR_DET"	:	10,
"P3:I2C1_SCL"	:	3,
"P3:PIO0"	:	6,
"P3:PIO1"	:	7,
"P3:PIO2"	:	8,
"P3:PWM1_B"	:	4,
"P3:SIO"	:	5,
"P3:SPI0_TX"	:	1,
"P3:TRACEDATA1"	:	9,
"P3:UART0_RTS"	:	2,
"P3:UART0_RX"	:	11,
"P3:USB_OVCUR_DET"	:	10,
"P40:I2C0_SDA"	:	3,
"P40:PIO0"	:	6,
"P40:PIO1"	:	7,
"P40:PIO2"	:	8,
"P40:PWM8_A"	:	4,
"P40:SIO"	:	5,
"P40:SPI1_RX"	:	1,
"P40:SPI1_MISO"	:	1,
"P40:UART1_TX"	:	2,
"P40:USB_VBUS_DET"	:	10,
"P41:I2C0_SCL"	:	3,
"P41:PIO0"	:	6,
"P41:PIO1"	:	7,
"P41:PIO2"	:	8,
"P41:PWM8_B"	:	4,
"P41:SIO"	:	5,
"P41:SPI1_CSn"	:	1,
"P41:UART1_RX"	:	2,
"P41:USB_VBUS_EN"	:	10,
"P42:I2C1_SDA"	:	3,
"P42:PIO0"	:	6,
"P42:PIO1"	:	7,
"P42:PIO2"	:	8,
"P42:PWM9_A"	:	4,
"P42:SIO"	:	5,
"P42:SPI1_SCK"	:	1,
"P42:UART1_CTS"	:	2,
"P42:UART1_TX"	:	11,
"P42:USB_OVCUR_DET"	:	10,
"P43:I2C1_SCL"	:	3,
"P43:PIO0"	:	6,
"P43:PIO1"	:	7,
"P43:PIO2"	:	8,
"P43:PWM9_B"	:	4,
"P43:SIO"	:	5,
"P43:SPI1_TX"	:	1,
"P43:SPI1_MOSI"	:	1,
"P43:UART1_RTS"	:	2,
"P43:UART1_RX"	:	11,
"P43:USB_VBUS_DET"	:	10,
"P44:I2C0_SDA"	:	3,
"P44:PIO0"	:	6,
"P44:PIO1"	:	7,
"P44:PIO2"	:	8,
"P44:PWM10_A"	:	4,
"P44:SIO"	:	5,
"P44:SPI1_RX"	:	1,
"P44:SPI1_MISO"	:	1,
"P44:UART0_TX"	:	2,
"P44:USB_VBUS_EN"	:	10,
"P45:I2C0_SCL"	:	3,
"P45:PIO0"	:	6,
"P45:PIO1"	:	7,
"P45:PIO2"	:	8,
"P45:PWM10_B"	:	4,
"P45:SIO"	:	5,
"P45:SPI1_CSn"	:	1,
"P45:UART0_RX"	:	2,
"P45:USB_OVCUR_DET"	:	10,
"P46:I2C1_SDA"	:	3,
"P46:PIO0"	:	6,
"P46:PIO1"	:	7,
"P46:PIO2"	:	8,
"P46:PWM11_A"	:	4,
"P46:SIO"	:	5,
"P46:SPI1_SCK"	:	1,
"P46:UART0_CTS"	:	2,
"P46:UART0_TX"	:	11,
"P46:USB_VBUS_DET"	:	10,
"P47:I2C1_SCL"	:	3,
"P47:PIO0"	:	6,
"P47:PIO1"	:	7,
"P47:PIO2"	:	8,
"P47:PWM11_B"	:	4,
"P47:QMI_CS1n"	:	9,
"P47:SIO"	:	5,
"P47:SPI1_TX"	:	1,
"P47:SPI1_MOSI"	:	1,
"P47:UART0_RTS"	:	2,
"P47:UART0_RX"	:	11,
"P47:USB_VBUS_EN"	:	10,
"P4:I2C0_SDA"	:	3,
"P4:PIO0"	:	6,
"P4:PIO1"	:	7,
"P4:PIO2"	:	8,
"P4:PWM2_A"	:	4,
"P4:SIO"	:	5,
"P4:SPI0_RX"	:	1,
"P4:TRACEDATA2"	:	9,
"P4:UART1_TX"	:	2,
"P4:USB_VBUS_DET"	:	10,
"P5:I2C0_SCL"	:	3,
"P5:PIO0"	:	6,
"P5:PIO1"	:	7,
"P5:PIO2"	:	8,
"P5:PWM2_B"	:	4,
"P5:SIO"	:	5,
"P5:SPI0_CSn"	:	1,
"P5:TRACEDATA3"	:	9,
"P5:UART1_RX"	:	2,
"P5:USB_VBUS_EN"	:	10,
"P6:I2C1_SDA"	:	3,
"P6:PIO0"	:	6,
"P6:PIO1"	:	7,
"P6:PIO2"	:	8,
"P6:PWM3_A"	:	4,
"P6:SIO"	:	5,
"P6:SPI0_SCK"	:	1,
"P6:UART1_CTS"	:	2,
"P6:UART1_TX"	:	11,
"P6:USB_OVCUR_DET"	:	10,
"P7:I2C1_SCL"	:	3,
"P7:PIO0"	:	6,
"P7:PIO1"	:	7,
"P7:PIO2"	:	8,
"P7:PWM3_B"	:	4,
"P7:SIO"	:	5,
"P7:SPI0_TX"	:	1,
"P7:UART1_RTS"	:	2,
"P7:UART1_RX"	:	11,
"P7:USB_VBUS_DET"	:	10,
"P8:I2C0_SDA"	:	3,
"P8:PIO0"	:	6,
"P8:PIO1"	:	7,
"P8:PIO2"	:	8,
"P8:PWM4_A"	:	4,
"P8:QMI_CS1n"	:	9,
"P8:SIO"	:	5,
"P8:SPI1_RX"	:	1,
"P8:SPI1_MISO"	:	1,
"P8:UART1_TX"	:	2,
"P8:USB_VBUS_EN"	:	10,
"P9:I2C0_SCL"	:	3,
"P9:PIO0"	:	6,
"P9:PIO1"	:	7,
"P9:PIO2"	:	8,
"P9:PWM4_B"	:	4,
"P9:SIO"	:	5,
"P9:SPI1_CSn"	:	1,
"P9:UART1_RX"	:	2,
"P9:USB_OVCUR_DET"	:	10,
# 48,49 is a fake one for USB doesnt really exist or map to a pin.
"P48:OTG_FS_DM"	:	1,
"P49:OTG_FS_DP"	:	2,
}

for k,v in _AltFunction_map.items():
	AltFunction_map[k] = v

# 12.4. ADC and Temperature Sensor
# RP2350 has an internal analogue-digital converter (ADC) with the following features:
# • SAR ADC (see Section 12.4.3)
# • 500 kS/s (using an independent 48 MHz clock)
# • 12-bit with 9.2 ENOB (see Section 12.4.4)
# • Five or nine input mux:
# ◦ Four inputs available on QFN-60 package pins shared with GPIO[29:26]
# ◦ One input dedicated to the internal temperature sensor (see Section 12.4.6)
# • Eight element receive sample FIFO
# • Interrupt generation
# • DMA interface (see Section 12.4.3.5)

# qfn60 package has 4 ADC pins:, qfn80 todo
ADC1_map = {
	# format is PIN : ADC1_CHAN
	"P26"	:	0,
	"P27"	:	1,
	"P28"	:	2,
	"P29"	:	3,
    # internal temperature sensor is ADC1_CHAN 4 but we dont map it to a pin
}

# 2350 has 16 dma channels, 4 shared irqs.
# a READ_ADDR, WRITE_ADDR, TRANS_COUNT, and CTRL register per-channel.

# dma subsystem cant access SIO subsystem use PIO instead.

# DREQ is totally  different from stm32.
#The DMA controller on the RP2350 behaves very differently from the STM32F4. While the STM32F4 relies on a fixed "stream/channel" mapping where specific peripherals are tied to specific DMA streams, the
#RP2350 uses a flexible "request signal" (DREQ) system, where any of the 12+ DMA channels can be mapped to any peripheral or PIO state machine

#we can fake the any-dma-of-12 -> to any peripheral, by mapping 12 for each peripheral.

DMA_Map = {
    
	# strictly speaking its a single adc with 5x multiplexing, but 5x adcs is close enough.
	"ADC0"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"ADC1"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"ADC2"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"ADC3"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"ADC4"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],    # build in temp sensor
    
	"PIO0_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PIO0_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PIO1_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PIO1_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PIO2_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PIO2_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
    
	# dma subsystem cant access SIO subsystem use PIO instead.
    #"SIO_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	#"SIO_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
    
	"SPI0_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"SPI0_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"SPI1_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"SPI1_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
    
	"UART0_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"UART0_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"UART1_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"UART1_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],

    "I2C0_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"I2C0_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"I2C1_RX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"I2C1_TX" 	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
    
    
	# pwm, thre are 12 pwm outs that "can be continuously reprogrammed via the DMA", and "can generate interrupts to either of two system IRQ lines" and "can trigger DMA transfers to other peripherals"
    "PWM0"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM1"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM2"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM3"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM4"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM5"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM6"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM7"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM8"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM9"    	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM10"   	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"PWM11"   	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	
	#TIMER0 and TIMER1
	"TIMER0"  	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],
	"TIMER1"  	:	[(0,0,0),(1,1,1),(2,2,2),(3,3,3),(4,4,4),(5,5,5),(6,6,6),(7,7,7),(8,8,8),(9,9,9),(10,10,10),(11,11,11)],



}

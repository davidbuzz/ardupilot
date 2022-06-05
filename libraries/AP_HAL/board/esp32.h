#pragma once

//generated header
#include <hwdef.h>

#define HAL_BOARD_NAME "ESP32"
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#define HAL_WITH_UAVCAN 0
//#define HAL_HAVE_SAFETY_SWITCH 0
//#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0

#define HAL_WITH_IO_MCU 0

#define O_CLOEXEC 0
#define HAL_STORAGE_SIZE (16384)

// allow for static semaphores
#include <type_traits>
#include <AP_HAL_ESP32/Semaphores.h>
#define HAL_Semaphore ESP32::Semaphore

#include <AP_HAL/EventHandle.h>
#define HAL_EventHandle AP_HAL::EventHandle

#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#define HAL_MEM_CLASS HAL_MEM_CLASS_192


#define __LITTLE_ENDIAN  1234
#define __BYTE_ORDER     __LITTLE_ENDIAN

//- these are missing from esp-idf......will not be needed later
#define RTC_WDT_STG_SEL_OFF             0
#define RTC_WDT_STG_SEL_INT             1
#define RTC_WDT_STG_SEL_RESET_CPU       2
#define RTC_WDT_STG_SEL_RESET_SYSTEM    3
#define RTC_WDT_STG_SEL_RESET_RTC       4

// whenver u get ... error: "xxxxxxx" is not defined, evaluates to 0 [-Werror=undef]  just define it below as 0
#define CONFIG_SPIRAM_ALLOW_BSS_SEG_EXTERNAL_MEMORY 0
#define XCHAL_ERRATUM_453 0
#define CONFIG_FREERTOS_CORETIMER_0 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE 0
#define CONFIG_FREERTOS_CHECK_STACKOVERFLOW_PTRVAL 0
#define CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP 0
#define CONFIG_FREERTOS_USE_TICKLESS_IDLE 0
#define CONFIG_SYSVIEW_ENABLE 0
#define CONFIG_SPI_FLASH_DANGEROUS_WRITE_ALLOWED 0
#define CONFIG_SPI_FLASH_ENABLE_COUNTERS 0
#define USE_LIBC_REALLOC 0
#define CONFIG_LWIP_DHCP_RESTORE_LAST_IP 0
#define CONFIG_LWIP_STATS 0
#define CONFIG_LWIP_PPP_SUPPORT 0
#define CONFIG_LWIP_STATS 0
#define CONFIG_ESP32_WIFI_CSI_ENABLED 0
#define CONFIG_ESP32_WIFI_NVS_ENABLED 0
#define CONFIG_NEWLIB_NANO_FORMAT 0
#define CONFIG_LWIP_IP4_REASSEMBLY 0
#define CONFIG_LWIP_IP6_REASSEMBLY 0
#define CONFIG_LWIP_STATS 0
#define LWIP_COMPAT_SOCKET_INET 0
#define LWIP_COMPAT_SOCKET_ADDR 0
#define CONFIG_ESP32_WIFI_TX_BA_WIN 0
#define CONFIG_ESP32_WIFI_RX_BA_WIN 0

// absolutely essential, as it defualts to 1324 in AP_Logger/AP_Logger.cpp, and that NOT enough.
// ....with stack checking enabled in FreRTOS and GDB connected, GDB reports:
// 0x4037ba21 in panic_abort (details=0x3fccdbb1 "***ERROR*** A stack overflow in task log_io has been detected.")
#define HAL_LOGGING_STACK_SIZE 1024*3

/* string names for well known SPI devices - stolen from ./chibios.h */
#define HAL_BARO_MS5611_NAME "ms5611"
#define HAL_BARO_MS5611_SPI_INT_NAME "ms5611_int"
#define HAL_BARO_MS5611_SPI_EXT_NAME "ms5611_ext"
#define HAL_BARO_LPS22H_NAME "lps22h"
#ifndef HAL_BARO_BMP280_NAME
#define HAL_BARO_BMP280_NAME "bmp280"
#endif
#define HAL_INS_MPU60x0_NAME "mpu6000"
#define HAL_INS_MPU60x0_EXT_NAME "mpu6000_ext"
#define HAL_INS_LSM9DS0_G_NAME "lsm9ds0_g"
#define HAL_INS_LSM9DS0_A_NAME "lsm9ds0_am"
#define HAL_INS_LSM9DS0_EXT_G_NAME "lsm9ds0_ext_g"
#define HAL_INS_LSM9DS0_EXT_A_NAME "lsm9ds0_ext_am"
#define HAL_INS_MPU9250_NAME "mpu9250"
#define HAL_INS_MPU9250_EXT_NAME "mpu9250_ext"
#define HAL_INS_MPU6500_NAME "mpu6500"
#define HAL_INS_ICM20608_NAME "icm20608"
#define HAL_INS_ICM20608_AM_NAME "icm20608-am"
#define HAL_INS_ICM20608_EXT_NAME "icm20608_ext"
#define HAL_COMPASS_HMC5843_NAME "hmc5843"
#define HAL_COMPASS_LIS3MDL_NAME "lis3mdl"

#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_IMU_SPI2(driver, devname1, devname2, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname1),hal.spi->get_device(devname2),##args))
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))
#define PROBE_MAG_I2C(driver, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_MAG_SPI(driver, devname, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe(hal.spi->get_device(devname),##args))
#define PROBE_MAG_IMU(driver, imudev, imu_instance, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(imu_instance,##args))
#define PROBE_MAG_IMU_I2C(driver, imudev, bus, addr, args ...) ADD_BACKEND(DRIVER_ ##driver, AP_Compass_ ## driver::probe_ ## imudev(GET_I2C_DEVICE(bus,addr),##args))

// our compile no understand this.
#ifndef WARN_IF_UNUSED
#define WARN_IF_UNUSED
#endif

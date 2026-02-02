/*
 * u8g2_esp32_hal.h
 *
 *  Created on: Feb 12, 2017
 *      Author: kolban
 *
 *  Modified: Migrated from legacy I2C driver (driver/i2c.h) to new
 *            I2C master driver (driver/i2c_master.h) for ESP-IDF 5.x+
 */

#ifndef U8G2_ESP32_HAL_H_
#define U8G2_ESP32_HAL_H_

#include "u8g2.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"

#define U8G2_ESP32_HAL_UNDEFINED GPIO_NUM_NC

/** @public
 * HAL configuration structure.
 */
typedef struct {
  union {
    /* SPI settings. */
    struct {
      /* GPIO num for clock. */
      gpio_num_t clk;
      /* GPIO num for SPI mosi. */
      gpio_num_t mosi;
      /* GPIO num for SPI slave/chip select. */
      gpio_num_t cs;
    } spi;
    /* I2C settings. */
    struct {
      /* GPIO num for I2C data. */
      gpio_num_t sda;
      /* GPIO num for I2C clock. */
      gpio_num_t scl;
    } i2c;
  } bus;
  /* GPIO num for reset. */
  gpio_num_t reset;
  /* GPIO num for DC. */
  gpio_num_t dc;
  /* I2C port number (e.g., I2C_NUM_0 or I2C_NUM_1) */
  i2c_port_num_t i2c_port;
  /* I2C clock speed in Hz (e.g., 100000 for 100kHz, 400000 for 400kHz) */
  uint32_t i2c_clk_speed;
  /* Pre-initialized I2C bus handle (optional, set to NULL to auto init) */
  i2c_master_bus_handle_t i2c_bus_handle;
  /* SPI host device (e.g., SPI2_HOST or SPI3_HOST) */
  spi_host_device_t spi_host;
  /* SPI clock speed in Hz (e.g., 1000000 for 1MHz, 10000000 for 10MHz) */
  uint32_t spi_clk_speed;
} u8g2_esp32_hal_t;

/**
 * Construct a default HAL configuration with all fields undefined.
 */
#define U8G2_ESP32_HAL_DEFAULT                       \
  {.bus = {.spi = {.clk = U8G2_ESP32_HAL_UNDEFINED,  \
                   .mosi = U8G2_ESP32_HAL_UNDEFINED, \
                   .cs = U8G2_ESP32_HAL_UNDEFINED}}, \
   .reset = U8G2_ESP32_HAL_UNDEFINED,                \
   .dc = U8G2_ESP32_HAL_UNDEFINED,                   \
   .i2c_port = I2C_NUM_0,                            \
   .i2c_clk_speed = 400000,                          \
   .i2c_bus_handle = NULL,                           \
   .spi_host = SPI2_HOST,                            \
   .spi_clk_speed = 1000000}

/**
 * Initialize the HAL with the given configuration.
 *
 * @see u8g2_esp32_hal_t
 * @see U8G2_ESP32_HAL_DEFAULT
 */
void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param);
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t* u8x8,
                               uint8_t msg,
                               uint8_t arg_int,
                               void* arg_ptr);
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t* u8x8,
                               uint8_t msg,
                               uint8_t arg_int,
                               void* arg_ptr);
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t* u8x8,
                                     uint8_t msg,
                                     uint8_t arg_int,
                                     void* arg_ptr);

/**
 * Deinitialize the HAL and release all resources.
 * Call this to clean up I2C/SPI resources before reinitializing.
 */
void u8g2_esp32_hal_deinit(void);

#endif /* U8G2_ESP32_HAL_H_ */
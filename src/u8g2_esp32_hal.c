#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "u8g2_esp32_hal.h"

static const char* TAG = "u8g2_hal";
static const unsigned int I2C_TIMEOUT_MS = 1000;

static spi_device_handle_t handle_spi = NULL;
static bool spi_bus_owned =
    false;  // is the spi bus initialized by this component

static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_dev_handle = NULL;
static bool i2c_bus_owned =
    false;  // is the i2c bus initialized by this component

#ifndef U8G2_I2C_BUFFER_SIZE
#define U8G2_I2C_BUFFER_SIZE 256
#endif
static uint8_t i2c_buffer[U8G2_I2C_BUFFER_SIZE];
static size_t i2c_buffer_len = 0;
static uint8_t i2c_cached_address =
    0;  // Cached I2C address for device handle reuse

static u8g2_esp32_hal_t u8g2_esp32_hal;  // HAL state data.

// Macro for error handling - logs error and returns failure code
#define HAL_ERROR_CHECK(x, fail_return)                                 \
  do {                                                                  \
    esp_err_t rc = (x);                                                 \
    if (rc != ESP_OK) {                                                 \
      ESP_LOGE(TAG, "Error: %s (%d) at %s:%d", esp_err_to_name(rc), rc, \
               __FILE__, __LINE__);                                     \
      return fail_return;                                               \
    }                                                                   \
  } while (0)

/*
 * Initialze the ESP32 HAL.
 */
void u8g2_esp32_hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param) {
  u8g2_esp32_hal = u8g2_esp32_hal_param;
}  // u8g2_esp32_hal_init

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is
 * invoked to handle SPI communications.
 */
uint8_t u8g2_esp32_spi_byte_cb(u8x8_t* u8x8,
                               uint8_t msg,
                               uint8_t arg_int,
                               void* arg_ptr) {
  ESP_LOGD(TAG, "spi_byte_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p",
           msg, arg_int, arg_ptr);
  switch (msg) {
    case U8X8_MSG_BYTE_SET_DC:
      if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
        gpio_set_level(u8g2_esp32_hal.dc, arg_int);
      }
      break;

    case U8X8_MSG_BYTE_INIT: {
      if (u8g2_esp32_hal.bus.spi.clk == U8G2_ESP32_HAL_UNDEFINED ||
          u8g2_esp32_hal.bus.spi.mosi == U8G2_ESP32_HAL_UNDEFINED ||
          u8g2_esp32_hal.bus.spi.cs == U8G2_ESP32_HAL_UNDEFINED) {
        break;
      }

      // Guard against double initialization
      if (handle_spi != NULL) {
        ESP_LOGW(TAG, "SPI already initialized, skipping");
        break;
      }

      spi_bus_config_t bus_config = {0};
      bus_config.sclk_io_num = u8g2_esp32_hal.bus.spi.clk;   // CLK
      bus_config.mosi_io_num = u8g2_esp32_hal.bus.spi.mosi;  // MOSI
      bus_config.miso_io_num = GPIO_NUM_NC;                  // MISO
      bus_config.quadwp_io_num = GPIO_NUM_NC;                // Not used
      bus_config.quadhd_io_num = GPIO_NUM_NC;                // Not used

      // Try to initialize bus - if already initialized by another component,
      // skip
      esp_err_t ret = spi_bus_initialize(u8g2_esp32_hal.spi_host, &bus_config,
                                         SPI_DMA_CH_AUTO);
      if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already initialized by another component");
        spi_bus_owned = false;
      } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return 0;
      } else {
        spi_bus_owned = true;
      }

      spi_device_interface_config_t dev_config = {0};
      dev_config.address_bits = 0;
      dev_config.command_bits = 0;
      dev_config.dummy_bits = 0;
      dev_config.mode = 0;
      dev_config.duty_cycle_pos = 0;
      dev_config.cs_ena_posttrans = 0;
      dev_config.cs_ena_pretrans = 0;
      dev_config.clock_speed_hz = u8g2_esp32_hal.spi_clk_speed;
      dev_config.spics_io_num = u8g2_esp32_hal.bus.spi.cs;
      dev_config.flags = 0;
      dev_config.queue_size = 200;
      dev_config.pre_cb = NULL;
      dev_config.post_cb = NULL;
      HAL_ERROR_CHECK(
          spi_bus_add_device(u8g2_esp32_hal.spi_host, &dev_config, &handle_spi),
          0);

      break;
    }

    case U8X8_MSG_BYTE_SEND: {
      spi_transaction_t trans_desc = {0};
      trans_desc.addr = 0;
      trans_desc.cmd = 0;
      trans_desc.flags = 0;
      trans_desc.length = 8 * arg_int;  // Number of bits NOT number of bytes.
      trans_desc.rxlength = 0;
      trans_desc.tx_buffer = arg_ptr;
      trans_desc.rx_buffer = NULL;
      HAL_ERROR_CHECK(spi_device_transmit(handle_spi, &trans_desc), 0);
      break;
    }
  }
  return 0;
}  // u8g2_esp32_spi_byte_cb

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is
 * invoked to handle I2C communications.
 */
uint8_t u8g2_esp32_i2c_byte_cb(u8x8_t* u8x8,
                               uint8_t msg,
                               uint8_t arg_int,
                               void* arg_ptr) {
  ESP_LOGD(TAG, "i2c_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg,
           arg_int, arg_ptr);

  switch (msg) {
    case U8X8_MSG_BYTE_SET_DC: {
      if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
        gpio_set_level(u8g2_esp32_hal.dc, arg_int);
      }
      break;
    }

    case U8X8_MSG_BYTE_INIT: {
      if (u8g2_esp32_hal.bus.i2c.sda == U8G2_ESP32_HAL_UNDEFINED ||
          u8g2_esp32_hal.bus.i2c.scl == U8G2_ESP32_HAL_UNDEFINED) {
        break;
      }

      // Guard against double initialization
      if (i2c_bus_handle != NULL) {
        ESP_LOGW(TAG, "I2C bus already initialized, skipping");
        break;
      }

      // Check if user provided a pre-initialized bus handle
      if (u8g2_esp32_hal.i2c_bus_handle != NULL) {
        ESP_LOGI(TAG, "Using pre-initialized I2C bus handle");
        i2c_bus_handle = u8g2_esp32_hal.i2c_bus_handle;
        i2c_bus_owned = false;  // Don't free this bus on deinit
        break;
      }

      i2c_master_bus_config_t bus_config = {
          .i2c_port = u8g2_esp32_hal.i2c_port,
          .sda_io_num = u8g2_esp32_hal.bus.i2c.sda,
          .scl_io_num = u8g2_esp32_hal.bus.i2c.scl,
          .clk_source = I2C_CLK_SRC_DEFAULT,
          .glitch_ignore_cnt = 7,
          .flags.enable_internal_pullup = true,
      };
      ESP_LOGI(TAG, "sda_io_num %d", u8g2_esp32_hal.bus.i2c.sda);
      ESP_LOGI(TAG, "scl_io_num %d", u8g2_esp32_hal.bus.i2c.scl);
      ESP_LOGI(TAG, "i2c_port %d", u8g2_esp32_hal.i2c_port);
      ESP_LOGI(TAG, "clk_speed %lu",
               (unsigned long)u8g2_esp32_hal.i2c_clk_speed);

      // Try to initialize bus
      esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
      if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return 0;
      }
      i2c_bus_owned = true;
      break;
    }

    case U8X8_MSG_BYTE_SEND: {
      uint8_t* data_ptr = (uint8_t*)arg_ptr;
      ESP_LOG_BUFFER_HEXDUMP(TAG, data_ptr, arg_int, ESP_LOG_VERBOSE);

      if (i2c_buffer_len + arg_int <= sizeof(i2c_buffer)) {
        memcpy(i2c_buffer + i2c_buffer_len, data_ptr, arg_int);
        i2c_buffer_len += arg_int;
      } else {
        ESP_LOGE(TAG, "I2C buffer overflow!");
        return 0;  // Return early on overflow
      }
      break;
    }

    case U8X8_MSG_BYTE_START_TRANSFER: {
      uint8_t i2c_address = u8x8_GetI2CAddress(u8x8);
      ESP_LOGD(TAG, "Start I2C transfer to %02X.", i2c_address >> 1);

      // Only add device if handle is NULL or address changed
      if (i2c_dev_handle == NULL || i2c_cached_address != i2c_address) {
        // Remove old device if address changed
        if (i2c_dev_handle != NULL) {
          i2c_master_bus_rm_device(i2c_dev_handle);
          i2c_dev_handle = NULL;
        }

        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = i2c_address >> 1,
            .scl_speed_hz = u8g2_esp32_hal.i2c_clk_speed,
        };
        HAL_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_config,
                                                  &i2c_dev_handle),
                        0);
        i2c_cached_address = i2c_address;
      }
      i2c_buffer_len = 0;
      break;
    }

    case U8X8_MSG_BYTE_END_TRANSFER: {
      ESP_LOGD(TAG, "End I2C transfer, sending %d bytes.", i2c_buffer_len);
      HAL_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, i2c_buffer,
                                          i2c_buffer_len, I2C_TIMEOUT_MS),
                      0);
      // Keep device handle cached for subsequent transfers
      break;
    }
  }
  return 0;
}  // u8g2_esp32_i2c_byte_cb

/*
 * HAL callback function as prescribed by the U8G2 library.  This callback is
 * invoked to handle callbacks for GPIO and delay functions.
 */
uint8_t u8g2_esp32_gpio_and_delay_cb(u8x8_t* u8x8,
                                     uint8_t msg,
                                     uint8_t arg_int,
                                     void* arg_ptr) {
  ESP_LOGD(TAG,
           "gpio_and_delay_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p",
           msg, arg_int, arg_ptr);

  switch (msg) {
    case U8X8_MSG_GPIO_AND_DELAY_INIT: {
      uint64_t bitmask = 0;
      if (u8g2_esp32_hal.dc != U8G2_ESP32_HAL_UNDEFINED) {
        bitmask = bitmask | (1ull << u8g2_esp32_hal.dc);
      }
      if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
        bitmask = bitmask | (1ull << u8g2_esp32_hal.reset);
      }
      // Note: CS is managed by SPI master driver via spics_io_num

      if (bitmask == 0) {
        break;
      }
      gpio_config_t gpioConfig;
      gpioConfig.pin_bit_mask = bitmask;
      gpioConfig.mode = GPIO_MODE_OUTPUT;
      gpioConfig.pull_up_en =
          GPIO_PULLUP_ENABLE;  // Pull-up for CS/RESET (active low)
      gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
      gpioConfig.intr_type = GPIO_INTR_DISABLE;
      gpio_config(&gpioConfig);
      break;
    }

    case U8X8_MSG_GPIO_RESET:
      if (u8g2_esp32_hal.reset != U8G2_ESP32_HAL_UNDEFINED) {
        gpio_set_level(u8g2_esp32_hal.reset, arg_int);
      }
      break;

    case U8X8_MSG_GPIO_CS:
      // CS is managed by SPI master driver via spics_io_num
      break;

    case U8X8_MSG_GPIO_I2C_CLOCK:
      // I2C clock is managed by i2c_master driver
      break;

    case U8X8_MSG_GPIO_I2C_DATA:
      // I2C data is managed by i2c_master driver
      break;

    case U8X8_MSG_DELAY_MILLI:
      // Round up to avoid truncating small delays to 0 ticks
      vTaskDelay((arg_int + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
      break;
  }
  return 0;
}  // u8g2_esp32_gpio_and_delay_cb

/*
 * Deinitialize the ESP32 HAL and release all resources.
 */
void u8g2_esp32_hal_deinit(void) {
  // Clean up I2C resources
  if (i2c_dev_handle != NULL) {
    i2c_master_bus_rm_device(i2c_dev_handle);
    i2c_dev_handle = NULL;
  }
  if (i2c_bus_handle != NULL && i2c_bus_owned) {
    i2c_del_master_bus(i2c_bus_handle);
    i2c_bus_handle = NULL;
  }
  i2c_bus_owned = false;
  i2c_cached_address = 0;
  i2c_buffer_len = 0;

  // Clean up SPI resources
  if (handle_spi != NULL) {
    spi_bus_remove_device(handle_spi);
    handle_spi = NULL;
  }
  if (spi_bus_owned) {
    spi_bus_free(u8g2_esp32_hal.spi_host);
    spi_bus_owned = false;
  }
}  // u8g2_esp32_hal_deinit
/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(main);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

// Command to read flash device ID
#define FLASH_CMD_READ_ID 0x9F

// Command to read flash size (Winbond standard)
#define FLASH_CMD_READ_SIZE 0xD8

static struct device *spi_flash_dev;

const struct spi_config spi_cfg = {
    .frequency = 40000000, // Set the SPI frequency
    .operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL, // CS will be controlled manually, via the GPIO pin defined
                // in the device tree
};

int flash_get_size(void) {

  uint8_t tx_buf[1] = {FLASH_CMD_READ_SIZE}; // Command to read size
  uint8_t rx_buf[3] = {0};                   // Space for size (3 bytes)

  const struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
  const struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};

  const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
  const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

  int ret = spi_transceive(spi_flash_dev, &spi_cfg, &tx_set, &rx_set);
  if (ret) {
    printk("SPI transceive failed: %d\n", ret);
    return ret;
  }

  // The response typically contains the flash size in bytes
  uint32_t flash_size = (rx_buf[0] << 16) | (rx_buf[1] << 8) | rx_buf[2];

  printk("Flash size: %u bytes\n", flash_size);
  return flash_size;
}

int flash_init(void) {
  // Get the SPI device at runtime using device_get_binding
  spi_flash_dev = device_get_binding("extflash");
  if (!spi_flash_dev) {
    printk("Failed to get SPI device binding\n");
    return -ENODEV;
  }

  uint8_t tx_buf[1] = {FLASH_CMD_READ_ID};
  uint8_t rx_buf[3] = {
      0}; // Space for manufacturer ID, memory type, and capacity

  const struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
  const struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};

  const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
  const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

  int ret = spi_transceive(spi_flash_dev, &spi_cfg, &tx_set, &rx_set);
  if (ret) {
    printk("SPI transceive failed: %d\n", ret);
    return ret;
  }

  printk("Flash device ID: 0x%02X 0x%02X 0x%02X\n", rx_buf[0], rx_buf[1],
         rx_buf[2]);
  return 0;
}

void blink0(void) {

#if MPU_FAULT_EXAMPLE
  uint8_t buf[2000] = {0};
#endif

  while (true) {
    gpio_pin_toggle_dt(&led0);
    k_msleep(200);
  }
}

bool led_init() {
  if (!gpio_is_ready_dt(&led0)) {
    return false;
  }

  int ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return false;
  }

  return true;
}

int main(void) {

  k_msleep(1000);

  led_init();

  flash_init();

  flash_get_size();

#if USAGE_FAULT_EXAMPLE
  void (*callback)() = 0;

  callback();
#endif

  while (1) {
    k_msleep(1000);
  }
  return 0;
}

K_THREAD_DEFINE(blink0_id, 1024, blink0, NULL, NULL, NULL, 7, 0, 0);
// K_THREAD_DEFINE(blink1_id, 1024, blink1, NULL, NULL, NULL, 7, 0, 0);

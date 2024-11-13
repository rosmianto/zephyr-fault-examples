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

// Use SPI_DT_SPEC_GET to fetch the SPI device configuration
static const struct spi_dt_spec spi_flash = SPI_DT_SPEC_GET(
    DT_NODELABEL(w25q16cv),
    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA, 0);

int flash_get_size(void) {
  // if (!spi_is_ready_dt(&spi_flash)) {
  //   printk("SPI device not ready\n");
  //   return -ENODEV;
  // }

  uint8_t tx_buf[1] = {FLASH_CMD_READ_SIZE}; // Command to read size
  uint8_t rx_buf[3] = {0};                   // Space for size (3 bytes)

  const struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
  const struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};

  const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
  const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

  int ret = spi_transceive_dt(&spi_flash, &tx_set, &rx_set);
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
  if (!spi_is_ready_dt(&spi_flash)) {
    printk("SPI device not ready\n");
    return -ENODEV;
  }

  uint8_t tx_buf[1] = {FLASH_CMD_READ_ID};
  uint8_t rx_buf[3] = {
      0}; // Space for manufacturer ID, memory type, and capacity

  const struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
  const struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};

  const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
  const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

  int ret = spi_transceive_dt(&spi_flash, &tx_set, &rx_set);
  if (ret) {
    printk("SPI transceive failed: %d\n", ret);
    return ret;
  }

  printk("Flash device ID: 0x%02X 0x%02X 0x%02X\n", rx_buf[0], rx_buf[1],
         rx_buf[2]);
  return 0;
}

void blink0(void) {

  flash_get_size();

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

  while (1) {
    k_msleep(1000);
  }
  return 0;
}

K_THREAD_DEFINE(blink0_id, 1024, blink0, NULL, NULL, NULL, 7, 0, 0);
// K_THREAD_DEFINE(blink1_id, 1024, blink1, NULL, NULL, NULL, 7, 0, 0);

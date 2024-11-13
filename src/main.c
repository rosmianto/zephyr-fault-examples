/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>

LOG_MODULE_REGISTER(main);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

static int lsdir(const char *path) {
  int res;
  struct fs_dir_t dirp;
  static struct fs_dirent entry;

  fs_dir_t_init(&dirp);

  /* Verify fs_opendir() */
  res = fs_opendir(&dirp, path);
  if (res) {
    LOG_ERR("Error opening dir %s [%d]\n", path, res);
    return res;
  }

  LOG_PRINTK("\nListing dir %s ...\n", path);
  for (;;) {
    /* Verify fs_readdir() */
    res = fs_readdir(&dirp, &entry);

    /* entry.name[0] == 0 means end-of-dir */
    if (res || entry.name[0] == 0) {
      if (res < 0) {
        LOG_ERR("Error reading dir [%d]\n", res);
      }
      break;
    }

    if (entry.type == FS_DIR_ENTRY_DIR) {
      LOG_PRINTK("[DIR ] %s\n", entry.name);
    } else {
      LOG_PRINTK("[FILE] %s (size = %zu)\n", entry.name, entry.size);
    }
  }

  /* Verify fs_closedir() */
  fs_closedir(&dirp);

  return res;
}

static int littlefs_flash_erase(unsigned int id) {
  const struct flash_area *pfa;
  int rc;

  rc = flash_area_open(id, &pfa);
  if (rc < 0) {
    LOG_ERR("FAIL: unable to find flash area %u: %d\n", id, rc);
    return rc;
  }

  LOG_PRINTK("Area %u at 0x%x on %s for %u bytes\n", id,
             (unsigned int)pfa->fa_off, pfa->fa_dev->name,
             (unsigned int)pfa->fa_size);

  /* Optional wipe flash contents */
  if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
    rc = flash_area_flatten(pfa, 0, pfa->fa_size);
    LOG_ERR("Erasing flash area ... %d", rc);
  }

  flash_area_close(pfa);
  return rc;
}
#define PARTITION_NODE DT_NODELABEL(lfs1)

#if DT_NODE_EXISTS(PARTITION_NODE)
FS_FSTAB_DECLARE_ENTRY(PARTITION_NODE);
#else  /* PARTITION_NODE */
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_storage_mnt = {
    .type = FS_LITTLEFS,
    .fs_data = &storage,
    .storage_dev = (void *)FIXED_PARTITION_ID(storage_partition),
    .mnt_point = "/lfs",
};
#endif /* PARTITION_NODE */

struct fs_mount_t *mountpoint =
#if DT_NODE_EXISTS(PARTITION_NODE)
    &FS_FSTAB_ENTRY(PARTITION_NODE)
#else
    &lfs_storage_mnt
#endif
    ;

static int littlefs_mount(struct fs_mount_t *mp) {
  int rc;

  rc = littlefs_flash_erase((uintptr_t)mp->storage_dev);
  if (rc < 0) {
    return rc;
  }

  /* Do not mount if auto-mount has been enabled */
#if !DT_NODE_EXISTS(PARTITION_NODE) ||                                         \
    !(FSTAB_ENTRY_DT_MOUNT_FLAGS(PARTITION_NODE) & FS_MOUNT_FLAG_AUTOMOUNT)
  rc = fs_mount(mp);
  if (rc < 0) {
    LOG_PRINTK("FAIL: mount id %" PRIuPTR " at %s: %d\n",
               (uintptr_t)mp->storage_dev, mp->mnt_point, rc);
    return rc;
  }
  LOG_PRINTK("%s mount: %d\n", mp->mnt_point, rc);
#else
  LOG_PRINTK("%s automounted\n", mp->mnt_point);
#endif

  return 0;
}

struct personal_info {
  int age;
  int height;
};

struct personal_info *generate_info() {
  struct personal_info info;

  info.age = 10;
  info.height = 150;

  return &info;
}

void blink0(void) {

  struct personal_info *get_info = generate_info();
  LOG_PRINTK("age: %d\nheight: %d\n", get_info->age, get_info->height);

  while (true) {
    gpio_pin_toggle_dt(&led0);
    k_msleep(200);
  }
}

void blink1(void) {
  while (true) {
    gpio_pin_toggle_dt(&led1);
    k_msleep(100);
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

  if (!gpio_is_ready_dt(&led1)) {
    return false;
  }

  ret = gpio_pin_configure_dt(&led1, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return false;
  }

  return true;
}

int main(void) {

  k_msleep(3000);

  led_init();

  while (1) {
    k_msleep(SLEEP_TIME_MS);
  }
  return 0;
}

K_THREAD_DEFINE(blink0_id, 1024, blink0, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(blink1_id, 1024, blink1, NULL, NULL, NULL, 7, 0, 0);

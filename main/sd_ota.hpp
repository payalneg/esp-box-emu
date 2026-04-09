#pragma once

#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"

static const char *SD_OTA_TAG = "sd_ota";

/// Check for update.bin on SD card and flash it via OTA.
/// Returns only if no update found or on error. On success, reboots.
static inline void check_sd_ota(const char *mount_point) {
    char path[64];
    snprintf(path, sizeof(path), "%s/update.bin", mount_point);

    FILE *f = fopen(path, "rb");
    if (!f) {
        return; // no update file — normal boot
    }

    // Get file size for progress logging
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    ESP_LOGI(SD_OTA_TAG, "Found %s (%ld bytes), starting OTA update...", path, file_size);

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        ESP_LOGE(SD_OTA_TAG, "No OTA partition found");
        fclose(f);
        return;
    }

    ESP_LOGI(SD_OTA_TAG, "Writing to partition '%s' at offset 0x%lx",
             update_partition->label, (unsigned long)update_partition->address);

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(SD_OTA_TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        fclose(f);
        return;
    }

    char buf[4096];
    size_t total_written = 0;
    size_t bytes_read;

    while ((bytes_read = fread(buf, 1, sizeof(buf), f)) > 0) {
        err = esp_ota_write(ota_handle, buf, bytes_read);
        if (err != ESP_OK) {
            ESP_LOGE(SD_OTA_TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            fclose(f);
            return;
        }
        total_written += bytes_read;
        if (total_written % (256 * 1024) == 0 || total_written == (size_t)file_size) {
            ESP_LOGI(SD_OTA_TAG, "Written %zu / %ld bytes (%d%%)",
                     total_written, file_size,
                     (int)(total_written * 100 / file_size));
        }
    }

    fclose(f);

    if (total_written == 0) {
        ESP_LOGE(SD_OTA_TAG, "Update file is empty");
        esp_ota_abort(ota_handle);
        return;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(SD_OTA_TAG, "esp_ota_end failed: %s (image may be invalid)", esp_err_to_name(err));
        return;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(SD_OTA_TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return;
    }

    // Rename update.bin so it doesn't re-flash on next boot
    char done_path[64];
    snprintf(done_path, sizeof(done_path), "%s/update.bin.done", mount_point);
    remove(done_path); // remove old .done if exists
    rename(path, done_path);

    ESP_LOGI(SD_OTA_TAG, "OTA update successful (%zu bytes). Rebooting...", total_written);
    esp_restart();
}

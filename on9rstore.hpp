#pragma once

#include <cstddef>
#include <cstdint>
#include <limits.h>
#include <memory>

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "on9rstore_defs.hpp"

struct on9rstore_cfg {
    // FAT mount path passed to esp_vfs_fat_*(), e.g. "/data" for "/data/log.db".
    const char *base_path = nullptr;

    // The file is created once at this exact size and is never grown or shrunk.
    uint64_t file_size = 1024ULL * 1024ULL * 1024ULL;

    // Entries up to this size are batched before flush_write(). Larger entries are
    // committed directly without allocating a second full-entry buffer.
    size_t write_buffer_size = 8192;

    // When ESP-IDF exposes a new valid flash coredump, append it after the boot_event
    // header. The original coredump partition is left untouched.
    bool copy_coredump = true;
};

class on9rstore
{
public:
    explicit on9rstore(const char *file_path, on9rstore_cfg *config);
    ~on9rstore();

public:
    esp_err_t init();
    esp_err_t append_entry(uint16_t type, const uint8_t *payload, size_t payload_len,
                            on9rstore_def::entry_header *entry_info_out = nullptr,
                            uint32_t timeout_ticks = portMAX_DELAY, bool force_flush = false);
    esp_err_t append_time_sync(on9rstore_def::time_sync_type sync_type, uint64_t ts_millisec,
                               uint32_t timeout_ticks = portMAX_DELAY, bool force_flush = false);
    esp_err_t flush_write(uint32_t timeout_ticks = portMAX_DELAY);
    esp_err_t deinit(bool force = false);

    [[nodiscard]] uint64_t get_newest_entry_id() const;
    [[nodiscard]] uint32_t get_boot_counter() const;
    [[nodiscard]] uint64_t get_used_size() const;

private:
    esp_err_t setup_store_file();
    esp_err_t open_store_file(const char *path);
    esp_err_t prepare_staging_file();
    esp_err_t publish_staging_file();
    esp_err_t write_store_headers_unsafe();
    esp_err_t load_store_header_unsafe();
    esp_err_t load_metadata();
    esp_err_t recover_from_entries_unsafe();
    esp_err_t initialise_empty_metadata_unsafe();
    esp_err_t commit_metadata_unsafe();
    esp_err_t persist_state_unsafe();
    esp_err_t increase_boot_counter();
    esp_err_t append_boot_entry_unsafe();
    esp_err_t acquire_operation_lock(uint32_t timeout_ticks) const;
    void release_operation_lock() const;
    void close_storage_unsafe();

private: // These APIs require write_lock to be held
    esp_err_t flush_unsafe();
    esp_err_t append_entry_unsafe(uint16_t type, const uint8_t *payload, uint32_t payload_len,
                                   on9rstore_def::entry_header *entry_info_out, bool force_flush);
    esp_err_t prepare_entry_space_unsafe(uint64_t entry_size);
    esp_err_t discard_until_free_unsafe(uint64_t required_size);
    esp_err_t discard_oldest_entry_unsafe();
    esp_err_t resynchronise_oldest_unsafe();
    esp_err_t wrap_write_position_unsafe(uint64_t tail_size);
    esp_err_t reserve_direct_entry_unsafe(uint16_t type, uint32_t payload_len,
                                           on9rstore_def::entry_header *header_out,
                                           uint64_t *entry_offset_out, uint64_t *entry_size_out);
    esp_err_t write_entry_trailer_unsafe(uint64_t entry_offset, uint32_t payload_len, uint32_t crc,
                                          uint64_t entry_size);
    esp_err_t finish_direct_entry_unsafe(uint64_t entry_offset, uint64_t entry_size);
    esp_err_t write_zeroes_unsafe(uint64_t offset, uint64_t len);
    esp_err_t read_entry_header_unsafe(uint64_t offset, on9rstore_def::entry_header *header_out);
    esp_err_t validate_entry_unsafe(uint64_t offset, const on9rstore_def::entry_header &header, uint64_t entry_size);
    bool is_store_header_valid(const on9rstore_def::store_header &candidate) const;
    bool is_metadata_valid(const on9rstore_def::metadata &candidate) const;
    bool is_metadata_geometry_valid(const on9rstore_def::metadata &candidate) const;
    bool is_entry_header_valid(const on9rstore_def::entry_header &header) const;

private:
    esp_err_t read_exact(uint64_t offset, void *buf_out, size_t len) const;
    esp_err_t write_exact(uint64_t offset, const void *buf, size_t len) const;
    esp_err_t sync_file() const;
    static uint32_t calc_crc32(const uint8_t *buf, size_t len);
    static uint32_t calc_crc32_update(uint32_t crc, const uint8_t *buf, size_t len);
    static uint64_t get_entry_size(uint32_t payload_len);
    static bool is_entry_id_valid(uint64_t entry_id);
    uint64_t make_next_entry_id_unsafe();
    uint64_t data_file_offset(uint64_t data_offset) const;

private:
    const char *file_path = nullptr;
    on9rstore_cfg cfg = {};
    char staging_file_path[PATH_MAX] = {};

    int fd = -1;
    SemaphoreHandle_t lifecycle_lock = nullptr;
    SemaphoreHandle_t write_lock = nullptr;
    std::unique_ptr<uint8_t[]> write_buf;
    size_t write_buf_pos = 0;
    uint64_t write_buf_offset = 0;
    uint64_t data_size = 0;
    uint16_t store_id = 0;
    uint32_t metadata_slot = on9rstore_def::metadata_slot_count - 1;
    on9rstore_def::metadata state = {};
    uint64_t newest_entry_id = 0;
    bool initialized = false;
    bool shutting_down = false;
    bool staging_file_active = false;
    bool metadata_dirty = false;

private:
    static const constexpr char TAG[] = "on9rstore";
};

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
    size_t write_buffer_size = 8192;
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
    esp_err_t append_time_anchor(const on9rstore_def::time_anchor &anchor,
                                 uint32_t timeout_ticks = portMAX_DELAY);
    esp_err_t flush_write(uint32_t timeout_ticks = portMAX_DELAY);
    esp_err_t deinit(bool force = false);

    [[nodiscard]] uint64_t get_newest_entry_id() const;
    [[nodiscard]] uint32_t get_boot_counter() const;
    [[nodiscard]] uint64_t get_used_size() const;
    [[nodiscard]] uint32_t get_time_anchor_count() const;

private:
    struct segment_descriptor {
        bool valid = false;
        bool sealed = false;
        uint32_t slot = 0;
        uint64_t generation = 0;
        uint64_t first_entry_id = 0;
        uint64_t last_entry_id = 0;
        uint64_t entry_count = 0;
        uint64_t data_end = 0;
        uint32_t index_count = 0;
    };

private:
    esp_err_t create_locks();
    esp_err_t validate_init_args() const;
    esp_err_t allocate_runtime_buffers();
    esp_err_t initialise_storage();
    esp_err_t finish_initialisation();
    void reset_runtime_state();

private: // Manifest operations
    esp_err_t setup_manifest();
    esp_err_t create_manifest();
    esp_err_t open_existing_manifest();
    esp_err_t initialise_manifest_superblock();
    esp_err_t load_manifest_superblock();
    esp_err_t commit_manifest_superblock_unsafe();
    esp_err_t write_initial_manifest_copies();
    esp_err_t recover_time_anchor_ring();
    esp_err_t append_time_anchor_unsafe(const on9rstore_def::time_anchor &anchor);
    esp_err_t write_time_anchor_slot(const on9rstore_def::time_anchor_entry &entry,
                                     uint32_t slot);
    esp_err_t read_time_anchor_slot(on9rstore_def::time_anchor_entry *entry_out,
                                    uint32_t slot) const;
    bool is_manifest_superblock_valid(const on9rstore_def::manifest_superblock &candidate) const;
    bool is_time_anchor_valid(const on9rstore_def::time_anchor_entry &candidate) const;
    bool is_time_anchor_input_valid(const on9rstore_def::time_anchor &anchor) const;
    uint64_t get_manifest_file_size(uint32_t anchor_count) const;
    void apply_manifest_geometry();
    void log_kconfig_geometry_mismatch() const;

private: // Segment operations
    esp_err_t provision_all_segments();
    esp_err_t provision_one_segment(uint32_t slot);
    esp_err_t initialise_empty_segment(int segment_fd, uint32_t slot);
    esp_err_t recover_all_segments();
    esp_err_t recover_one_segment(uint32_t slot);
    esp_err_t recover_open_segment(uint32_t slot,
                                   const on9rstore_def::segment_header &header);
    esp_err_t seal_recovered_segment(uint32_t slot);
    esp_err_t select_or_create_active_segment();
    esp_err_t open_active_segment(uint32_t slot);
    esp_err_t open_next_segment_unsafe();
    esp_err_t retire_segment_unsafe(uint32_t slot);
    esp_err_t activate_segment_unsafe(uint32_t slot, uint64_t generation);
    esp_err_t seal_active_segment_unsafe();
    esp_err_t write_active_sparse_index_unsafe();
    esp_err_t write_segment_headers(int segment_fd,
                                    const on9rstore_def::segment_header &header);
    esp_err_t write_segment_footers(int segment_fd,
                                    const on9rstore_def::segment_footer &footer);
    esp_err_t load_segment_header(int segment_fd, uint32_t slot,
                                  on9rstore_def::segment_header *header_out) const;
    esp_err_t load_segment_footer(int segment_fd,
                                  const on9rstore_def::segment_header &header,
                                  on9rstore_def::segment_footer *footer_out) const;
    esp_err_t scan_segment_entries(int segment_fd,
                                   const on9rstore_def::segment_header &header,
                                   segment_descriptor *descriptor_out,
                                   bool build_index);
    esp_err_t scan_one_entry(int segment_fd,
                             const on9rstore_def::segment_header &segment,
                             uint64_t offset,
                             on9rstore_def::entry_header *header_out,
                             uint64_t *entry_size_out) const;
    bool is_segment_header_valid(const on9rstore_def::segment_header &candidate,
                                 uint32_t slot) const;
    bool is_segment_footer_valid(const on9rstore_def::segment_footer &candidate,
                                 const on9rstore_def::segment_header &header) const;
    bool calculate_segment_geometry(uint64_t segment_size,
                                    on9rstore_def::segment_header *geometry_out) const;
    void update_recovered_manifest_state();
    void update_oldest_segment_generation();

private: // Entry operations; write_lock must be held
    esp_err_t append_boot_entry_unsafe();
    esp_err_t append_coredump_entry_unsafe(
        const void *partition, size_t partition_offset, size_t coredump_size,
        uint32_t coredump_crc, const on9rstore_def::boot_event &event);
    esp_err_t calculate_coredump_crc_unsafe(
        const void *partition, size_t partition_offset, size_t coredump_size,
        uint32_t *crc_out) const;
    esp_err_t append_entry_unsafe(uint16_t type, const uint8_t *payload,
                                  uint32_t payload_len,
                                  on9rstore_def::entry_header *entry_info_out,
                                  bool force_flush);
    esp_err_t append_buffered_entry_unsafe(uint16_t type, const uint8_t *payload,
                                           uint32_t payload_len,
                                           on9rstore_def::entry_header *entry_info_out,
                                           bool force_flush);
    esp_err_t append_direct_entry_unsafe(uint16_t type, const uint8_t *payload,
                                         uint32_t payload_len,
                                         on9rstore_def::entry_header *entry_info_out);
    esp_err_t prepare_entry_space_unsafe(uint64_t entry_size);
    esp_err_t rotate_active_segment_unsafe();
    esp_err_t flush_unsafe();
    esp_err_t write_entry_trailer_unsafe(uint64_t entry_offset,
                                         uint32_t payload_len, uint32_t crc,
                                         uint64_t entry_size);
    esp_err_t write_zeroes_unsafe(int file_fd, uint64_t file_size,
                                  uint64_t offset, uint64_t len) const;
    esp_err_t build_entry_header_unsafe(uint16_t type, uint32_t payload_len,
                                        on9rstore_def::entry_header *header_out);
    void account_appended_entry_unsafe(const on9rstore_def::entry_header &header,
                                       uint64_t offset, uint64_t entry_size);
    void add_sparse_index_entry_unsafe(const on9rstore_def::entry_header &header,
                                       uint64_t offset);
    uint64_t make_next_entry_id_unsafe();
    static uint64_t get_entry_size(uint32_t payload_len);
    static bool is_entry_id_valid(uint64_t entry_id);

private: // File operations
    esp_err_t build_manifest_path();
    esp_err_t build_data_path(uint32_t slot, char *path_out,
                              size_t path_out_len) const;
    esp_err_t provision_contiguous_file(const char *path, uint64_t size,
                                        bool *created_out) const;
    esp_err_t validate_contiguous_file(const char *path, uint64_t size) const;
    esp_err_t open_file(const char *path, int *fd_out) const;
    esp_err_t open_reader_segment(uint32_t slot);
    esp_err_t read_exact_fd(int file_fd, uint64_t file_size, uint64_t offset,
                            void *buf_out, size_t len) const;
    esp_err_t write_exact_fd(int file_fd, uint64_t file_size, uint64_t offset,
                             const void *buf, size_t len) const;
    esp_err_t sync_fd(int file_fd) const;
    void close_reader_segment();
    void close_writer_segment();
    void close_storage_unsafe();

private: // Locking and utilities
    esp_err_t acquire_operation_lock(uint32_t timeout_ticks) const;
    void release_operation_lock() const;
    static uint32_t calc_crc32(const uint8_t *buf, size_t len);
    static uint32_t calc_crc32_update(uint32_t crc, const uint8_t *buf, size_t len);
    static uint8_t count_bits(uint32_t value);

private:
    const char *file_path = nullptr;
    on9rstore_cfg cfg = {};
    char manifest_path[PATH_MAX] = {};
    char active_data_path[PATH_MAX] = {};

    int manifest_fd = -1;
    int reader_fd = -1;
    int writer_fd = -1;
    uint64_t manifest_file_size = 0;
    uint64_t segment_file_size = 0;
    uint32_t segment_count = 0;
    uint32_t time_anchor_count = 0;
    uint16_t store_id = 0;
    uint32_t manifest_slot = on9rstore_def::manifest_superblock_slot_count - 1;
    on9rstore_def::manifest_superblock state = {};
    on9rstore_def::segment_header active_segment = {};

    std::unique_ptr<segment_descriptor[]> segments;
    std::unique_ptr<on9rstore_def::sparse_index_entry[]> sparse_index;
    uint32_t sparse_index_count = 0;
    uint32_t sparse_index_capacity = 0;

    std::unique_ptr<uint8_t[]> write_buf;
    size_t write_buf_pos = 0;
    uint64_t write_buf_offset = 0;
    uint64_t active_write_offset = 0;
    uint64_t newest_entry_id = 0;

    SemaphoreHandle_t lifecycle_lock = nullptr;
    SemaphoreHandle_t read_lock = nullptr;
    SemaphoreHandle_t write_lock = nullptr;
    bool initialized = false;
    bool shutting_down = false;
    bool manifest_created = false;

private:
    static const constexpr char TAG[] = "on9rstore";
};

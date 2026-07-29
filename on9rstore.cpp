#include <cstring>
#include <inttypes.h>
#include <limits>
#include <new>
#include <unistd.h>

#include <esp_log.h>
#include <sdkconfig.h>

#include "on9rstore.hpp"

namespace
{
    static const constexpr uint32_t crc32_polynomial = 0xedb88320UL;

    constexpr uint32_t make_crc32_lut_entry(uint32_t value)
    {
        for (uint8_t bit = 0; bit < 8; bit += 1) {
            value = (value & 1U) ? ((value >> 1U) ^ crc32_polynomial) : (value >> 1U);
        }

        return value;
    }

    struct crc32_lut {
        uint32_t values[256] = {};

        constexpr crc32_lut()
        {
            for (size_t idx = 0; idx < 256; idx += 1) {
                values[idx] = make_crc32_lut_entry(static_cast<uint32_t>(idx));
            }
        }
    };

    static const constexpr crc32_lut crc32_table = {};

    constexpr uint32_t crc32_constexpr(const char *buf, size_t len)
    {
        uint32_t crc = UINT32_MAX;
        for (size_t idx = 0; idx < len; idx += 1) {
            const uint8_t table_idx = static_cast<uint8_t>(crc ^ static_cast<uint8_t>(buf[idx]));
            crc = crc32_table.values[table_idx] ^ (crc >> 8U);
        }

        return ~crc;
    }

    static_assert(crc32_constexpr("123456789", 9) == 0xcbf43926UL);
}

on9rstore::on9rstore(const char *_file_path, on9rstore_cfg *config) : file_path(_file_path)
{
    if (config != nullptr) {
        cfg = *config;
    }
}

on9rstore::~on9rstore()
{
    if (deinit(false) != ESP_OK) {
        (void)deinit(true);
    }

    if (write_lock != nullptr) {
        vSemaphoreDelete(write_lock);
        write_lock = nullptr;
    }

    if (read_lock != nullptr) {
        vSemaphoreDelete(read_lock);
        read_lock = nullptr;
    }

    if (lifecycle_lock != nullptr) {
        vSemaphoreDelete(lifecycle_lock);
        lifecycle_lock = nullptr;
    }
}

esp_err_t on9rstore::create_locks()
{
    if (lifecycle_lock == nullptr) {
        lifecycle_lock = xSemaphoreCreateMutex();
        if (lifecycle_lock == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (write_lock == nullptr) {
        write_lock = xSemaphoreCreateMutex();
        if (write_lock == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (read_lock == nullptr) {
        read_lock = xSemaphoreCreateMutex();
        if (read_lock == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}

esp_err_t on9rstore::validate_init_args() const
{
    if (file_path == nullptr || file_path[0] != '/' || cfg.write_buffer_size < on9rstore_def::min_entry_size) {
        return ESP_ERR_INVALID_ARG;
    }

    if (CONFIG_ON9RSTORE_SPARSE_FILE_SIZE > static_cast<uint64_t>(std::numeric_limits<off_t>::max())) {
        return ESP_ERR_INVALID_SIZE;
    }

    on9rstore_def::segment_header geometry = {};
    if (!calculate_segment_geometry(CONFIG_ON9RSTORE_SPARSE_FILE_SIZE, &geometry)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

esp_err_t on9rstore::allocate_runtime_buffers()
{
    write_buf.reset(new (std::nothrow) uint8_t[cfg.write_buffer_size]);
    if (write_buf == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    read_buf.reset(new (std::nothrow) uint8_t[cfg.write_buffer_size]);
    if (read_buf == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    segments.reset(new (std::nothrow) segment_descriptor[segment_count]);
    if (segments == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    read_segments.reset(new (std::nothrow) segment_descriptor[segment_count]);
    if (read_segments == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    on9rstore_def::segment_header geometry = {};
    if (!calculate_segment_geometry(segment_file_size, &geometry)) {
        return ESP_ERR_INVALID_SIZE;
    }

    sparse_index_capacity = geometry.index_capacity;
    sparse_index.reset(new (std::nothrow) on9rstore_def::sparse_index_entry[sparse_index_capacity]);
    if (sparse_index == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    read_sparse_index.reset(new (std::nothrow) on9rstore_def::sparse_index_entry[sparse_index_capacity]);
    if (read_sparse_index == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t on9rstore::initialise_storage()
{
    esp_err_t ret = build_manifest_path();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = setup_manifest();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = allocate_runtime_buffers();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = provision_all_segments();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = recover_all_segments();
    if (ret != ESP_OK) {
        return ret;
    }

    return recover_time_anchor_ring();
}

esp_err_t on9rstore::finish_initialisation()
{
    if (state.boot_counter >= on9rstore_def::entry_id_boot_mask) {
        return ESP_ERR_INVALID_STATE;
    }

    state.boot_counter += 1;
    state.next_entry_sequence = 0;
    esp_err_t ret = commit_manifest_superblock_unsafe();
    if (ret != ESP_OK) {
        return ret;
    }

    initialized = true;
    if (xSemaphoreTake(write_lock, portMAX_DELAY) != pdTRUE) {
        initialized = false;
        return ESP_ERR_TIMEOUT;
    }

    ret = append_boot_entry_unsafe();
    xSemaphoreGive(write_lock);
    if (ret != ESP_OK) {
        initialized = false;
    }

    return ret;
}

esp_err_t on9rstore::init()
{
    esp_err_t ret = create_locks();
    if (ret != ESP_OK) {
        return ret;
    }

    if (xSemaphoreTake(lifecycle_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (initialized || shutting_down) {
        xSemaphoreGive(lifecycle_lock);
        return ESP_ERR_INVALID_STATE;
    }

    ret = validate_init_args();
    if (ret == ESP_OK) {
        ret = initialise_storage();
    }
    if (ret == ESP_OK) {
        ret = finish_initialisation();
    }

    if (ret != ESP_OK) {
        initialized = false;
        close_storage_unsafe();
    } else {
        ESP_LOGI(TAG, "Init: boot=%" PRIu32 ", files=%" PRIu32 ", file_size=%" PRIu64 ", anchors=%" PRIu32, state.boot_counter,
                 segment_count, segment_file_size, time_anchor_count);
    }

    xSemaphoreGive(lifecycle_lock);
    return ret;
}

esp_err_t on9rstore::acquire_operation_lock(uint32_t timeout_ticks) const
{
    if (lifecycle_lock == nullptr || xSemaphoreTake(lifecycle_lock, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized || shutting_down || write_lock == nullptr) {
        xSemaphoreGive(lifecycle_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(write_lock, timeout_ticks) != pdTRUE) {
        xSemaphoreGive(lifecycle_lock);
        return ESP_ERR_TIMEOUT;
    }

    xSemaphoreGive(lifecycle_lock);
    return ESP_OK;
}

void on9rstore::release_operation_lock() const
{
    xSemaphoreGive(write_lock);
}

uint64_t on9rstore::get_newest_entry_id() const
{
    if (acquire_operation_lock(portMAX_DELAY) != ESP_OK) {
        return 0;
    }

    const uint64_t result = newest_entry_id;
    release_operation_lock();
    return result;
}

uint32_t on9rstore::get_boot_counter() const
{
    if (acquire_operation_lock(portMAX_DELAY) != ESP_OK) {
        return 0;
    }

    const uint32_t result = state.boot_counter;
    release_operation_lock();
    return result;
}

uint64_t on9rstore::get_used_size() const
{
    if (acquire_operation_lock(portMAX_DELAY) != ESP_OK) {
        return 0;
    }

    const uint64_t result = state.used_size;
    release_operation_lock();
    return result;
}

uint32_t on9rstore::get_time_anchor_count() const
{
    if (acquire_operation_lock(portMAX_DELAY) != ESP_OK) {
        return 0;
    }

    const uint32_t result = time_anchor_count;
    release_operation_lock();
    return result;
}

void on9rstore::reset_runtime_state()
{
    write_buf.reset();
    read_buf.reset();
    segments.reset();
    read_segments.reset();
    sparse_index.reset();
    read_sparse_index.reset();
    write_buf_pos = 0;
    write_buf_offset = 0;
    active_write_offset = 0;
    sparse_index_count = 0;
    sparse_index_capacity = 0;
    read_sparse_index_count = 0;
    read_buf_pos = 0;
    read_buf_offset = 0;
    manifest_file_size = 0;
    segment_file_size = 0;
    segment_count = 0;
    time_anchor_count = 0;
    store_id = 0;
    newest_entry_id = 0;
    manifest_created = false;
    manifest_path[0] = '\0';
    active_data_path[0] = '\0';
    state = {};
    active_segment = {};
    read_active_segment = {};
}

void on9rstore::close_storage_unsafe()
{
    close_reader_segment();
    close_writer_segment();
    if (manifest_fd >= 0) {
        (void)close(manifest_fd);
        manifest_fd = -1;
    }

    reset_runtime_state();
}

esp_err_t on9rstore::deinit(bool force)
{
    if (lifecycle_lock == nullptr) {
        return ESP_OK;
    }

    if (xSemaphoreTake(lifecycle_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized) {
        xSemaphoreGive(lifecycle_lock);
        return ESP_OK;
    }

    shutting_down = true;
    if (xSemaphoreTake(write_lock, portMAX_DELAY) != pdTRUE) {
        shutting_down = false;
        xSemaphoreGive(lifecycle_lock);
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(read_lock, portMAX_DELAY) != pdTRUE) {
        xSemaphoreGive(write_lock);
        shutting_down = false;
        xSemaphoreGive(lifecycle_lock);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = force ? ESP_OK : flush_unsafe();
    if (ret == ESP_OK || force) {
        initialized = false;
        close_storage_unsafe();
    }

    xSemaphoreGive(read_lock);
    xSemaphoreGive(write_lock);
    shutting_down = false;
    xSemaphoreGive(lifecycle_lock);
    return ret;
}

uint32_t on9rstore::calc_crc32(const uint8_t *buf, size_t len)
{
    return ~calc_crc32_update(UINT32_MAX, buf, len);
}

uint32_t on9rstore::calc_crc32_update(uint32_t crc, const uint8_t *buf, size_t len)
{
    for (size_t idx = 0; idx < len; idx += 1) {
        const uint8_t table_idx = static_cast<uint8_t>(crc ^ buf[idx]);
        crc = crc32_table.values[table_idx] ^ (crc >> 8U);
    }

    return crc;
}

uint8_t on9rstore::count_bits(uint32_t value)
{
    uint8_t count = 0;
    while (value != 0) {
        count += static_cast<uint8_t>(value & 1U);
        value >>= 1U;
    }

    return count;
}

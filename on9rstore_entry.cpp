#include <cstring>
#include <inttypes.h>

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <sdkconfig.h>

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#include <esp_core_dump.h>
#include <esp_partition.h>
#endif

#include "on9rstore.hpp"

uint64_t on9rstore::get_entry_size(uint32_t payload_len)
{
    const uint64_t base_size =
        sizeof(on9rstore_def::entry_header) + static_cast<uint64_t>(payload_len) + on9rstore_def::entry_crc_len;
    if (base_size > SIZE_MAX - (on9rstore_def::entry_alignment - 1)) {
        return 0;
    }

    return on9rstore_def::align_up(static_cast<size_t>(base_size), on9rstore_def::entry_alignment);
}

bool on9rstore::is_entry_id_valid(uint64_t entry_id)
{
    return entry_id != 0;
}

uint64_t on9rstore::make_next_entry_id_unsafe()
{
    if (state.next_entry_sequence >= on9rstore_def::entry_id_sequence_mask) {
        return 0;
    }

    state.next_entry_sequence += 1;
    newest_entry_id = (static_cast<uint64_t>(state.boot_counter) << 40ULL) | state.next_entry_sequence;
    state.newest_entry_id = newest_entry_id;
    return newest_entry_id;
}

esp_err_t on9rstore::build_entry_header_unsafe(uint16_t type, uint32_t payload_len, on9rstore_def::entry_header *header_out)
{
    if (header_out == nullptr || active_segment.generation == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint64_t entry_id = make_next_entry_id_unsafe();
    if (!is_entry_id_valid(entry_id)) {
        return ESP_ERR_INVALID_STATE;
    }

    on9rstore_def::entry_header header = {};
    header.magic = on9rstore_def::entry_magic;
    header.revision = on9rstore_def::entry_revision;
    header.type = type;
    header.entry_id = entry_id;
    header.uptime_us = static_cast<uint64_t>(esp_timer_get_time());
    header.len = payload_len;
    header.store_id = store_id;
    header.segment_slot = static_cast<uint16_t>(active_segment.slot);
    header.segment_generation = active_segment.generation;
    *header_out = header;
    return ESP_OK;
}

void on9rstore::add_sparse_index_entry_unsafe(const on9rstore_def::entry_header &header, uint64_t offset)
{
    if (sparse_index_count >= sparse_index_capacity || offset > UINT32_MAX) {
        return;
    }

    on9rstore_def::sparse_index_entry &index = sparse_index[sparse_index_count];
    index = {};
    index.entry_id = header.entry_id;
    index.uptime_us = header.uptime_us;
    index.offset = static_cast<uint32_t>(offset);
    index.type = header.type;
    sparse_index_count += 1;
}

void on9rstore::account_appended_entry_unsafe(const on9rstore_def::entry_header &header, uint64_t offset, uint64_t entry_size)
{
    segment_descriptor &descriptor = segments[active_segment.slot];
    if (descriptor.entry_count % active_segment.index_stride == 0) {
        add_sparse_index_entry_unsafe(header, offset);
    }

    if (descriptor.entry_count == 0) {
        descriptor.first_entry_id = header.entry_id;
    }
    descriptor.last_entry_id = header.entry_id;
    descriptor.entry_count += 1;
    descriptor.data_end = offset + entry_size;
    descriptor.index_count = sparse_index_count;

    active_write_offset = descriptor.data_end;
    state.used_size += entry_size;
    state.newest_entry_id = header.entry_id;
    newest_entry_id = header.entry_id;
}

esp_err_t on9rstore::prepare_entry_space_unsafe(uint64_t entry_size)
{
    const uint64_t capacity = active_segment.data_end - active_segment.data_start;
    if (entry_size > capacity) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (entry_size <= active_segment.data_end - active_write_offset) {
        return ESP_OK;
    }

    return rotate_active_segment_unsafe();
}

esp_err_t on9rstore::rotate_active_segment_unsafe()
{
    esp_err_t ret = seal_active_segment_unsafe();
    if (ret != ESP_OK) {
        return ret;
    }

    return open_next_segment_unsafe();
}

esp_err_t on9rstore::write_zeroes_unsafe(int file_fd, uint64_t file_size, uint64_t offset, uint64_t len) const
{
    uint8_t zeroes[256] = {};
    while (len > 0) {
        const size_t chunk_len = len < sizeof(zeroes) ? static_cast<size_t>(len) : sizeof(zeroes);
        esp_err_t ret = write_exact_fd(file_fd, file_size, offset, zeroes, chunk_len);
        if (ret != ESP_OK) {
            return ret;
        }

        offset += chunk_len;
        len -= chunk_len;
    }

    return ESP_OK;
}

esp_err_t on9rstore::write_entry_trailer_unsafe(uint64_t entry_offset, uint32_t payload_len, uint32_t crc, uint64_t entry_size)
{
    const uint64_t crc_offset = entry_offset + sizeof(on9rstore_def::entry_header) + payload_len;
    esp_err_t ret = write_exact_fd(writer_fd, segment_file_size, crc_offset, &crc, sizeof(crc));
    if (ret != ESP_OK) {
        return ret;
    }

    const uint64_t written = sizeof(on9rstore_def::entry_header) + static_cast<uint64_t>(payload_len) + sizeof(crc);
    if (entry_size < written) {
        return ESP_ERR_INVALID_SIZE;
    }

    return write_zeroes_unsafe(writer_fd, segment_file_size, entry_offset + written, entry_size - written);
}

esp_err_t on9rstore::append_buffered_entry_unsafe(uint16_t type, const uint8_t *payload, uint32_t payload_len,
                                                  on9rstore_def::entry_header *entry_info_out, bool force_flush)
{
    const uint64_t entry_size = get_entry_size(payload_len);
    if (write_buf_pos + entry_size > cfg.write_buffer_size) {
        esp_err_t ret = flush_unsafe();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    esp_err_t ret = prepare_entry_space_unsafe(entry_size);
    if (ret != ESP_OK) {
        return ret;
    }

    on9rstore_def::entry_header header = {};
    ret = build_entry_header_unsafe(type, payload_len, &header);
    if (ret != ESP_OK) {
        return ret;
    }

    const uint64_t entry_offset = active_write_offset;
    if (write_buf_pos == 0) {
        write_buf_offset = entry_offset;
    }

    uint8_t *entry = write_buf.get() + write_buf_pos;
    memset(entry, 0, static_cast<size_t>(entry_size));
    memcpy(entry, &header, sizeof(header));
    if (payload_len > 0) {
        memcpy(entry + sizeof(header), payload, payload_len);
    }

    const uint32_t checksum = calc_crc32(entry, sizeof(header) + payload_len);
    memcpy(entry + sizeof(header) + payload_len, &checksum, sizeof(checksum));

    write_buf_pos += static_cast<size_t>(entry_size);
    account_appended_entry_unsafe(header, entry_offset, entry_size);
    if (entry_info_out != nullptr) {
        *entry_info_out = header;
    }

    return force_flush ? flush_unsafe() : ESP_OK;
}

esp_err_t on9rstore::append_direct_entry_unsafe(uint16_t type, const uint8_t *payload, uint32_t payload_len,
                                                on9rstore_def::entry_header *entry_info_out)
{
    esp_err_t ret = flush_unsafe();
    if (ret != ESP_OK) {
        return ret;
    }

    const uint64_t entry_size = get_entry_size(payload_len);
    ret = prepare_entry_space_unsafe(entry_size);
    if (ret != ESP_OK) {
        return ret;
    }

    on9rstore_def::entry_header header = {};
    ret = build_entry_header_unsafe(type, payload_len, &header);
    if (ret != ESP_OK) {
        return ret;
    }

    const uint64_t entry_offset = active_write_offset;
    ret = write_exact_fd(writer_fd, segment_file_size, entry_offset, &header, sizeof(header));
    uint32_t crc = calc_crc32_update(UINT32_MAX, reinterpret_cast<const uint8_t *>(&header), sizeof(header));
    if (ret == ESP_OK && payload_len > 0) {
        ret = write_exact_fd(writer_fd, segment_file_size, entry_offset + sizeof(header), payload, payload_len);
        crc = calc_crc32_update(crc, payload, payload_len);
    }
    if (ret == ESP_OK) {
        ret = write_entry_trailer_unsafe(entry_offset, payload_len, ~crc, entry_size);
    }
    if (ret == ESP_OK) {
        ret = sync_fd(writer_fd);
    }
    if (ret != ESP_OK) {
        return ret;
    }

    account_appended_entry_unsafe(header, entry_offset, entry_size);
    ret = commit_manifest_superblock_unsafe();
    if (ret == ESP_OK && entry_info_out != nullptr) {
        *entry_info_out = header;
    }

    return ret;
}

esp_err_t on9rstore::append_entry_unsafe(uint16_t type, const uint8_t *payload, uint32_t payload_len,
                                         on9rstore_def::entry_header *entry_info_out, bool force_flush)
{
    const uint64_t entry_size = get_entry_size(payload_len);
    if (entry_size == 0 || entry_size > active_segment.data_end - active_segment.data_start) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (entry_size > cfg.write_buffer_size) {
        return append_direct_entry_unsafe(type, payload, payload_len, entry_info_out);
    }

    return append_buffered_entry_unsafe(type, payload, payload_len, entry_info_out, force_flush);
}

esp_err_t on9rstore::append_entry(uint16_t type, const uint8_t *payload, size_t payload_len,
                                  on9rstore_def::entry_header *entry_info_out, uint32_t timeout_ticks, bool force_flush)
{
    if ((payload == nullptr && payload_len > 0) || payload_len > UINT32_MAX || type >= on9rstore_def::ENTRY_RESERVED_START) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = acquire_operation_lock(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = append_entry_unsafe(type, payload, static_cast<uint32_t>(payload_len), entry_info_out, force_flush);
    release_operation_lock();
    return ret;
}

esp_err_t on9rstore::flush_unsafe()
{
    if (write_buf_pos == 0) {
        return ESP_OK;
    }

    esp_err_t ret = write_exact_fd(writer_fd, segment_file_size, write_buf_offset, write_buf.get(), write_buf_pos);
    if (ret == ESP_OK) {
        ret = sync_fd(writer_fd);
    }
    if (ret == ESP_OK) {
        ret = commit_manifest_superblock_unsafe();
    }
    if (ret != ESP_OK) {
        return ret;
    }

    write_buf_pos = 0;
    write_buf_offset = active_write_offset;
    return ESP_OK;
}

esp_err_t on9rstore::flush_write(uint32_t timeout_ticks)
{
    esp_err_t ret = acquire_operation_lock(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = flush_unsafe();
    release_operation_lock();
    return ret;
}

esp_err_t on9rstore::calculate_coredump_crc_unsafe(const void *partition_ptr, size_t partition_offset, size_t coredump_size,
                                                   uint32_t *crc_out) const
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    if (partition_ptr == nullptr || crc_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const auto *partition = static_cast<const esp_partition_t *>(partition_ptr);
    uint8_t chunk[256] = {};
    uint32_t crc = UINT32_MAX;
    size_t remaining = coredump_size;
    size_t offset = partition_offset;
    while (remaining > 0) {
        const size_t chunk_len = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
        esp_err_t ret = esp_partition_read(partition, offset, chunk, chunk_len);
        if (ret != ESP_OK) {
            return ret;
        }

        crc = calc_crc32_update(crc, chunk, chunk_len);
        offset += chunk_len;
        remaining -= chunk_len;
    }

    *crc_out = ~crc;
    return ESP_OK;
#else
    (void)partition_ptr;
    (void)partition_offset;
    (void)coredump_size;
    (void)crc_out;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t on9rstore::append_coredump_entry_unsafe(const void *partition_ptr, size_t partition_offset, size_t coredump_size,
                                                  uint32_t coredump_crc, const on9rstore_def::boot_event &base_event)
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    const auto *partition = static_cast<const esp_partition_t *>(partition_ptr);
    on9rstore_def::boot_event event = base_event;
    event.coredump_len = static_cast<uint32_t>(coredump_size);
    const uint32_t payload_len = sizeof(event) + static_cast<uint32_t>(coredump_size);
    const uint64_t entry_size = get_entry_size(payload_len);

    esp_err_t ret = flush_unsafe();
    if (ret == ESP_OK) {
        ret = prepare_entry_space_unsafe(entry_size);
    }

    on9rstore_def::entry_header header = {};
    if (ret == ESP_OK) {
        ret = build_entry_header_unsafe(on9rstore_def::ENTRY_BOOT_EVENT, payload_len, &header);
    }

    const uint64_t entry_offset = active_write_offset;
    if (ret == ESP_OK) {
        ret = write_exact_fd(writer_fd, segment_file_size, entry_offset, &header, sizeof(header));
    }

    uint32_t entry_crc = calc_crc32_update(UINT32_MAX, reinterpret_cast<const uint8_t *>(&header), sizeof(header));
    if (ret == ESP_OK) {
        ret = write_exact_fd(writer_fd, segment_file_size, entry_offset + sizeof(header), &event, sizeof(event));
        entry_crc = calc_crc32_update(entry_crc, reinterpret_cast<const uint8_t *>(&event), sizeof(event));
    }

    if (ret == ESP_OK) {
        ret = stream_coredump_payload_unsafe(partition, partition_offset, coredump_size,
                                             entry_offset + sizeof(header) + sizeof(event), &entry_crc);
    }

    if (ret == ESP_OK) {
        ret = write_entry_trailer_unsafe(entry_offset, payload_len, ~entry_crc, entry_size);
    }
    if (ret == ESP_OK) {
        ret = sync_fd(writer_fd);
    }
    if (ret != ESP_OK) {
        return ret;
    }

    account_appended_entry_unsafe(header, entry_offset, entry_size);
    state.coredump_crc32 = coredump_crc;
    state.coredump_size = static_cast<uint32_t>(coredump_size);
    return commit_manifest_superblock_unsafe();
#else
    (void)partition_ptr;
    (void)partition_offset;
    (void)coredump_size;
    (void)coredump_crc;
    (void)base_event;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t on9rstore::stream_coredump_payload_unsafe(const void *partition_ptr, size_t partition_offset, size_t coredump_size,
                                                    uint64_t destination_offset, uint32_t *entry_crc) const
{
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    if (partition_ptr == nullptr || entry_crc == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const auto *partition = static_cast<const esp_partition_t *>(partition_ptr);
    uint8_t chunk[256] = {};
    size_t remaining = coredump_size;
    while (remaining > 0) {
        const size_t chunk_len = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
        esp_err_t ret = esp_partition_read(partition, partition_offset, chunk, chunk_len);
        if (ret == ESP_OK) {
            ret = write_exact_fd(writer_fd, segment_file_size, destination_offset, chunk, chunk_len);
        }
        if (ret != ESP_OK) {
            return ret;
        }

        *entry_crc = calc_crc32_update(*entry_crc, chunk, chunk_len);
        partition_offset += chunk_len;
        destination_offset += chunk_len;
        remaining -= chunk_len;
    }

    return ESP_OK;
#else
    (void)partition_ptr;
    (void)partition_offset;
    (void)coredump_size;
    (void)destination_offset;
    (void)entry_crc;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t on9rstore::append_boot_entry_unsafe()
{
    on9rstore_def::boot_event event = {};
    event.reset_reason = static_cast<uint32_t>(esp_reset_reason());

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    if (cfg.copy_coredump && esp_core_dump_image_check() == ESP_OK) {
        size_t address = 0;
        size_t size = 0;
        esp_err_t ret = esp_core_dump_image_get(&address, &size);
        const esp_partition_t *partition =
            esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
        if (ret == ESP_OK && partition != nullptr && size <= UINT32_MAX - sizeof(event) && address >= partition->address &&
            size <= partition->size - (address - partition->address)) {
            const size_t partition_offset = address - partition->address;
            uint32_t coredump_crc = 0;
            ret = calculate_coredump_crc_unsafe(partition, partition_offset, size, &coredump_crc);
            if (ret == ESP_OK && (state.coredump_size != size || state.coredump_crc32 != coredump_crc)) {
                ret = append_coredump_entry_unsafe(partition, partition_offset, size, coredump_crc, event);
                if (ret == ESP_OK) {
                    return ESP_OK;
                }
                ESP_LOGW(TAG, "Boot: coredump append failed: 0x%x", ret);
            }
        }
    }
#endif

    return append_entry_unsafe(on9rstore_def::ENTRY_BOOT_EVENT, reinterpret_cast<const uint8_t *>(&event), sizeof(event), nullptr,
                               true);
}

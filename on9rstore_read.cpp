#include <cstring>

#include "on9rstore.hpp"

esp_err_t on9rstore::acquire_read_operation_locks(
    uint32_t timeout_ticks) const
{
    if (lifecycle_lock == nullptr || write_lock == nullptr ||
        read_lock == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(lifecycle_lock, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized || shutting_down) {
        xSemaphoreGive(lifecycle_lock);
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(write_lock, timeout_ticks) != pdTRUE) {
        xSemaphoreGive(lifecycle_lock);
        return ESP_ERR_TIMEOUT;
    }

    if (xSemaphoreTake(read_lock, timeout_ticks) != pdTRUE) {
        xSemaphoreGive(write_lock);
        xSemaphoreGive(lifecycle_lock);
        return ESP_ERR_TIMEOUT;
    }

    xSemaphoreGive(lifecycle_lock);
    return ESP_OK;
}

void on9rstore::snapshot_read_state_unsafe()
{
    memcpy(read_segments.get(), segments.get(),
           sizeof(segment_descriptor) * segment_count);
    memcpy(read_sparse_index.get(), sparse_index.get(),
           sizeof(on9rstore_def::sparse_index_entry) *
               sparse_index_count);
    if (write_buf_pos > 0) {
        memcpy(read_buf.get(), write_buf.get(), write_buf_pos);
    }

    read_active_segment = active_segment;
    read_sparse_index_count = sparse_index_count;
    read_buf_pos = write_buf_pos;
    read_buf_offset = write_buf_offset;
}

void on9rstore::release_read_operation_lock() const
{
    xSemaphoreGive(read_lock);
}

bool on9rstore::find_read_segment(
    uint64_t first_entry_id, uint64_t last_entry_id, bool exact,
    segment_descriptor *descriptor_out) const
{
    if (descriptor_out == nullptr) {
        return false;
    }

    bool found = false;
    segment_descriptor selected = {};
    for (uint32_t slot = 0; slot < segment_count; slot += 1) {
        const segment_descriptor &candidate = read_segments[slot];
        if (!candidate.valid || candidate.entry_count == 0 ||
            candidate.last_entry_id < first_entry_id ||
            candidate.first_entry_id > last_entry_id) {
            continue;
        }
        if (exact &&
            (first_entry_id < candidate.first_entry_id ||
             first_entry_id > candidate.last_entry_id)) {
            continue;
        }

        if (!found || candidate.generation < selected.generation) {
            selected = candidate;
            found = true;
        }
    }

    if (found) {
        *descriptor_out = selected;
    }
    return found;
}

esp_err_t on9rstore::load_read_sparse_index(
    const segment_descriptor &descriptor,
    const on9rstore_def::segment_header &header)
{
    if (!descriptor.sealed) {
        if (descriptor.slot != read_active_segment.slot ||
            descriptor.generation != read_active_segment.generation ||
            descriptor.index_count != read_sparse_index_count) {
            return ESP_ERR_INVALID_STATE;
        }
        return ESP_OK;
    }

    on9rstore_def::segment_footer footer = {};
    esp_err_t ret = load_segment_footer(reader_fd, header, &footer);
    if (ret != ESP_OK) {
        return ret == ESP_ERR_NOT_FOUND ?
            ESP_ERR_INVALID_CRC : ret;
    }

    if (footer.first_entry_id != descriptor.first_entry_id ||
        footer.last_entry_id != descriptor.last_entry_id ||
        footer.entry_count != descriptor.entry_count ||
        footer.data_end != descriptor.data_end ||
        footer.index_count != descriptor.index_count) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = validate_segment_index(reader_fd, header, footer);
    if (ret != ESP_OK) {
        return ret;
    }

    if (footer.index_count > sparse_index_capacity) {
        return ESP_ERR_INVALID_SIZE;
    }

    const size_t index_size =
        static_cast<size_t>(footer.index_count) *
        sizeof(on9rstore_def::sparse_index_entry);
    ret = read_exact_fd(reader_fd, segment_file_size,
                        header.index_start, read_sparse_index.get(),
                        index_size);
    if (ret == ESP_OK) {
        read_sparse_index_count = footer.index_count;
    }
    return ret;
}

bool on9rstore::is_read_sparse_index_valid(
    const segment_descriptor &descriptor,
    const on9rstore_def::segment_header &header) const
{
    if (descriptor.entry_count == 0) {
        return read_sparse_index_count == 0;
    }

    const uint64_t expected_count =
        (descriptor.entry_count - 1) / header.index_stride + 1;
    if (expected_count != descriptor.index_count ||
        descriptor.index_count != read_sparse_index_count ||
        read_sparse_index_count == 0) {
        return false;
    }

    uint64_t previous_id = 0;
    uint32_t previous_offset = 0;
    for (uint32_t idx = 0; idx < read_sparse_index_count; idx += 1) {
        const on9rstore_def::sparse_index_entry &entry =
            read_sparse_index[idx];
        if (entry.reserved != 0 ||
            entry.entry_id < descriptor.first_entry_id ||
            entry.entry_id > descriptor.last_entry_id ||
            entry.offset < header.data_start ||
            entry.offset >= descriptor.data_end ||
            entry.offset % on9rstore_def::entry_alignment != 0 ||
            (idx > 0 &&
             (entry.entry_id <= previous_id ||
              entry.offset <= previous_offset))) {
            return false;
        }

        previous_id = entry.entry_id;
        previous_offset = entry.offset;
    }

    return read_sparse_index[0].entry_id ==
               descriptor.first_entry_id &&
           read_sparse_index[0].offset == header.data_start;
}

esp_err_t on9rstore::prepare_read_segment(
    const segment_descriptor &descriptor,
    on9rstore_def::segment_header *header_out)
{
    if (header_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = open_reader_segment(descriptor.slot);
    if (ret != ESP_OK) {
        return ret;
    }

    on9rstore_def::segment_header header = {};
    ret = load_segment_header(reader_fd, descriptor.slot, &header);
    if (ret != ESP_OK) {
        return ret;
    }
    if (header.generation != descriptor.generation) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = load_read_sparse_index(descriptor, header);
    if (ret != ESP_OK) {
        return ret;
    }
    if (!is_read_sparse_index_valid(descriptor, header)) {
        return ESP_ERR_INVALID_CRC;
    }

    *header_out = header;
    return ESP_OK;
}

void on9rstore::find_read_start(
    uint64_t entry_id,
    const on9rstore_def::segment_header &header,
    uint64_t *offset_out,
    on9rstore_def::sparse_index_entry *index_entry_out) const
{
    uint32_t low = 0;
    uint32_t high = read_sparse_index_count;
    while (low < high) {
        const uint32_t middle = low + (high - low) / 2;
        if (read_sparse_index[middle].entry_id <= entry_id) {
            low = middle + 1;
        } else {
            high = middle;
        }
    }

    const uint32_t selected = low == 0 ? 0 : low - 1;
    *offset_out = read_sparse_index_count == 0 ?
        header.data_start : read_sparse_index[selected].offset;
    *index_entry_out = read_sparse_index_count == 0 ?
        on9rstore_def::sparse_index_entry{} :
        read_sparse_index[selected];
}

esp_err_t on9rstore::read_snapshot_bytes(
    const on9rstore_def::segment_header &segment,
    uint64_t offset, void *buf_out, size_t len) const
{
    if (len == 0) {
        return ESP_OK;
    }
    if (buf_out == nullptr || len > segment_file_size ||
        offset > segment_file_size - len) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool active_snapshot =
        segment.slot == read_active_segment.slot &&
        segment.generation == read_active_segment.generation;
    if (active_snapshot && read_buf_pos > 0) {
        const uint64_t buffer_end =
            read_buf_offset + read_buf_pos;
        const uint64_t read_end = offset + len;
        if (offset >= read_buf_offset) {
            if (read_end > buffer_end) {
                return ESP_ERR_INVALID_SIZE;
            }

            memcpy(buf_out,
                   read_buf.get() +
                       static_cast<size_t>(offset - read_buf_offset),
                   len);
            return ESP_OK;
        }
        if (read_end > read_buf_offset) {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    return read_exact_fd(reader_fd, segment_file_size,
                         offset, buf_out, len);
}

esp_err_t on9rstore::read_snapshot_entry_header(
    const on9rstore_def::segment_header &segment,
    uint64_t entry_limit, uint64_t offset,
    on9rstore_def::entry_header *header_out,
    uint64_t *entry_size_out) const
{
    if (header_out == nullptr || entry_size_out == nullptr ||
        entry_limit < segment.data_start ||
        entry_limit > segment.data_end ||
        offset < segment.data_start ||
        offset > entry_limit ||
        entry_limit - offset < on9rstore_def::min_entry_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    on9rstore_def::entry_header header = {};
    esp_err_t ret = read_snapshot_bytes(
        segment, offset, &header, sizeof(header));
    if (ret != ESP_OK) {
        return ret;
    }

    if (header.magic != on9rstore_def::entry_magic ||
        header.revision != on9rstore_def::entry_revision ||
        header.store_id != store_id ||
        header.segment_slot != segment.slot ||
        header.segment_generation != segment.generation ||
        !is_entry_id_valid(header.entry_id)) {
        return ESP_ERR_INVALID_CRC;
    }

    const uint64_t entry_size = get_entry_size(header.len);
    if (entry_size == 0 || entry_size > entry_limit - offset) {
        return ESP_ERR_INVALID_SIZE;
    }

    *header_out = header;
    *entry_size_out = entry_size;
    return ESP_OK;
}

esp_err_t on9rstore::validate_snapshot_entry_payload(
    const on9rstore_def::segment_header &segment,
    uint64_t offset, const on9rstore_def::entry_header &header,
    uint8_t *payload_out, size_t payload_out_len,
    bool copy_payload) const
{
    const bool buffer_too_small =
        copy_payload &&
        (header.len > payload_out_len ||
         (header.len > 0 && payload_out == nullptr));
    uint32_t crc = calc_crc32_update(
        UINT32_MAX,
        reinterpret_cast<const uint8_t *>(&header),
        sizeof(header));

    uint8_t chunk[256] = {};
    uint64_t payload_offset = offset + sizeof(header);
    uint32_t copied = 0;
    uint32_t remaining = header.len;
    while (remaining > 0) {
        const size_t chunk_len =
            remaining < sizeof(chunk) ? remaining : sizeof(chunk);
        uint8_t *destination =
            copy_payload && !buffer_too_small ?
                payload_out + copied : chunk;
        esp_err_t ret = read_snapshot_bytes(
            segment, payload_offset, destination, chunk_len);
        if (ret != ESP_OK) {
            return ret;
        }

        crc = calc_crc32_update(crc, destination, chunk_len);
        payload_offset += chunk_len;
        copied += static_cast<uint32_t>(chunk_len);
        remaining -= static_cast<uint32_t>(chunk_len);
    }

    uint32_t expected_crc = 0;
    esp_err_t ret = read_snapshot_bytes(
        segment, payload_offset, &expected_crc, sizeof(expected_crc));
    if (ret != ESP_OK) {
        return ret;
    }
    if (~crc != expected_crc) {
        return ESP_ERR_INVALID_CRC;
    }

    return buffer_too_small ? ESP_ERR_INVALID_SIZE : ESP_OK;
}

esp_err_t on9rstore::read_matching_entry(
    const segment_descriptor &descriptor,
    const on9rstore_def::segment_header &header,
    uint64_t first_entry_id, uint64_t last_entry_id, bool exact,
    uint8_t *payload_out, size_t payload_out_len,
    on9rstore_def::entry_header *entry_info_out)
{
    uint64_t offset = 0;
    on9rstore_def::sparse_index_entry start_index = {};
    find_read_start(first_entry_id, header, &offset, &start_index);

    uint64_t previous_id = 0;
    bool first_scanned = true;
    while (offset < descriptor.data_end) {
        on9rstore_def::entry_header entry = {};
        uint64_t entry_size = 0;
        esp_err_t ret = read_snapshot_entry_header(
            header, descriptor.data_end, offset,
            &entry, &entry_size);
        if (ret != ESP_OK) {
            return ret;
        }

        const bool matches =
            exact ? entry.entry_id == first_entry_id :
                    entry.entry_id >= first_entry_id &&
                        entry.entry_id <= last_entry_id;
        ret = validate_snapshot_entry_payload(
            header, offset, entry, payload_out, payload_out_len,
            matches);
        if (ret != ESP_OK && !(matches &&
                              ret == ESP_ERR_INVALID_SIZE)) {
            return ret;
        }

        if ((first_scanned && start_index.entry_id != 0 &&
             (entry.entry_id != start_index.entry_id ||
              entry.uptime_us != start_index.uptime_us ||
              entry.type != start_index.type ||
              offset != start_index.offset)) ||
            (previous_id != 0 && entry.entry_id <= previous_id)) {
            return ESP_ERR_INVALID_CRC;
        }

        if (matches) {
            if (entry_info_out != nullptr) {
                *entry_info_out = entry;
            }
            return ret;
        }

        if (entry.entry_id > last_entry_id ||
            (exact && entry.entry_id > first_entry_id)) {
            return ESP_ERR_NOT_FOUND;
        }

        first_scanned = false;
        previous_id = entry.entry_id;
        offset += entry_size;
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t on9rstore::read_entry_internal(
    uint64_t first_entry_id, uint64_t last_entry_id, bool exact,
    uint8_t *payload_out, size_t payload_out_len,
    on9rstore_def::entry_header *entry_info_out,
    uint32_t timeout_ticks)
{
    esp_err_t ret = acquire_read_operation_locks(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    snapshot_read_state_unsafe();
    xSemaphoreGive(write_lock);

    segment_descriptor descriptor = {};
    if (!find_read_segment(first_entry_id, last_entry_id,
                           exact, &descriptor)) {
        release_read_operation_lock();
        return ESP_ERR_NOT_FOUND;
    }

    on9rstore_def::segment_header header = {};
    ret = prepare_read_segment(descriptor, &header);
    if (ret == ESP_OK) {
        ret = read_matching_entry(
            descriptor, header, first_entry_id, last_entry_id,
            exact, payload_out, payload_out_len, entry_info_out);
    }

    close_reader_segment();
    release_read_operation_lock();
    return ret;
}

esp_err_t on9rstore::read_entry(
    uint64_t entry_id, uint8_t *payload_out, size_t payload_out_len,
    on9rstore_def::entry_header *entry_info_out,
    uint32_t timeout_ticks)
{
    if (!is_entry_id_valid(entry_id) ||
        (payload_out == nullptr && payload_out_len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    return read_entry_internal(
        entry_id, entry_id, true, payload_out, payload_out_len,
        entry_info_out, timeout_ticks);
}

esp_err_t on9rstore::read_next_entry(
    on9rstore_def::entry_range_cursor *cursor,
    uint8_t *payload_out, size_t payload_out_len,
    on9rstore_def::entry_header *entry_info_out,
    uint32_t timeout_ticks)
{
    if (cursor == nullptr ||
        (payload_out == nullptr && payload_out_len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cursor->finished ||
        cursor->next_entry_id > cursor->last_entry_id) {
        cursor->finished = true;
        return ESP_ERR_NOT_FOUND;
    }

    on9rstore_def::entry_header entry = {};
    esp_err_t ret = read_entry_internal(
        cursor->next_entry_id, cursor->last_entry_id, false,
        payload_out, payload_out_len, &entry, timeout_ticks);
    if (ret == ESP_ERR_NOT_FOUND) {
        cursor->finished = true;
        return ret;
    }
    if (ret != ESP_OK) {
        if (entry_info_out != nullptr &&
            ret == ESP_ERR_INVALID_SIZE && entry.entry_id != 0) {
            *entry_info_out = entry;
        }
        return ret;
    }

    if (entry_info_out != nullptr) {
        *entry_info_out = entry;
    }
    if (entry.entry_id >= cursor->last_entry_id ||
        entry.entry_id == UINT64_MAX) {
        cursor->finished = true;
    } else {
        cursor->next_entry_id = entry.entry_id + 1;
    }
    return ESP_OK;
}

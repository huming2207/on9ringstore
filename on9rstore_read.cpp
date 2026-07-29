#include <cstring>

#include "on9rstore.hpp"

esp_err_t on9rstore::acquire_read_operation_locks(uint32_t timeout_ticks) const
{
    if (lifecycle_lock == nullptr || write_lock == nullptr || read_lock == nullptr) {
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
    memcpy(read_segments.get(), segments.get(), sizeof(segment_descriptor) * segment_count);
    memcpy(read_active_sparse_index.get(), sparse_index.get(), sizeof(on9rstore_def::sparse_index_entry) * sparse_index_count);
    if (write_buf_pos > 0) {
        memcpy(read_buf.get(), write_buf.get(), write_buf_pos);
    }

    read_active_segment = active_segment;
    read_sparse_index_count = 0;
    read_active_sparse_index_count = sparse_index_count;
    read_buf_pos = write_buf_pos;
    read_buf_offset = write_buf_offset;
}

void on9rstore::release_read_operation_lock() const
{
    xSemaphoreGive(read_lock);
}

bool on9rstore::find_read_segment(uint64_t first_entry_id, uint64_t last_entry_id, bool exact,
                                  segment_descriptor *descriptor_out) const
{
    if (descriptor_out == nullptr) {
        return false;
    }

    bool found = false;
    segment_descriptor selected = {};
    for (uint32_t slot = 0; slot < segment_count; slot += 1) {
        const segment_descriptor &candidate = read_segments[slot];
        if (!candidate.valid || candidate.entry_count == 0 || candidate.last_entry_id < first_entry_id ||
            candidate.first_entry_id > last_entry_id) {
            continue;
        }
        if (exact && (first_entry_id < candidate.first_entry_id || first_entry_id > candidate.last_entry_id)) {
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

esp_err_t on9rstore::load_read_sparse_index(const segment_descriptor &descriptor, const on9rstore_def::segment_header &header)
{
    if (!descriptor.sealed) {
        if (descriptor.slot != read_active_segment.slot || descriptor.generation != read_active_segment.generation ||
            descriptor.index_count != read_active_sparse_index_count) {
            return ESP_ERR_INVALID_STATE;
        }

        memcpy(read_sparse_index.get(), read_active_sparse_index.get(),
               sizeof(on9rstore_def::sparse_index_entry) * read_active_sparse_index_count);
        read_sparse_index_count = read_active_sparse_index_count;
        return ESP_OK;
    }

    on9rstore_def::segment_footer footer = {};
    esp_err_t ret = load_segment_footer(reader_fd, header, &footer);
    if (ret != ESP_OK) {
        return ret == ESP_ERR_NOT_FOUND ? ESP_ERR_INVALID_CRC : ret;
    }

    if (footer.first_entry_id != descriptor.first_entry_id || footer.last_entry_id != descriptor.last_entry_id ||
        footer.entry_count != descriptor.entry_count || footer.data_end != descriptor.data_end ||
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

    const size_t index_size = static_cast<size_t>(footer.index_count) * sizeof(on9rstore_def::sparse_index_entry);
    ret = read_exact_fd(reader_fd, segment_file_size, header.index_start, read_sparse_index.get(), index_size);
    if (ret == ESP_OK) {
        read_sparse_index_count = footer.index_count;
    }
    return ret;
}

bool on9rstore::is_read_sparse_index_valid(const segment_descriptor &descriptor,
                                           const on9rstore_def::segment_header &header) const
{
    if (descriptor.entry_count == 0) {
        return read_sparse_index_count == 0;
    }

    const uint64_t expected_count = (descriptor.entry_count - 1) / header.index_stride + 1;
    if (expected_count != descriptor.index_count || descriptor.index_count != read_sparse_index_count ||
        read_sparse_index_count == 0) {
        return false;
    }

    uint64_t previous_id = 0;
    uint64_t previous_uptime_us = 0;
    uint32_t previous_offset = 0;
    for (uint32_t idx = 0; idx < read_sparse_index_count; idx += 1) {
        const on9rstore_def::sparse_index_entry &entry = read_sparse_index[idx];
        const uint32_t entry_boot_counter = get_entry_boot_counter(entry.entry_id);
        const uint32_t previous_boot_counter = get_entry_boot_counter(previous_id);
        if (entry.reserved != 0 || entry.entry_id < descriptor.first_entry_id || entry.entry_id > descriptor.last_entry_id ||
            entry.offset < header.data_start || entry.offset >= descriptor.data_end ||
            entry.offset % on9rstore_def::entry_alignment != 0 ||
            (idx > 0 && (entry.entry_id <= previous_id || entry.offset <= previous_offset ||
                         (entry_boot_counter == previous_boot_counter && entry.uptime_us < previous_uptime_us)))) {
            return false;
        }

        previous_id = entry.entry_id;
        previous_uptime_us = entry.uptime_us;
        previous_offset = entry.offset;
    }

    return read_sparse_index[0].entry_id == descriptor.first_entry_id && read_sparse_index[0].offset == header.data_start;
}

esp_err_t on9rstore::prepare_read_segment(const segment_descriptor &descriptor, on9rstore_def::segment_header *header_out)
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

void on9rstore::find_read_start(uint64_t entry_id, const on9rstore_def::segment_header &header, uint64_t *offset_out,
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
    *offset_out = read_sparse_index_count == 0 ? header.data_start : read_sparse_index[selected].offset;
    *index_entry_out = read_sparse_index_count == 0 ? on9rstore_def::sparse_index_entry{} : read_sparse_index[selected];
}

void on9rstore::find_boot_read_start(uint32_t boot_counter, uint64_t uptime_us, const on9rstore_def::segment_header &header,
                                     uint64_t *offset_out, on9rstore_def::sparse_index_entry *index_entry_out) const
{
    uint32_t low = 0;
    uint32_t high = read_sparse_index_count;
    while (low < high) {
        const uint32_t middle = low + (high - low) / 2;
        const on9rstore_def::sparse_index_entry &entry = read_sparse_index[middle];
        const uint32_t entry_boot_counter = get_entry_boot_counter(entry.entry_id);
        if (entry_boot_counter < boot_counter || (entry_boot_counter == boot_counter && entry.uptime_us <= uptime_us)) {
            low = middle + 1;
        } else {
            high = middle;
        }
    }

    const uint32_t selected = low == 0 ? 0 : low - 1;
    *offset_out = read_sparse_index_count == 0 ? header.data_start : read_sparse_index[selected].offset;
    *index_entry_out = read_sparse_index_count == 0 ? on9rstore_def::sparse_index_entry{} : read_sparse_index[selected];
}

esp_err_t on9rstore::read_snapshot_bytes(const on9rstore_def::segment_header &segment, uint64_t offset, void *buf_out,
                                         size_t len) const
{
    if (len == 0) {
        return ESP_OK;
    }
    if (buf_out == nullptr || len > segment_file_size || offset > segment_file_size - len) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool active_snapshot = segment.slot == read_active_segment.slot && segment.generation == read_active_segment.generation;
    if (active_snapshot && read_buf_pos > 0) {
        const uint64_t buffer_end = read_buf_offset + read_buf_pos;
        const uint64_t read_end = offset + len;
        if (offset >= read_buf_offset) {
            if (read_end > buffer_end) {
                return ESP_ERR_INVALID_SIZE;
            }

            memcpy(buf_out, read_buf.get() + static_cast<size_t>(offset - read_buf_offset), len);
            return ESP_OK;
        }
        if (read_end > read_buf_offset) {
            return ESP_ERR_INVALID_SIZE;
        }
    }

    return read_exact_fd(reader_fd, segment_file_size, offset, buf_out, len);
}

esp_err_t on9rstore::read_snapshot_entry_header(const on9rstore_def::segment_header &segment, uint64_t entry_limit,
                                                uint64_t offset, on9rstore_def::entry_header *header_out,
                                                uint64_t *entry_size_out) const
{
    if (header_out == nullptr || entry_size_out == nullptr || entry_limit < segment.data_start ||
        entry_limit > segment.data_end || offset < segment.data_start || offset > entry_limit ||
        entry_limit - offset < on9rstore_def::min_entry_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    on9rstore_def::entry_header header = {};
    esp_err_t ret = read_snapshot_bytes(segment, offset, &header, sizeof(header));
    if (ret != ESP_OK) {
        return ret;
    }

    if (header.magic != on9rstore_def::entry_magic || header.revision != on9rstore_def::entry_revision ||
        header.store_id != store_id || header.segment_slot != segment.slot || header.segment_generation != segment.generation ||
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

esp_err_t on9rstore::validate_snapshot_entry_payload(const on9rstore_def::segment_header &segment, uint64_t offset,
                                                     const on9rstore_def::entry_header &header, uint8_t *payload_out,
                                                     size_t payload_out_len, bool copy_payload) const
{
    const bool buffer_too_small = copy_payload && (header.len > payload_out_len || (header.len > 0 && payload_out == nullptr));
    uint32_t crc = calc_crc32_update(UINT32_MAX, reinterpret_cast<const uint8_t *>(&header), sizeof(header));

    uint8_t chunk[256] = {};
    uint64_t payload_offset = offset + sizeof(header);
    uint32_t copied = 0;
    uint32_t remaining = header.len;
    while (remaining > 0) {
        const size_t chunk_len = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
        uint8_t *destination = copy_payload && !buffer_too_small ? payload_out + copied : chunk;
        esp_err_t ret = read_snapshot_bytes(segment, payload_offset, destination, chunk_len);
        if (ret != ESP_OK) {
            return ret;
        }

        crc = calc_crc32_update(crc, destination, chunk_len);
        payload_offset += chunk_len;
        copied += static_cast<uint32_t>(chunk_len);
        remaining -= static_cast<uint32_t>(chunk_len);
    }

    uint32_t expected_crc = 0;
    esp_err_t ret = read_snapshot_bytes(segment, payload_offset, &expected_crc, sizeof(expected_crc));
    if (ret != ESP_OK) {
        return ret;
    }
    if (~crc != expected_crc) {
        return ESP_ERR_INVALID_CRC;
    }

    return buffer_too_small ? ESP_ERR_INVALID_SIZE : ESP_OK;
}

esp_err_t on9rstore::read_matching_entry(const segment_descriptor &descriptor, const on9rstore_def::segment_header &header,
                                         uint64_t first_entry_id, uint64_t last_entry_id, bool exact, uint8_t *payload_out,
                                         size_t payload_out_len, on9rstore_def::entry_header *entry_info_out)
{
    uint64_t offset = 0;
    on9rstore_def::sparse_index_entry start_index = {};
    find_read_start(first_entry_id, header, &offset, &start_index);

    uint64_t previous_id = 0;
    bool first_scanned = true;
    while (offset < descriptor.data_end) {
        on9rstore_def::entry_header entry = {};
        uint64_t entry_size = 0;
        esp_err_t ret = read_snapshot_entry_header(header, descriptor.data_end, offset, &entry, &entry_size);
        if (ret != ESP_OK) {
            return ret;
        }

        const bool matches =
            exact ? entry.entry_id == first_entry_id : entry.entry_id >= first_entry_id && entry.entry_id <= last_entry_id;
        ret = validate_snapshot_entry_payload(header, offset, entry, payload_out, payload_out_len, matches);
        if (ret != ESP_OK && !(matches && ret == ESP_ERR_INVALID_SIZE)) {
            return ret;
        }

        if ((first_scanned && start_index.entry_id != 0 &&
             (entry.entry_id != start_index.entry_id || entry.uptime_us != start_index.uptime_us ||
              entry.type != start_index.type || offset != start_index.offset)) ||
            (previous_id != 0 && entry.entry_id <= previous_id)) {
            return ESP_ERR_INVALID_CRC;
        }

        if (matches) {
            if (entry_info_out != nullptr) {
                *entry_info_out = entry;
            }
            return ret;
        }

        if (entry.entry_id > last_entry_id || (exact && entry.entry_id > first_entry_id)) {
            return ESP_ERR_NOT_FOUND;
        }

        first_scanned = false;
        previous_id = entry.entry_id;
        offset += entry_size;
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t on9rstore::read_matching_boot_entry(const segment_descriptor &descriptor, const on9rstore_def::segment_header &header,
                                              const on9rstore_def::boot_uptime_range_cursor &cursor, uint64_t first_entry_id,
                                              bool use_uptime_start, uint8_t *payload_out, size_t payload_out_len,
                                              on9rstore_def::entry_header *entry_info_out)
{
    uint64_t offset = 0;
    on9rstore_def::sparse_index_entry start_index = {};
    if (use_uptime_start) {
        find_boot_read_start(cursor.boot_counter, cursor.first_uptime_us, header, &offset, &start_index);
    } else {
        find_read_start(first_entry_id, header, &offset, &start_index);
    }

    uint64_t previous_id = 0;
    uint64_t previous_uptime_us = 0;
    bool first_scanned = true;
    while (offset < descriptor.data_end) {
        on9rstore_def::entry_header entry = {};
        uint64_t entry_size = 0;
        esp_err_t ret = read_snapshot_entry_header(header, descriptor.data_end, offset, &entry, &entry_size);
        if (ret != ESP_OK) {
            return ret;
        }

        const uint32_t entry_boot_counter = get_entry_boot_counter(entry.entry_id);
        const bool matches = entry.entry_id >= first_entry_id && entry_boot_counter == cursor.boot_counter &&
                             entry.uptime_us >= cursor.first_uptime_us && entry.uptime_us <= cursor.last_uptime_us;
        ret = validate_snapshot_entry_payload(header, offset, entry, payload_out, payload_out_len, matches);
        if (ret != ESP_OK && !(matches && ret == ESP_ERR_INVALID_SIZE)) {
            return ret;
        }

        if ((first_scanned && start_index.entry_id != 0 &&
             (entry.entry_id != start_index.entry_id || entry.uptime_us != start_index.uptime_us ||
              entry.type != start_index.type || offset != start_index.offset)) ||
            (previous_id != 0 && (entry.entry_id <= previous_id || (entry_boot_counter == get_entry_boot_counter(previous_id) &&
                                                                    entry.uptime_us < previous_uptime_us)))) {
            return ESP_ERR_INVALID_CRC;
        }

        if (matches) {
            if (entry_info_out != nullptr) {
                *entry_info_out = entry;
            }
            return ret;
        }
        if (entry_boot_counter > cursor.boot_counter ||
            (entry_boot_counter == cursor.boot_counter && entry.uptime_us > cursor.last_uptime_us)) {
            return ESP_ERR_NOT_FOUND;
        }

        first_scanned = false;
        previous_id = entry.entry_id;
        previous_uptime_us = entry.uptime_us;
        offset += entry_size;
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t on9rstore::read_entry_internal(uint64_t first_entry_id, uint64_t last_entry_id, bool exact, uint8_t *payload_out,
                                         size_t payload_out_len, on9rstore_def::entry_header *entry_info_out,
                                         uint32_t timeout_ticks)
{
    esp_err_t ret = acquire_read_operation_locks(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    snapshot_read_state_unsafe();
    xSemaphoreGive(write_lock);

    segment_descriptor descriptor = {};
    if (!find_read_segment(first_entry_id, last_entry_id, exact, &descriptor)) {
        release_read_operation_lock();
        return ESP_ERR_NOT_FOUND;
    }

    on9rstore_def::segment_header header = {};
    ret = prepare_read_segment(descriptor, &header);
    if (ret == ESP_OK) {
        ret = read_matching_entry(descriptor, header, first_entry_id, last_entry_id, exact, payload_out, payload_out_len,
                                  entry_info_out);
    }

    close_reader_segment();
    release_read_operation_lock();
    return ret;
}

uint32_t on9rstore::get_entry_boot_counter(uint64_t entry_id)
{
    return static_cast<uint32_t>(entry_id >> 40ULL);
}

uint64_t on9rstore::get_boot_first_entry_id(uint32_t boot_counter)
{
    return (static_cast<uint64_t>(boot_counter) << 40ULL) | 1ULL;
}

uint64_t on9rstore::get_boot_last_entry_id(uint32_t boot_counter)
{
    return (static_cast<uint64_t>(boot_counter) << 40ULL) | on9rstore_def::entry_id_sequence_mask;
}

bool on9rstore::is_boot_range_cursor_valid(const on9rstore_def::boot_uptime_range_cursor &cursor)
{
    if (cursor.boot_counter == 0 || cursor.boot_counter > on9rstore_def::entry_id_boot_mask ||
        cursor.first_uptime_us > cursor.last_uptime_us) {
        return false;
    }
    if (cursor.next_entry_id == 0) {
        return true;
    }

    return get_entry_boot_counter(cursor.next_entry_id) == cursor.boot_counter &&
           cursor.next_entry_id >= get_boot_first_entry_id(cursor.boot_counter) &&
           cursor.next_entry_id <= get_boot_last_entry_id(cursor.boot_counter);
}

esp_err_t on9rstore::read_boot_entry_internal(const on9rstore_def::boot_uptime_range_cursor &cursor, uint8_t *payload_out,
                                              size_t payload_out_len, on9rstore_def::entry_header *entry_info_out,
                                              uint32_t timeout_ticks)
{
    esp_err_t ret = acquire_read_operation_locks(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    snapshot_read_state_unsafe();
    xSemaphoreGive(write_lock);

    ret = read_boot_entry_from_snapshot(cursor, payload_out, payload_out_len, entry_info_out);
    release_read_operation_lock();
    return ret;
}

esp_err_t on9rstore::read_boot_entry_from_snapshot(const on9rstore_def::boot_uptime_range_cursor &cursor, uint8_t *payload_out,
                                                   size_t payload_out_len, on9rstore_def::entry_header *entry_info_out)
{
    esp_err_t ret = ESP_ERR_NOT_FOUND;
    const uint64_t boot_last_entry_id = get_boot_last_entry_id(cursor.boot_counter);
    uint64_t first_entry_id = cursor.next_entry_id == 0 ? get_boot_first_entry_id(cursor.boot_counter) : cursor.next_entry_id;
    bool use_uptime_start = cursor.next_entry_id == 0;
    segment_descriptor descriptor = {};
    while (find_read_segment(first_entry_id, boot_last_entry_id, false, &descriptor)) {
        on9rstore_def::segment_header header = {};
        ret = prepare_read_segment(descriptor, &header);
        if (ret == ESP_OK) {
            ret = read_matching_boot_entry(descriptor, header, cursor, first_entry_id, use_uptime_start, payload_out,
                                           payload_out_len, entry_info_out);
        }
        close_reader_segment();
        if (ret != ESP_ERR_NOT_FOUND || descriptor.last_entry_id >= boot_last_entry_id) {
            break;
        }

        first_entry_id = descriptor.last_entry_id + 1;
        use_uptime_start = cursor.next_entry_id == 0;
    }

    return ret;
}

bool on9rstore::make_utc_boot_cursor(const time_model_epoch &epoch, const on9rstore_def::utc_range_cursor &utc_cursor,
                                     on9rstore_def::boot_uptime_range_cursor *boot_cursor_out) const
{
    if (boot_cursor_out == nullptr) {
        return false;
    }

    uint64_t first_uptime = 0;
    uint64_t last_uptime = 0;
    const timestamp_translation_result first_result =
        translate_timestamp(utc_cursor.first_utc_us, epoch.anchor.utc_us, epoch.anchor.monotonic_us, &first_uptime);
    const timestamp_translation_result last_result =
        translate_timestamp(utc_cursor.last_utc_us, epoch.anchor.utc_us, epoch.anchor.monotonic_us, &last_uptime);
    if (first_result == timestamp_translation_result::above_max || last_result == timestamp_translation_result::below_zero) {
        return false;
    }

    if (first_result == timestamp_translation_result::below_zero || first_uptime < epoch.first_uptime_us) {
        first_uptime = epoch.first_uptime_us;
    }
    if (last_result == timestamp_translation_result::above_max || last_uptime > epoch.last_uptime_us) {
        last_uptime = epoch.last_uptime_us;
    }
    if (first_uptime > last_uptime) {
        return false;
    }

    on9rstore_def::boot_uptime_range_cursor boot_cursor = {};
    boot_cursor.boot_counter = epoch.anchor.boot_counter;
    boot_cursor.first_uptime_us = first_uptime;
    boot_cursor.last_uptime_us = last_uptime;
    if (utc_cursor.next_entry_id != 0) {
        const uint32_t next_boot_counter = get_entry_boot_counter(utc_cursor.next_entry_id);
        if (next_boot_counter > boot_cursor.boot_counter) {
            return false;
        }
        if (next_boot_counter == boot_cursor.boot_counter) {
            boot_cursor.next_entry_id = utc_cursor.next_entry_id;
        }
    }

    *boot_cursor_out = boot_cursor;
    return true;
}

on9rstore::timestamp_translation_result on9rstore::translate_timestamp(uint64_t value, uint64_t from_origin, uint64_t to_origin,
                                                                       uint64_t *translated_out)
{
    if (value >= from_origin) {
        const uint64_t delta = value - from_origin;
        if (delta > UINT64_MAX - to_origin) {
            return timestamp_translation_result::above_max;
        }
        *translated_out = to_origin + delta;
        return timestamp_translation_result::in_range;
    }

    const uint64_t delta = from_origin - value;
    if (delta > to_origin) {
        return timestamp_translation_result::below_zero;
    }
    *translated_out = to_origin - delta;
    return timestamp_translation_result::in_range;
}

bool on9rstore::calculate_entry_utc(const time_model_epoch &epoch, uint64_t uptime_us, uint64_t *utc_us_out)
{
    if (utc_us_out == nullptr) {
        return false;
    }

    return translate_timestamp(uptime_us, epoch.anchor.monotonic_us, epoch.anchor.utc_us, utc_us_out) ==
           timestamp_translation_result::in_range;
}

void on9rstore::set_entry_utc_info(const time_model_epoch &epoch, uint64_t utc_us, on9rstore_def::entry_utc_info *utc_info_out)
{
    if (utc_info_out == nullptr) {
        return;
    }

    utc_info_out->utc_us = utc_us;
    utc_info_out->anchor_sequence = epoch.anchor.sequence;
    utc_info_out->anchor_uncertainty_us = epoch.anchor.uncertainty_us;
    utc_info_out->source_mask = epoch.anchor.source_mask;
    utc_info_out->source_count = epoch.anchor.source_count;
    utc_info_out->quality = epoch.anchor.quality;
    utc_info_out->flags = epoch.anchor.flags;
}

esp_err_t on9rstore::read_utc_entry_internal(const on9rstore_def::utc_range_cursor &cursor, uint8_t *payload_out,
                                             size_t payload_out_len, on9rstore_def::entry_header *entry_info_out,
                                             on9rstore_def::entry_utc_info *utc_info_out, uint32_t timeout_ticks)
{
    esp_err_t ret = acquire_read_operation_locks(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    snapshot_read_state_unsafe();
    xSemaphoreGive(write_lock);
    ret = ESP_ERR_NOT_FOUND;
    for (uint32_t index = 0; index < time_model_epoch_count; index += 1) {
        const time_model_epoch &epoch = time_model_epochs[index];
        on9rstore_def::boot_uptime_range_cursor boot_cursor = {};
        if (!make_utc_boot_cursor(epoch, cursor, &boot_cursor)) {
            continue;
        }

        ret = read_boot_entry_from_snapshot(boot_cursor, payload_out, payload_out_len, entry_info_out);
        if (ret == ESP_ERR_NOT_FOUND) {
            continue;
        }
        if ((ret == ESP_OK || ret == ESP_ERR_INVALID_SIZE) && entry_info_out != nullptr) {
            uint64_t utc_us = 0;
            if (!calculate_entry_utc(epoch, entry_info_out->uptime_us, &utc_us)) {
                ret = ESP_ERR_INVALID_STATE;
            } else {
                set_entry_utc_info(epoch, utc_us, utc_info_out);
            }
        }
        break;
    }

    close_reader_segment();
    release_read_operation_lock();
    return ret;
}

esp_err_t on9rstore::read_entry(uint64_t entry_id, uint8_t *payload_out, size_t payload_out_len,
                                on9rstore_def::entry_header *entry_info_out, uint32_t timeout_ticks)
{
    if (!is_entry_id_valid(entry_id) || (payload_out == nullptr && payload_out_len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    return read_entry_internal(entry_id, entry_id, true, payload_out, payload_out_len, entry_info_out, timeout_ticks);
}

esp_err_t on9rstore::read_next_entry(on9rstore_def::entry_range_cursor *cursor, uint8_t *payload_out, size_t payload_out_len,
                                     on9rstore_def::entry_header *entry_info_out, uint32_t timeout_ticks)
{
    if (cursor == nullptr || (payload_out == nullptr && payload_out_len > 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cursor->finished || cursor->next_entry_id > cursor->last_entry_id) {
        cursor->finished = true;
        return ESP_ERR_NOT_FOUND;
    }

    on9rstore_def::entry_header entry = {};
    esp_err_t ret = read_entry_internal(cursor->next_entry_id, cursor->last_entry_id, false, payload_out, payload_out_len, &entry,
                                        timeout_ticks);
    if (ret == ESP_ERR_NOT_FOUND) {
        cursor->finished = true;
        return ret;
    }
    if (ret != ESP_OK) {
        if (entry_info_out != nullptr && ret == ESP_ERR_INVALID_SIZE && entry.entry_id != 0) {
            *entry_info_out = entry;
        }
        return ret;
    }

    if (entry_info_out != nullptr) {
        *entry_info_out = entry;
    }
    if (entry.entry_id >= cursor->last_entry_id || entry.entry_id == UINT64_MAX) {
        cursor->finished = true;
    } else {
        cursor->next_entry_id = entry.entry_id + 1;
    }
    return ESP_OK;
}

esp_err_t on9rstore::read_next_entry_by_uptime(on9rstore_def::boot_uptime_range_cursor *cursor, uint8_t *payload_out,
                                               size_t payload_out_len, on9rstore_def::entry_header *entry_info_out,
                                               uint32_t timeout_ticks)
{
    if (cursor == nullptr || (payload_out == nullptr && payload_out_len > 0) || !is_boot_range_cursor_valid(*cursor)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cursor->finished) {
        return ESP_ERR_NOT_FOUND;
    }

    on9rstore_def::entry_header entry = {};
    esp_err_t ret = read_boot_entry_internal(*cursor, payload_out, payload_out_len, &entry, timeout_ticks);
    if (ret == ESP_ERR_NOT_FOUND) {
        cursor->finished = true;
        return ret;
    }
    if (ret != ESP_OK) {
        if (entry_info_out != nullptr && ret == ESP_ERR_INVALID_SIZE && entry.entry_id != 0) {
            *entry_info_out = entry;
        }
        return ret;
    }

    if (entry_info_out != nullptr) {
        *entry_info_out = entry;
    }
    if (entry.entry_id >= get_boot_last_entry_id(cursor->boot_counter)) {
        cursor->finished = true;
    } else {
        cursor->next_entry_id = entry.entry_id + 1;
    }
    return ESP_OK;
}

esp_err_t on9rstore::read_next_entry_by_utc(on9rstore_def::utc_range_cursor *cursor, uint8_t *payload_out, size_t payload_out_len,
                                            on9rstore_def::entry_header *entry_info_out,
                                            on9rstore_def::entry_utc_info *utc_info_out, uint32_t timeout_ticks)
{
    if (cursor == nullptr || (payload_out == nullptr && payload_out_len > 0) || cursor->first_utc_us > cursor->last_utc_us) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cursor->finished) {
        return ESP_ERR_NOT_FOUND;
    }

    on9rstore_def::entry_header entry = {};
    on9rstore_def::entry_utc_info utc_info = {};
    esp_err_t ret = read_utc_entry_internal(*cursor, payload_out, payload_out_len, &entry, &utc_info, timeout_ticks);
    if (ret == ESP_ERR_NOT_FOUND) {
        cursor->finished = true;
        return ret;
    }
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_INVALID_SIZE && entry.entry_id != 0) {
            if (entry_info_out != nullptr) {
                *entry_info_out = entry;
            }
            if (utc_info_out != nullptr) {
                *utc_info_out = utc_info;
            }
        }
        return ret;
    }

    if (entry_info_out != nullptr) {
        *entry_info_out = entry;
    }
    if (utc_info_out != nullptr) {
        *utc_info_out = utc_info;
    }
    if (entry.entry_id == UINT64_MAX) {
        cursor->finished = true;
    } else {
        cursor->next_entry_id = entry.entry_id + 1;
    }
    return ESP_OK;
}

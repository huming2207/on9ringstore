#include <cstring>
#include <inttypes.h>
#include <unistd.h>

#include <esp_log.h>

#include "on9rstore.hpp"

bool on9rstore::calculate_segment_geometry(
    uint64_t segment_size,
    on9rstore_def::segment_header *geometry_out) const
{
    if (geometry_out == nullptr ||
        segment_size % on9rstore_def::segment_header_slot_size != 0 ||
        segment_size <= on9rstore_def::segment_header_region_size +
                            on9rstore_def::segment_footer_region_size) {
        return false;
    }

    const uint64_t available =
        segment_size - on9rstore_def::segment_header_region_size -
        on9rstore_def::segment_footer_region_size;
    const uint64_t max_entries =
        available / on9rstore_def::min_entry_size;
    const uint64_t index_capacity =
        max_entries / on9rstore_def::sparse_index_stride + 1;
    const uint64_t raw_index_size =
        index_capacity * sizeof(on9rstore_def::sparse_index_entry);
    const uint64_t index_size = on9rstore_def::align_up(
        static_cast<size_t>(raw_index_size),
        on9rstore_def::segment_header_slot_size);

    if (index_capacity > UINT32_MAX || index_size >= available) {
        return false;
    }

    geometry_out->segment_size = segment_size;
    geometry_out->data_start =
        on9rstore_def::segment_header_region_size;
    geometry_out->index_start =
        segment_size - on9rstore_def::segment_footer_region_size -
        index_size;
    geometry_out->data_end = geometry_out->index_start;
    geometry_out->index_capacity =
        static_cast<uint32_t>(index_capacity);
    geometry_out->index_stride =
        on9rstore_def::sparse_index_stride;
    return geometry_out->data_end >
           geometry_out->data_start + on9rstore_def::min_entry_size;
}

esp_err_t on9rstore::write_segment_headers(
    int segment_fd, const on9rstore_def::segment_header &header)
{
    on9rstore_def::segment_header copy = header;
    copy.checksum = 0;
    copy.checksum =
        calc_crc32(reinterpret_cast<const uint8_t *>(&copy), sizeof(copy));

    for (uint32_t slot = 0;
         slot < on9rstore_def::segment_header_slot_count; slot += 1) {
        const uint64_t offset =
            static_cast<uint64_t>(slot) *
            on9rstore_def::segment_header_slot_size;
        esp_err_t ret = write_exact_fd(segment_fd, segment_file_size,
                                       offset, &copy, sizeof(copy));
        if (ret != ESP_OK) {
            return ret;
        }

        ret = sync_fd(segment_fd);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t on9rstore::write_segment_footers(
    int segment_fd, const on9rstore_def::segment_footer &footer)
{
    on9rstore_def::segment_footer copy = footer;
    copy.checksum = 0;
    copy.checksum =
        calc_crc32(reinterpret_cast<const uint8_t *>(&copy), sizeof(copy));

    const uint64_t footer_start =
        segment_file_size - on9rstore_def::segment_footer_region_size;
    for (uint32_t slot = 0;
         slot < on9rstore_def::segment_footer_slot_count; slot += 1) {
        const uint64_t offset =
            footer_start + static_cast<uint64_t>(slot) *
                               on9rstore_def::segment_footer_slot_size;
        esp_err_t ret = write_exact_fd(segment_fd, segment_file_size,
                                       offset, &copy, sizeof(copy));
        if (ret != ESP_OK) {
            return ret;
        }

        ret = sync_fd(segment_fd);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t on9rstore::initialise_empty_segment(int segment_fd, uint32_t slot)
{
    on9rstore_def::segment_header header = {};
    if (!calculate_segment_geometry(segment_file_size, &header)) {
        return ESP_ERR_INVALID_SIZE;
    }

    header.magic = on9rstore_def::segment_header_magic;
    header.revision = on9rstore_def::segment_header_revision;
    header.size = sizeof(header);
    header.store_id = store_id;
    header.state = on9rstore_def::segment_state_empty;
    header.slot = slot;
    return write_segment_headers(segment_fd, header);
}

esp_err_t on9rstore::provision_one_segment(uint32_t slot)
{
    char path[PATH_MAX] = {};
    esp_err_t ret = build_data_path(slot, path, sizeof(path));
    if (ret != ESP_OK) {
        return ret;
    }

    if (state.state == on9rstore_def::manifest_state_ready) {
        return validate_contiguous_file(path, segment_file_size);
    }

    bool created = false;
    ret = provision_contiguous_file(path, segment_file_size, &created);
    if (ret != ESP_OK) {
        return ret;
    }

    int segment_fd = -1;
    ret = open_file(path, &segment_fd);
    if (ret == ESP_OK) {
        ret = initialise_empty_segment(segment_fd, slot);
    }
    if (segment_fd >= 0) {
        (void)close(segment_fd);
    }

    return ret;
}

esp_err_t on9rstore::provision_all_segments()
{
    for (uint32_t slot = 0; slot < segment_count; slot += 1) {
        esp_err_t ret = provision_one_segment(slot);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    if (state.state == on9rstore_def::manifest_state_ready) {
        return ESP_OK;
    }

    esp_err_t ret = activate_segment_unsafe(0, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    state.state = on9rstore_def::manifest_state_ready;
    state.active_slot = 0;
    state.active_segment_generation = 1;
    state.next_segment_generation = 2;
    state.oldest_segment_generation = 1;
    return commit_manifest_superblock_unsafe();
}

bool on9rstore::is_segment_header_valid(
    const on9rstore_def::segment_header &candidate, uint32_t slot) const
{
    if (candidate.magic != on9rstore_def::segment_header_magic ||
        candidate.revision != on9rstore_def::segment_header_revision ||
        candidate.size != sizeof(candidate) ||
        candidate.store_id != store_id || candidate.slot != slot ||
        candidate.segment_size != segment_file_size ||
        candidate.index_stride != on9rstore_def::sparse_index_stride) {
        return false;
    }

    if ((candidate.generation == 0 &&
         candidate.state != on9rstore_def::segment_state_empty) ||
        (candidate.generation != 0 &&
         candidate.state != on9rstore_def::segment_state_active)) {
        return false;
    }

    on9rstore_def::segment_header geometry = {};
    if (!calculate_segment_geometry(segment_file_size, &geometry) ||
        candidate.data_start != geometry.data_start ||
        candidate.data_end != geometry.data_end ||
        candidate.index_start != geometry.index_start ||
        candidate.index_capacity != geometry.index_capacity) {
        return false;
    }

    on9rstore_def::segment_header copy = candidate;
    const uint32_t expected_crc = copy.checksum;
    copy.checksum = 0;
    return calc_crc32(reinterpret_cast<const uint8_t *>(&copy),
                      sizeof(copy)) == expected_crc;
}

esp_err_t on9rstore::load_segment_header(
    int segment_fd, uint32_t slot,
    on9rstore_def::segment_header *header_out) const
{
    if (header_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    bool found = false;
    on9rstore_def::segment_header selected = {};
    for (uint32_t copy_slot = 0;
         copy_slot < on9rstore_def::segment_header_slot_count;
         copy_slot += 1) {
        on9rstore_def::segment_header candidate = {};
        const uint64_t offset =
            static_cast<uint64_t>(copy_slot) *
            on9rstore_def::segment_header_slot_size;
        esp_err_t ret = read_exact_fd(segment_fd, segment_file_size,
                                      offset, &candidate,
                                      sizeof(candidate));
        if (ret != ESP_OK) {
            return ret;
        }

        if (is_segment_header_valid(candidate, slot) &&
            (!found || candidate.generation > selected.generation)) {
            selected = candidate;
            found = true;
        }
    }

    if (!found) {
        return ESP_ERR_INVALID_STATE;
    }

    *header_out = selected;
    return ESP_OK;
}

bool on9rstore::is_segment_footer_valid(
    const on9rstore_def::segment_footer &candidate,
    const on9rstore_def::segment_header &header) const
{
    if (candidate.magic != on9rstore_def::segment_footer_magic ||
        candidate.revision != on9rstore_def::segment_footer_revision ||
        candidate.size != sizeof(candidate) ||
        candidate.store_id != store_id ||
        candidate.state != on9rstore_def::segment_footer_state_sealed ||
        candidate.slot != header.slot ||
        candidate.generation != header.generation ||
        candidate.data_end < header.data_start ||
        candidate.data_end > header.data_end ||
        candidate.index_count > header.index_capacity ||
        candidate.index_stride != header.index_stride) {
        return false;
    }

    if ((candidate.entry_count == 0 &&
         (candidate.first_entry_id != 0 ||
          candidate.last_entry_id != 0)) ||
        (candidate.entry_count != 0 &&
         (candidate.first_entry_id == 0 ||
          candidate.last_entry_id < candidate.first_entry_id))) {
        return false;
    }

    on9rstore_def::segment_footer copy = candidate;
    const uint32_t expected_crc = copy.checksum;
    copy.checksum = 0;
    return calc_crc32(reinterpret_cast<const uint8_t *>(&copy),
                      sizeof(copy)) == expected_crc;
}

esp_err_t on9rstore::load_segment_footer(
    int segment_fd, const on9rstore_def::segment_header &header,
    on9rstore_def::segment_footer *footer_out) const
{
    if (footer_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint64_t footer_start =
        segment_file_size - on9rstore_def::segment_footer_region_size;
    bool found = false;
    on9rstore_def::segment_footer selected = {};

    for (uint32_t slot = 0;
         slot < on9rstore_def::segment_footer_slot_count; slot += 1) {
        on9rstore_def::segment_footer candidate = {};
        const uint64_t offset =
            footer_start + static_cast<uint64_t>(slot) *
                               on9rstore_def::segment_footer_slot_size;
        esp_err_t ret = read_exact_fd(segment_fd, segment_file_size,
                                      offset, &candidate,
                                      sizeof(candidate));
        if (ret != ESP_OK) {
            return ret;
        }

        if (is_segment_footer_valid(candidate, header)) {
            selected = candidate;
            found = true;
        }
    }

    if (!found) {
        return ESP_ERR_NOT_FOUND;
    }

    *footer_out = selected;
    return ESP_OK;
}

esp_err_t on9rstore::scan_one_entry(
    int segment_fd, const on9rstore_def::segment_header &segment,
    uint64_t offset, on9rstore_def::entry_header *header_out,
    uint64_t *entry_size_out) const
{
    if (header_out == nullptr || entry_size_out == nullptr ||
        offset > segment.data_end - on9rstore_def::min_entry_size) {
        return ESP_ERR_NOT_FOUND;
    }

    on9rstore_def::entry_header header = {};
    esp_err_t ret = read_exact_fd(segment_fd, segment_file_size, offset,
                                  &header, sizeof(header));
    if (ret != ESP_OK) {
        return ret;
    }

    if (header.magic != on9rstore_def::entry_magic ||
        header.revision != on9rstore_def::entry_revision ||
        header.store_id != store_id ||
        header.segment_slot != segment.slot ||
        header.segment_generation != segment.generation ||
        !is_entry_id_valid(header.entry_id)) {
        return ESP_ERR_NOT_FOUND;
    }

    const uint64_t entry_size = get_entry_size(header.len);
    if (entry_size == 0 || entry_size > segment.data_end - offset) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t crc = calc_crc32_update(
        UINT32_MAX, reinterpret_cast<const uint8_t *>(&header),
        sizeof(header));
    uint8_t chunk[256] = {};
    uint64_t payload_offset = offset + sizeof(header);
    uint32_t remaining = header.len;
    while (remaining > 0) {
        const size_t chunk_len =
            remaining < sizeof(chunk) ? remaining : sizeof(chunk);
        ret = read_exact_fd(segment_fd, segment_file_size, payload_offset,
                            chunk, chunk_len);
        if (ret != ESP_OK) {
            return ret;
        }

        crc = calc_crc32_update(crc, chunk, chunk_len);
        payload_offset += chunk_len;
        remaining -= static_cast<uint32_t>(chunk_len);
    }

    uint32_t expected_crc = 0;
    ret = read_exact_fd(segment_fd, segment_file_size, payload_offset,
                        &expected_crc, sizeof(expected_crc));
    if (ret != ESP_OK) {
        return ret;
    }
    if (~crc != expected_crc) {
        return ESP_ERR_INVALID_CRC;
    }

    *header_out = header;
    *entry_size_out = entry_size;
    return ESP_OK;
}

esp_err_t on9rstore::scan_segment_entries(
    int segment_fd, const on9rstore_def::segment_header &header,
    segment_descriptor *descriptor_out, bool build_index)
{
    if (descriptor_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    segment_descriptor descriptor = {};
    descriptor.valid = true;
    descriptor.slot = header.slot;
    descriptor.generation = header.generation;
    descriptor.data_end = header.data_start;
    if (build_index) {
        sparse_index_count = 0;
    }

    uint64_t offset = header.data_start;
    uint64_t previous_id = 0;
    while (offset <= header.data_end - on9rstore_def::min_entry_size) {
        on9rstore_def::entry_header entry = {};
        uint64_t entry_size = 0;
        esp_err_t ret = scan_one_entry(segment_fd, header, offset,
                                       &entry, &entry_size);
        if (ret == ESP_ERR_NOT_FOUND || ret == ESP_ERR_INVALID_SIZE ||
            ret == ESP_ERR_INVALID_CRC) {
            break;
        }
        if (ret != ESP_OK) {
            return ret;
        }
        if (previous_id != 0 && entry.entry_id <= previous_id) {
            break;
        }

        if (descriptor.entry_count == 0) {
            descriptor.first_entry_id = entry.entry_id;
        }
        if (build_index &&
            descriptor.entry_count % header.index_stride == 0) {
            add_sparse_index_entry_unsafe(entry, offset);
        }

        descriptor.last_entry_id = entry.entry_id;
        descriptor.entry_count += 1;
        descriptor.data_end = offset + entry_size;
        descriptor.index_count = build_index ? sparse_index_count : 0;
        previous_id = entry.entry_id;
        offset += entry_size;
    }

    *descriptor_out = descriptor;
    return ESP_OK;
}

esp_err_t on9rstore::recover_one_segment(uint32_t slot)
{
    esp_err_t ret = open_reader_segment(slot);
    if (ret != ESP_OK) {
        return ret;
    }

    on9rstore_def::segment_header header = {};
    ret = load_segment_header(reader_fd, slot, &header);
    if (ret != ESP_OK) {
        close_reader_segment();
        return ret;
    }

    if (header.generation == 0) {
        segments[slot] = {};
        close_reader_segment();
        return ESP_OK;
    }

    on9rstore_def::segment_footer footer = {};
    ret = load_segment_footer(reader_fd, header, &footer);
    if (ret == ESP_OK) {
        segment_descriptor descriptor = {};
        descriptor.valid = true;
        descriptor.sealed = true;
        descriptor.slot = slot;
        descriptor.generation = header.generation;
        descriptor.first_entry_id = footer.first_entry_id;
        descriptor.last_entry_id = footer.last_entry_id;
        descriptor.entry_count = footer.entry_count;
        descriptor.data_end = footer.data_end;
        descriptor.index_count = footer.index_count;
        segments[slot] = descriptor;
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ret = scan_segment_entries(reader_fd, header, &segments[slot],
                                   false);
    }

    close_reader_segment();
    if (ret == ESP_OK &&
        state.oldest_segment_generation != 0 &&
        segments[slot].valid &&
        segments[slot].generation <
            state.oldest_segment_generation) {
        segments[slot] = {};
    }
    return ret;
}

esp_err_t on9rstore::open_active_segment(uint32_t slot)
{
    close_writer_segment();
    esp_err_t ret =
        build_data_path(slot, active_data_path, sizeof(active_data_path));
    if (ret != ESP_OK) {
        return ret;
    }

    return open_file(active_data_path, &writer_fd);
}

esp_err_t on9rstore::recover_open_segment(
    uint32_t slot, const on9rstore_def::segment_header &header)
{
    esp_err_t ret = open_reader_segment(slot);
    if (ret != ESP_OK) {
        return ret;
    }

    segment_descriptor descriptor = {};
    ret = scan_segment_entries(reader_fd, header, &descriptor, true);
    close_reader_segment();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = open_active_segment(slot);
    if (ret != ESP_OK) {
        return ret;
    }

    active_segment = header;
    segments[slot] = descriptor;
    active_write_offset = descriptor.data_end;
    write_buf_offset = active_write_offset;
    return ESP_OK;
}

esp_err_t on9rstore::write_active_sparse_index_unsafe()
{
    if (sparse_index_count == 0) {
        return sync_fd(writer_fd);
    }

    const size_t bytes =
        static_cast<size_t>(sparse_index_count) *
        sizeof(on9rstore_def::sparse_index_entry);
    esp_err_t ret = write_exact_fd(writer_fd, segment_file_size,
                                   active_segment.index_start,
                                   sparse_index.get(), bytes);
    if (ret == ESP_OK) {
        ret = sync_fd(writer_fd);
    }

    return ret;
}

esp_err_t on9rstore::seal_active_segment_unsafe()
{
    esp_err_t ret = flush_unsafe();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = write_active_sparse_index_unsafe();
    if (ret != ESP_OK) {
        return ret;
    }

    const segment_descriptor &descriptor =
        segments[active_segment.slot];
    on9rstore_def::segment_footer footer = {};
    footer.magic = on9rstore_def::segment_footer_magic;
    footer.revision = on9rstore_def::segment_footer_revision;
    footer.size = sizeof(footer);
    footer.store_id = store_id;
    footer.state = on9rstore_def::segment_footer_state_sealed;
    footer.slot = active_segment.slot;
    footer.generation = active_segment.generation;
    footer.first_entry_id = descriptor.first_entry_id;
    footer.last_entry_id = descriptor.last_entry_id;
    footer.entry_count = descriptor.entry_count;
    footer.data_end = descriptor.data_end;
    footer.index_count = sparse_index_count;
    footer.index_stride = active_segment.index_stride;

    ret = write_segment_footers(writer_fd, footer);
    if (ret == ESP_OK) {
        segments[active_segment.slot].sealed = true;
        segments[active_segment.slot].index_count =
            sparse_index_count;
    }

    return ret;
}

esp_err_t on9rstore::seal_recovered_segment(uint32_t slot)
{
    esp_err_t ret = open_reader_segment(slot);
    if (ret != ESP_OK) {
        return ret;
    }

    on9rstore_def::segment_header header = {};
    ret = load_segment_header(reader_fd, slot, &header);
    if (ret == ESP_OK) {
        ret = scan_segment_entries(reader_fd, header, &segments[slot],
                                   true);
    }
    close_reader_segment();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = open_active_segment(slot);
    if (ret != ESP_OK) {
        return ret;
    }

    active_segment = header;
    active_write_offset = segments[slot].data_end;
    ret = seal_active_segment_unsafe();
    close_writer_segment();
    return ret;
}

void on9rstore::update_oldest_segment_generation()
{
    uint64_t oldest = 0;
    for (uint32_t slot = 0; slot < segment_count; slot += 1) {
        if (!segments[slot].valid) {
            continue;
        }
        if (oldest == 0 || segments[slot].generation < oldest) {
            oldest = segments[slot].generation;
        }
    }

    state.oldest_segment_generation = oldest;
}

void on9rstore::update_recovered_manifest_state()
{
    uint64_t used_size = 0;
    uint64_t newest_id = 0;
    uint64_t highest_generation = 0;

    for (uint32_t slot = 0; slot < segment_count; slot += 1) {
        const segment_descriptor &descriptor = segments[slot];
        if (!descriptor.valid) {
            continue;
        }

        used_size +=
            descriptor.data_end -
            on9rstore_def::segment_header_region_size;
        if (descriptor.last_entry_id > newest_id) {
            newest_id = descriptor.last_entry_id;
        }
        if (descriptor.generation > highest_generation) {
            highest_generation = descriptor.generation;
        }
    }

    state.used_size = used_size;
    state.newest_entry_id = newest_id;
    newest_entry_id = newest_id;
    if (newest_id != 0) {
        const uint32_t entry_boot =
            static_cast<uint32_t>(newest_id >> 40ULL);
        if (entry_boot > state.boot_counter) {
            state.boot_counter = entry_boot;
            state.next_entry_sequence =
                newest_id & on9rstore_def::entry_id_sequence_mask;
        } else if (entry_boot == state.boot_counter) {
            const uint64_t entry_sequence =
                newest_id & on9rstore_def::entry_id_sequence_mask;
            if (entry_sequence > state.next_entry_sequence) {
                state.next_entry_sequence = entry_sequence;
            }
        }
    }

    if (state.next_segment_generation <= highest_generation) {
        state.next_segment_generation = highest_generation + 1;
    }
    state.active_slot = active_segment.slot;
    state.active_segment_generation = active_segment.generation;
    update_oldest_segment_generation();
}

esp_err_t on9rstore::recover_all_segments()
{
    uint64_t highest_generation = 0;
    uint32_t highest_slot = 0;

    for (uint32_t slot = 0; slot < segment_count; slot += 1) {
        esp_err_t ret = recover_one_segment(slot);
        if (ret != ESP_OK) {
            return ret;
        }

        if (segments[slot].valid &&
            segments[slot].generation > highest_generation) {
            highest_generation = segments[slot].generation;
            highest_slot = slot;
        }
    }

    if (highest_generation == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    state.active_slot = highest_slot;
    state.active_segment_generation = highest_generation;
    if (state.next_segment_generation <= highest_generation) {
        state.next_segment_generation = highest_generation + 1;
    }

    for (uint32_t slot = 0; slot < segment_count; slot += 1) {
        if (segments[slot].valid && !segments[slot].sealed &&
            slot != highest_slot) {
            esp_err_t ret = seal_recovered_segment(slot);
            if (ret != ESP_OK) {
                return ret;
            }
        }
    }

    esp_err_t ret = select_or_create_active_segment();
    if (ret != ESP_OK) {
        return ret;
    }

    update_recovered_manifest_state();
    return commit_manifest_superblock_unsafe();
}

esp_err_t on9rstore::select_or_create_active_segment()
{
    uint64_t highest_generation = 0;
    uint32_t highest_slot = 0;
    for (uint32_t slot = 0; slot < segment_count; slot += 1) {
        if (segments[slot].valid &&
            segments[slot].generation > highest_generation) {
            highest_generation = segments[slot].generation;
            highest_slot = slot;
        }
    }

    esp_err_t ret = open_reader_segment(highest_slot);
    if (ret != ESP_OK) {
        return ret;
    }

    on9rstore_def::segment_header header = {};
    ret = load_segment_header(reader_fd, highest_slot, &header);
    close_reader_segment();
    if (ret != ESP_OK) {
        return ret;
    }

    if (!segments[highest_slot].sealed) {
        return recover_open_segment(highest_slot, header);
    }

    active_segment = header;
    state.active_slot = highest_slot;
    state.active_segment_generation = highest_generation;
    update_recovered_manifest_state();
    return open_next_segment_unsafe();
}

esp_err_t on9rstore::retire_segment_unsafe(uint32_t slot)
{
    if (!segments[slot].valid) {
        return ESP_OK;
    }

    const uint64_t data_size =
        segments[slot].data_end -
        on9rstore_def::segment_header_region_size;
    state.used_size =
        state.used_size > data_size ? state.used_size - data_size : 0;
    segments[slot] = {};
    update_oldest_segment_generation();
    return commit_manifest_superblock_unsafe();
}

esp_err_t on9rstore::activate_segment_unsafe(uint32_t slot,
                                            uint64_t generation)
{
    esp_err_t ret = open_active_segment(slot);
    if (ret != ESP_OK) {
        return ret;
    }

    on9rstore_def::segment_header header = {};
    if (!calculate_segment_geometry(segment_file_size, &header)) {
        return ESP_ERR_INVALID_SIZE;
    }

    header.magic = on9rstore_def::segment_header_magic;
    header.revision = on9rstore_def::segment_header_revision;
    header.size = sizeof(header);
    header.store_id = store_id;
    header.state = on9rstore_def::segment_state_active;
    header.slot = slot;
    header.generation = generation;
    ret = write_segment_headers(writer_fd, header);
    if (ret != ESP_OK) {
        return ret;
    }

    active_segment = header;
    active_write_offset = header.data_start;
    write_buf_offset = active_write_offset;
    write_buf_pos = 0;
    sparse_index_count = 0;

    segment_descriptor descriptor = {};
    descriptor.valid = true;
    descriptor.slot = slot;
    descriptor.generation = generation;
    descriptor.data_end = header.data_start;
    segments[slot] = descriptor;
    return ESP_OK;
}

esp_err_t on9rstore::open_next_segment_unsafe()
{
    const uint32_t next_slot =
        (state.active_slot + 1) % segment_count;
    const uint64_t next_generation =
        state.next_segment_generation;
    if (next_generation == 0 || next_generation == UINT64_MAX) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = retire_segment_unsafe(next_slot);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = activate_segment_unsafe(next_slot, next_generation);
    if (ret != ESP_OK) {
        return ret;
    }

    state.active_slot = next_slot;
    state.active_segment_generation = next_generation;
    state.next_segment_generation = next_generation + 1;
    update_oldest_segment_generation();
    return commit_manifest_superblock_unsafe();
}

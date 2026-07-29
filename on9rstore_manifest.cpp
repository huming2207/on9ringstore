#include <cerrno>
#include <cstring>
#include <inttypes.h>
#include <sys/stat.h>

#include <esp_log.h>
#include <esp_random.h>
#include <sdkconfig.h>

#include "on9rstore.hpp"

uint64_t on9rstore::get_manifest_file_size(uint32_t anchor_count) const
{
    return on9rstore_def::manifest_superblock_region_size +
           static_cast<uint64_t>(anchor_count) * on9rstore_def::time_anchor_slot_size;
}

void on9rstore::apply_manifest_geometry()
{
    store_id = state.store_id;
    segment_count = state.segment_count;
    segment_file_size = state.segment_size;
    time_anchor_count = state.time_anchor_count;
    manifest_file_size = get_manifest_file_size(time_anchor_count);
}

void on9rstore::log_kconfig_geometry_mismatch() const
{
    if (segment_file_size != CONFIG_ON9RSTORE_SPARSE_FILE_SIZE) {
        ESP_LOGW(TAG, "Manifest: file size %" PRIu64 " overrides Kconfig value %d", segment_file_size,
                 CONFIG_ON9RSTORE_SPARSE_FILE_SIZE);
    }

    if (segment_count != CONFIG_ON9STORE_SPARSE_FILE_CNT) {
        ESP_LOGW(TAG, "Manifest: file count %" PRIu32 " overrides Kconfig value %d", segment_count,
                 CONFIG_ON9STORE_SPARSE_FILE_CNT);
    }

    if (time_anchor_count != CONFIG_ON9RSTORE_TIME_ANCHOR_CNT) {
        ESP_LOGW(TAG, "Manifest: anchor count %" PRIu32 " overrides Kconfig value %d", time_anchor_count,
                 CONFIG_ON9RSTORE_TIME_ANCHOR_CNT);
    }
}

esp_err_t on9rstore::initialise_manifest_superblock()
{
    uint16_t new_store_id = 0;
    while (new_store_id == 0) {
        new_store_id = static_cast<uint16_t>(esp_random());
    }

    state = {};
    state.magic = on9rstore_def::manifest_magic;
    state.revision = on9rstore_def::manifest_revision;
    state.size = sizeof(state);
    state.store_id = new_store_id;
    state.state = on9rstore_def::manifest_state_provisioning_owned;
    state.segment_count = CONFIG_ON9STORE_SPARSE_FILE_CNT;
    state.segment_size = CONFIG_ON9RSTORE_SPARSE_FILE_SIZE;
    state.time_anchor_count = CONFIG_ON9RSTORE_TIME_ANCHOR_CNT;
    state.time_anchor_slot_size = on9rstore_def::time_anchor_slot_size;
    state.sparse_index_stride = on9rstore_def::sparse_index_stride;
    state.active_slot = 0;
    state.next_segment_generation = 1;

    apply_manifest_geometry();
    return ESP_OK;
}

esp_err_t on9rstore::commit_manifest_superblock_unsafe()
{
    if (manifest_fd < 0 || state.store_id == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (state.generation == UINT64_MAX) {
        return ESP_ERR_INVALID_STATE;
    }

    state.generation += 1;
    state.checksum = 0;
    state.checksum = calc_crc32(reinterpret_cast<const uint8_t *>(&state), sizeof(state));

    const uint32_t next_slot = static_cast<uint32_t>(state.generation % on9rstore_def::manifest_superblock_slot_count);
    const uint64_t offset = static_cast<uint64_t>(next_slot) * on9rstore_def::manifest_superblock_slot_size;

    esp_err_t ret = write_exact_fd(manifest_fd, manifest_file_size, offset, &state, sizeof(state));
    if (ret == ESP_OK) {
        ret = sync_fd(manifest_fd);
    }
    if (ret == ESP_OK) {
        manifest_slot = next_slot;
    }

    return ret;
}

esp_err_t on9rstore::write_initial_manifest_copies()
{
    state.generation = 0;
    esp_err_t ret = commit_manifest_superblock_unsafe();
    if (ret != ESP_OK) {
        return ret;
    }

    return commit_manifest_superblock_unsafe();
}

esp_err_t on9rstore::create_manifest()
{
    esp_err_t ret = verify_new_store_namespace_empty();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = initialise_manifest_superblock();
    if (ret != ESP_OK) {
        return ret;
    }

    bool created = false;
    ret = provision_contiguous_file(manifest_path, manifest_file_size, &created);
    if (ret != ESP_OK) {
        return ret;
    }
    if (!created) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = open_file(manifest_path, &manifest_fd);
    if (ret != ESP_OK) {
        return ret;
    }

    manifest_created = true;
    return write_initial_manifest_copies();
}

bool on9rstore::is_manifest_superblock_valid(const on9rstore_def::manifest_superblock &candidate) const
{
    if (candidate.magic != on9rstore_def::manifest_magic || candidate.revision != on9rstore_def::manifest_revision ||
        candidate.size != sizeof(candidate) || candidate.store_id == 0 || candidate.segment_count < 2 ||
        candidate.segment_count > 9999 || candidate.time_anchor_count == 0 || candidate.time_anchor_count > 65535 ||
        candidate.time_anchor_slot_size != on9rstore_def::time_anchor_slot_size ||
        candidate.sparse_index_stride != on9rstore_def::sparse_index_stride || candidate.active_slot >= candidate.segment_count ||
        candidate.time_anchor_write_index >= candidate.time_anchor_count ||
        candidate.time_anchor_used > candidate.time_anchor_count ||
        candidate.next_entry_sequence > on9rstore_def::entry_id_sequence_mask ||
        candidate.boot_counter > on9rstore_def::entry_id_boot_mask) {
        return false;
    }

    if (candidate.state != on9rstore_def::manifest_state_provisioning_owned &&
        candidate.state != on9rstore_def::manifest_state_ready) {
        return false;
    }
    if (candidate.state == on9rstore_def::manifest_state_ready &&
        (candidate.active_segment_generation == 0 || candidate.next_segment_generation <= candidate.active_segment_generation ||
         candidate.oldest_segment_generation == 0 || candidate.oldest_segment_generation > candidate.active_segment_generation)) {
        return false;
    }

    on9rstore_def::segment_header geometry = {};
    if (!calculate_segment_geometry(candidate.segment_size, &geometry)) {
        return false;
    }

    const uint64_t expected_size = get_manifest_file_size(candidate.time_anchor_count);
    if (expected_size != manifest_file_size) {
        return false;
    }

    on9rstore_def::manifest_superblock copy = candidate;
    const uint32_t expected_crc = copy.checksum;
    copy.checksum = 0;
    return calc_crc32(reinterpret_cast<const uint8_t *>(&copy), sizeof(copy)) == expected_crc;
}

esp_err_t on9rstore::load_manifest_superblock()
{
    bool found = false;
    on9rstore_def::manifest_superblock newest = {};
    uint32_t newest_slot = 0;

    for (uint32_t slot = 0; slot < on9rstore_def::manifest_superblock_slot_count; slot += 1) {
        on9rstore_def::manifest_superblock candidate = {};
        const uint64_t offset = static_cast<uint64_t>(slot) * on9rstore_def::manifest_superblock_slot_size;
        esp_err_t ret = read_exact_fd(manifest_fd, manifest_file_size, offset, &candidate, sizeof(candidate));
        if (ret != ESP_OK) {
            return ret;
        }

        if (is_manifest_superblock_valid(candidate) && (!found || candidate.generation > newest.generation)) {
            newest = candidate;
            newest_slot = slot;
            found = true;
        }
    }

    if (!found) {
        return ESP_ERR_INVALID_STATE;
    }

    state = newest;
    manifest_slot = newest_slot;
    apply_manifest_geometry();
    return ESP_OK;
}

esp_err_t on9rstore::open_existing_manifest()
{
    struct stat file_stat = {};
    if (stat(manifest_path, &file_stat) != 0 || file_stat.st_size <= 0) {
        return ESP_FAIL;
    }

    manifest_file_size = static_cast<uint64_t>(file_stat.st_size);
    esp_err_t ret = open_file(manifest_path, &manifest_fd);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = load_manifest_superblock();
    if (ret != ESP_OK) {
        return ret;
    }

    ret = validate_contiguous_file(manifest_path, manifest_file_size);
    if (ret == ESP_OK) {
        log_kconfig_geometry_mismatch();
    }

    return ret;
}

esp_err_t on9rstore::setup_manifest()
{
    struct stat file_stat = {};
    const int stat_result = stat(manifest_path, &file_stat);
    if (stat_result == 0) {
        return file_stat.st_size > 0 ? open_existing_manifest() : create_manifest();
    }

    if (errno != ENOENT) {
        ESP_LOGE(TAG, "Manifest: stat failed: errno=%d", errno);
        return ESP_FAIL;
    }

    return create_manifest();
}

esp_err_t on9rstore::read_time_anchor_slot(on9rstore_def::time_anchor_entry *entry_out, uint32_t slot) const
{
    if (entry_out == nullptr || slot >= time_anchor_count) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint64_t offset =
        on9rstore_def::manifest_superblock_region_size + static_cast<uint64_t>(slot) * on9rstore_def::time_anchor_slot_size;
    return read_exact_fd(manifest_fd, manifest_file_size, offset, entry_out, sizeof(*entry_out));
}

esp_err_t on9rstore::write_time_anchor_slot(const on9rstore_def::time_anchor_entry &entry, uint32_t slot)
{
    if (slot >= time_anchor_count) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint64_t offset =
        on9rstore_def::manifest_superblock_region_size + static_cast<uint64_t>(slot) * on9rstore_def::time_anchor_slot_size;
    esp_err_t ret = write_exact_fd(manifest_fd, manifest_file_size, offset, &entry, sizeof(entry));
    if (ret == ESP_OK) {
        ret = sync_fd(manifest_fd);
    }

    return ret;
}

bool on9rstore::is_time_anchor_valid(const on9rstore_def::time_anchor_entry &candidate) const
{
    if (candidate.magic != on9rstore_def::time_anchor_magic || candidate.revision != on9rstore_def::time_anchor_revision ||
        candidate.size != sizeof(candidate) || candidate.store_id != store_id || candidate.sequence == 0 ||
        candidate.boot_counter == 0 || candidate.boot_counter > on9rstore_def::entry_id_boot_mask || candidate.reserved != 0 ||
        candidate.source_mask == 0 || (candidate.source_mask & ~on9rstore_def::TIME_SOURCE_ALL) != 0 ||
        candidate.source_count == 0 || candidate.source_count < count_bits(candidate.source_mask) ||
        candidate.quality > on9rstore_def::TIME_ANCHOR_QUALITY_CONFIRMED || candidate.utc_us == 0 ||
        candidate.replace_sequence >= candidate.sequence) {
        return false;
    }

    const uint16_t known_flags = on9rstore_def::TIME_ANCHOR_FLAG_CONSENSUS | on9rstore_def::TIME_ANCHOR_FLAG_PPS |
                                 on9rstore_def::TIME_ANCHOR_FLAG_CONTINUITY;
    if ((candidate.flags & ~known_flags) != 0) {
        return false;
    }

    on9rstore_def::time_anchor_entry copy = candidate;
    const uint32_t expected_crc = copy.checksum;
    copy.checksum = 0;
    return calc_crc32(reinterpret_cast<const uint8_t *>(&copy), sizeof(copy)) == expected_crc;
}

void on9rstore::insert_time_model_anchor(const on9rstore_def::time_anchor_entry &anchor, uint32_t slot)
{
    uint32_t position = time_model_epoch_count;
    while (position > 0 && time_model_epochs[position - 1].anchor.sequence > anchor.sequence) {
        time_model_epochs[position] = time_model_epochs[position - 1];
        position -= 1;
    }

    time_model_epoch &model = time_model_epochs[position];
    model = {};
    model.anchor = anchor;
    model.slot = slot;
    time_model_epoch_count += 1;
}

int32_t on9rstore::find_time_model_sequence(uint64_t sequence, uint32_t limit) const
{
    for (uint32_t index = 0; index < limit; index += 1) {
        if (time_model_epochs[index].anchor.sequence == sequence) {
            return static_cast<int32_t>(index);
        }
    }

    return -1;
}

void on9rstore::resolve_time_model_replacements()
{
    for (uint32_t index = 0; index < time_model_epoch_count; index += 1) {
        time_model_epoch &model = time_model_epochs[index];
        model.replacement_root_sequence = model.anchor.sequence;
        model.effective_uptime_us = model.anchor.monotonic_us;
        model.effective = true;
        if (model.anchor.replace_sequence == 0) {
            continue;
        }

        const int32_t target_index = find_time_model_sequence(model.anchor.replace_sequence, index);
        if (target_index < 0 || time_model_epochs[target_index].anchor.boot_counter != model.anchor.boot_counter) {
            model.replacement_root_sequence = 0;
            model.effective = false;
            continue;
        }

        const time_model_epoch &target = time_model_epochs[target_index];
        if (target.replacement_root_sequence == 0) {
            model.replacement_root_sequence = 0;
            model.effective = false;
            continue;
        }
        model.replacement_root_sequence = target.replacement_root_sequence;
        model.effective_uptime_us = target.effective_uptime_us;
        for (uint32_t prior = 0; prior < index; prior += 1) {
            if (time_model_epochs[prior].anchor.boot_counter == model.anchor.boot_counter &&
                time_model_epochs[prior].replacement_root_sequence == model.replacement_root_sequence) {
                time_model_epochs[prior].effective = false;
            }
        }
    }
}

bool on9rstore::is_time_model_epoch_before(const time_model_epoch &left, const time_model_epoch &right)
{
    if (left.anchor.boot_counter != right.anchor.boot_counter) {
        return left.anchor.boot_counter < right.anchor.boot_counter;
    }
    if (left.effective_uptime_us != right.effective_uptime_us) {
        return left.effective_uptime_us < right.effective_uptime_us;
    }
    return left.anchor.sequence < right.anchor.sequence;
}

void on9rstore::compact_and_sort_time_model_epochs()
{
    uint32_t compacted_count = 0;
    for (uint32_t index = 0; index < time_model_epoch_count; index += 1) {
        if (time_model_epochs[index].effective) {
            time_model_epochs[compacted_count] = time_model_epochs[index];
            compacted_count += 1;
        }
    }
    time_model_epoch_count = compacted_count;

    for (uint32_t index = 1; index < time_model_epoch_count; index += 1) {
        const time_model_epoch value = time_model_epochs[index];
        uint32_t position = index;
        while (position > 0 && is_time_model_epoch_before(value, time_model_epochs[position - 1])) {
            time_model_epochs[position] = time_model_epochs[position - 1];
            position -= 1;
        }
        time_model_epochs[position] = value;
    }

    compacted_count = 0;
    for (uint32_t index = 0; index < time_model_epoch_count; index += 1) {
        if (compacted_count > 0 &&
            time_model_epochs[compacted_count - 1].anchor.boot_counter == time_model_epochs[index].anchor.boot_counter &&
            time_model_epochs[compacted_count - 1].effective_uptime_us == time_model_epochs[index].effective_uptime_us) {
            time_model_epochs[compacted_count - 1] = time_model_epochs[index];
        } else {
            time_model_epochs[compacted_count] = time_model_epochs[index];
            compacted_count += 1;
        }
    }
    time_model_epoch_count = compacted_count;
}

void on9rstore::set_time_model_epoch_bounds()
{
    for (uint32_t index = 0; index < time_model_epoch_count; index += 1) {
        time_model_epoch &model = time_model_epochs[index];
        const bool first_for_boot = index == 0 || time_model_epochs[index - 1].anchor.boot_counter != model.anchor.boot_counter;
        const bool last_for_boot =
            index + 1 == time_model_epoch_count || time_model_epochs[index + 1].anchor.boot_counter != model.anchor.boot_counter;

        model.first_uptime_us = first_for_boot ? 0 : model.effective_uptime_us;
        model.last_uptime_us = last_for_boot ? UINT64_MAX : time_model_epochs[index + 1].effective_uptime_us - 1;
    }
}

esp_err_t on9rstore::load_time_model_catalog()
{
    uint64_t newest_sequence = 0;
    uint32_t newest_slot = 0;
    time_model_epoch_count = 0;

    for (uint32_t slot = 0; slot < time_anchor_count; slot += 1) {
        on9rstore_def::time_anchor_entry candidate = {};
        esp_err_t ret = read_time_anchor_slot(&candidate, slot);
        if (ret != ESP_OK) {
            time_model_epoch_count = 0;
            return ret;
        }

        if (!is_time_anchor_valid(candidate)) {
            continue;
        }

        insert_time_model_anchor(candidate, slot);
        if (candidate.sequence > newest_sequence) {
            newest_sequence = candidate.sequence;
            newest_slot = slot;
        }
    }

    state.next_time_anchor_sequence = newest_sequence;
    state.time_anchor_used = time_model_epoch_count;
    state.time_anchor_write_index = newest_sequence == 0 ? 0 : (newest_slot + 1) % time_anchor_count;
    resolve_time_model_replacements();
    compact_and_sort_time_model_epochs();
    set_time_model_epoch_bounds();
    return ESP_OK;
}

esp_err_t on9rstore::recover_time_anchor_ring()
{
    esp_err_t ret = load_time_model_catalog();
    if (ret != ESP_OK) {
        return ret;
    }

    return commit_manifest_superblock_unsafe();
}

bool on9rstore::is_time_anchor_input_valid(const on9rstore_def::time_anchor &anchor) const
{
    const uint32_t known_sources = anchor.source_mask & on9rstore_def::TIME_SOURCE_ALL;
    if (known_sources == 0 || known_sources != anchor.source_mask || anchor.source_count == 0 ||
        anchor.source_count < count_bits(known_sources) || anchor.quality > on9rstore_def::TIME_ANCHOR_QUALITY_CONFIRMED ||
        anchor.utc_us == 0 || anchor.replace_sequence > state.next_time_anchor_sequence) {
        return false;
    }

    const uint16_t known_flags = on9rstore_def::TIME_ANCHOR_FLAG_CONSENSUS | on9rstore_def::TIME_ANCHOR_FLAG_PPS |
                                 on9rstore_def::TIME_ANCHOR_FLAG_CONTINUITY;
    return (anchor.flags & ~known_flags) == 0;
}

esp_err_t on9rstore::validate_time_anchor_replacement(const on9rstore_def::time_anchor &anchor) const
{
    if (anchor.replace_sequence == 0) {
        return ESP_OK;
    }

    for (uint32_t slot = 0; slot < time_anchor_count; slot += 1) {
        on9rstore_def::time_anchor_entry candidate = {};
        esp_err_t ret = read_time_anchor_slot(&candidate, slot);
        if (ret != ESP_OK) {
            return ret;
        }
        if (is_time_anchor_valid(candidate) && candidate.sequence == anchor.replace_sequence) {
            return candidate.boot_counter == state.boot_counter ? ESP_OK : ESP_ERR_INVALID_ARG;
        }
    }

    return ESP_ERR_INVALID_ARG;
}

bool on9rstore::is_retained_boot(uint32_t boot_counter) const
{
    for (uint32_t slot = 0; slot < segment_count; slot += 1) {
        const segment_descriptor &segment = segments[slot];
        if (segment.valid && segment.entry_count > 0 && get_entry_boot_counter(segment.first_entry_id) <= boot_counter &&
            get_entry_boot_counter(segment.last_entry_id) >= boot_counter) {
            return true;
        }
    }

    return false;
}

esp_err_t on9rstore::check_time_anchor_slot_reuse(uint32_t slot) const
{
    on9rstore_def::time_anchor_entry candidate = {};
    esp_err_t ret = read_time_anchor_slot(&candidate, slot);
    if (ret != ESP_OK) {
        return ret;
    }
    if (!is_time_anchor_valid(candidate) || !is_retained_boot(candidate.boot_counter)) {
        return ESP_OK;
    }

    return ESP_ERR_NO_MEM;
}

esp_err_t on9rstore::append_time_anchor_unsafe(const on9rstore_def::time_anchor &anchor)
{
    if (!is_time_anchor_input_valid(anchor)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (state.next_time_anchor_sequence == UINT64_MAX) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = validate_time_anchor_replacement(anchor);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = check_time_anchor_slot_reuse(state.time_anchor_write_index);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = flush_unsafe();
    if (ret != ESP_OK) {
        return ret;
    }

    on9rstore_def::time_anchor_entry entry = {};
    entry.magic = on9rstore_def::time_anchor_magic;
    entry.revision = on9rstore_def::time_anchor_revision;
    entry.size = sizeof(entry);
    entry.store_id = store_id;
    entry.source_count = anchor.source_count;
    entry.quality = anchor.quality;
    entry.flags = anchor.flags;
    entry.source_mask = anchor.source_mask;
    entry.boot_counter = state.boot_counter;
    entry.sequence = state.next_time_anchor_sequence + 1;
    entry.monotonic_us = anchor.monotonic_us;
    entry.utc_us = anchor.utc_us;
    entry.uncertainty_us = anchor.uncertainty_us;
    entry.max_durable_entry_id = newest_entry_id;
    entry.replace_sequence = anchor.replace_sequence;
    entry.checksum = calc_crc32(reinterpret_cast<const uint8_t *>(&entry), sizeof(entry));

    const uint32_t slot = state.time_anchor_write_index;
    ret = write_time_anchor_slot(entry, slot);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = load_time_model_catalog();
    if (ret != ESP_OK) {
        return ret;
    }

    return commit_manifest_superblock_unsafe();
}

esp_err_t on9rstore::append_time_anchor(const on9rstore_def::time_anchor &anchor, uint32_t timeout_ticks)
{
    esp_err_t ret = acquire_operation_lock(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    if (xSemaphoreTake(read_lock, timeout_ticks) != pdTRUE) {
        release_operation_lock();
        return ESP_ERR_TIMEOUT;
    }

    ret = append_time_anchor_unsafe(anchor);
    xSemaphoreGive(read_lock);
    release_operation_lock();
    return ret;
}

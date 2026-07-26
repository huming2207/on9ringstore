#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <inttypes.h>
#include <limits>
#include <new>
#include <sys/stat.h>
#include <unistd.h>

#include <esp_log.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_vfs_fat.h>
#include <sdkconfig.h>

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
#include <esp_core_dump.h>
#include <esp_partition.h>
#endif

#include "on9rstore.hpp"

static_assert(sizeof(on9rstore_def::store_header) <= on9rstore_def::store_header_slot_size);
static_assert(sizeof(on9rstore_def::metadata) <= on9rstore_def::metadata_slot_size);

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

    // Destruction still requires external lifetime ownership: no task may call a
    // method after the C++ object itself begins destruction. deinit() keeps these
    // locks alive so queued callers can observe the stopped state safely.
    if (write_lock != nullptr) {
        vSemaphoreDelete(write_lock);
        write_lock = nullptr;
    }

    if (lifecycle_lock != nullptr) {
        vSemaphoreDelete(lifecycle_lock);
        lifecycle_lock = nullptr;
    }
}

esp_err_t on9rstore::init()
{
    if (lifecycle_lock == nullptr) {
        lifecycle_lock = xSemaphoreCreateMutex();
        if (lifecycle_lock == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (xSemaphoreTake(lifecycle_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;
    if (initialized || shutting_down) {
        ret = ESP_ERR_INVALID_STATE;
        goto exit;
    }

    if (file_path == nullptr || cfg.base_path == nullptr || cfg.file_size <= on9rstore_def::control_region_size ||
        cfg.write_buffer_size < on9rstore_def::min_entry_size ||
        ((cfg.file_size - on9rstore_def::control_region_size) % on9rstore_def::entry_alignment) != 0) {
        ret = ESP_ERR_INVALID_ARG;
        goto exit;
    }

    {
        const size_t base_path_len = strlen(cfg.base_path);
        if (base_path_len == 0 || strncmp(file_path, cfg.base_path, base_path_len) != 0 ||
            (base_path_len > 1 && file_path[base_path_len] != '/')) {
            ESP_LOGE(TAG, "Init: file path must be below base path: %s / %s", cfg.base_path, file_path);
            ret = ESP_ERR_INVALID_ARG;
            goto exit;
        }
    }

    if (cfg.file_size > static_cast<uint64_t>(std::numeric_limits<off_t>::max())) {
        ESP_LOGE(TAG, "Init: file size too large for VFS off_t: %" PRIu64, cfg.file_size);
        ret = ESP_ERR_INVALID_SIZE;
        goto exit;
    }

    if (write_lock == nullptr) {
        write_lock = xSemaphoreCreateMutex();
        if (write_lock == nullptr) {
            ret = ESP_ERR_NO_MEM;
            goto exit;
        }
    }

    write_buf.reset(new (std::nothrow) uint8_t[cfg.write_buffer_size]);
    if (write_buf == nullptr) {
        ret = ESP_ERR_NO_MEM;
        goto exit;
    }

    write_buf_pos = 0;
    write_buf_offset = 0;
    metadata_dirty = false;
    staging_file_active = false;
    data_size = cfg.file_size - on9rstore_def::control_region_size;

    ret = setup_store_file();
    if (ret != ESP_OK) {
        goto cleanup;
    }

    if (staging_file_active) {
        ret = write_store_headers_unsafe();
        if (ret != ESP_OK) {
            goto cleanup;
        }

        ret = initialise_empty_metadata_unsafe();
        if (ret != ESP_OK) {
            goto cleanup;
        }

        ret = publish_staging_file();
        if (ret != ESP_OK) {
            goto cleanup;
        }
    } else {
        ret = load_store_header_unsafe();
        if (ret != ESP_OK) {
            goto cleanup;
        }
    }

    ret = load_metadata();
    if (ret != ESP_OK) {
        goto cleanup;
    }

    ret = increase_boot_counter();
    if (ret != ESP_OK) {
        goto cleanup;
    }

    initialized = true;
    if (xSemaphoreTake(write_lock, portMAX_DELAY) != pdTRUE) {
        ret = ESP_ERR_TIMEOUT;
        goto cleanup;
    }

    ret = append_boot_entry_unsafe();
    xSemaphoreGive(write_lock);
    if (ret != ESP_OK) {
        goto cleanup;
    }

    ESP_LOGI(TAG, "Init: OK, boot=%" PRIu32 ", file=%" PRIu64 " bytes", state.boot_counter, cfg.file_size);
    goto exit;

cleanup:
    initialized = false;
    close_storage_unsafe();

exit:
    xSemaphoreGive(lifecycle_lock);
    return ret;
}

esp_err_t on9rstore::setup_store_file()
{
    struct stat file_stat = {};
    if (stat(file_path, &file_stat) == 0) {
        if (static_cast<uint64_t>(file_stat.st_size) != cfg.file_size) {
            ESP_LOGE(TAG, "Init: existing file size %" PRIu64 " does not match configured %" PRIu64,
                     static_cast<uint64_t>(file_stat.st_size), cfg.file_size);
            return ESP_ERR_INVALID_SIZE;
        }

        return open_store_file(file_path);
    }

    if (errno != ENOENT) {
        ESP_LOGE(TAG, "Init: stat(%s) failed: errno=%d", file_path, errno);
        return ESP_FAIL;
    }

    return prepare_staging_file();
}

esp_err_t on9rstore::open_store_file(const char *path)
{
    if (path == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    struct stat file_stat = {};
    if (stat(path, &file_stat) != 0 || static_cast<uint64_t>(file_stat.st_size) != cfg.file_size) {
        ESP_LOGE(TAG, "Init: %s has unexpected size: errno=%d", path, errno);
        return ESP_ERR_INVALID_SIZE;
    }

    bool is_contiguous = false;
    auto ret = esp_vfs_fat_test_contiguous_file(cfg.base_path, path, &is_contiguous);
    if (ret != ESP_OK || !is_contiguous) {
        ESP_LOGE(TAG, "Init: %s is not contiguous: ret=0x%x", path, ret);
        return ret != ESP_OK ? ret : ESP_ERR_INVALID_STATE;
    }

    fd = open(path, O_RDWR);
    if (fd < 0) {
        ESP_LOGE(TAG, "Init: open(%s) failed: errno=%d", path, errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t on9rstore::prepare_staging_file()
{
    constexpr char staging_suffix[] = ".on9rstore-new";
    const int path_len = std::snprintf(staging_file_path, sizeof(staging_file_path), "%s%s", file_path, staging_suffix);
    if (path_len < 0 || static_cast<size_t>(path_len) >= sizeof(staging_file_path)) {
        ESP_LOGE(TAG, "Init: staging path is too long");
        return ESP_ERR_INVALID_SIZE;
    }

    struct stat file_stat = {};
    if (stat(staging_file_path, &file_stat) == 0) {
        if (static_cast<uint64_t>(file_stat.st_size) == cfg.file_size) {
            auto ret = open_store_file(staging_file_path);
            if (ret == ESP_OK) {
                staging_file_active = true;
                return ESP_OK;
            }

            if (ret != ESP_ERR_INVALID_SIZE && ret != ESP_ERR_INVALID_STATE) {
                return ret;
            }
        }

        if (unlink(staging_file_path) != 0) {
            ESP_LOGE(TAG, "Init: remove stale staging file failed: errno=%d", errno);
            return ESP_FAIL;
        }
    } else if (errno != ENOENT) {
        ESP_LOGE(TAG, "Init: stat(%s) failed: errno=%d", staging_file_path, errno);
        return ESP_FAIL;
    }

    auto ret = esp_vfs_fat_create_contiguous_file(cfg.base_path, staging_file_path, cfg.file_size, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Init: failed to pre-allocate %s: ret=0x%x errno=%d", staging_file_path, ret, errno);
        return ret;
    }

    ret = open_store_file(staging_file_path);
    if (ret != ESP_OK) {
        return ret;
    }

    staging_file_active = true;
    return ESP_OK;
}

esp_err_t on9rstore::publish_staging_file()
{
    auto ret = sync_file();
    if (ret != ESP_OK) {
        return ret;
    }

    if (close(fd) != 0) {
        ESP_LOGE(TAG, "Init: close(%s) failed: errno=%d", staging_file_path, errno);
        fd = -1;
        return ESP_FAIL;
    }
    fd = -1;

    struct stat file_stat = {};
    if (stat(file_path, &file_stat) == 0 || errno != ENOENT) {
        ESP_LOGE(TAG, "Init: final file appeared while publishing staging file: errno=%d", errno);
        return ESP_ERR_INVALID_STATE;
    }

    if (std::rename(staging_file_path, file_path) != 0) {
        ESP_LOGE(TAG, "Init: rename(%s, %s) failed: errno=%d", staging_file_path, file_path, errno);
        return ESP_FAIL;
    }

    ret = open_store_file(file_path);
    if (ret != ESP_OK) {
        return ret;
    }

    staging_file_active = false;
    return ESP_OK;
}

esp_err_t on9rstore::write_store_headers_unsafe()
{
    do {
        store_id = static_cast<uint16_t>(esp_random());
    } while (store_id == 0);

    on9rstore_def::store_header header = {};
    header.magic = on9rstore_def::store_header_magic;
    header.revision = on9rstore_def::store_header_revision;
    header.size = sizeof(header);
    header.file_size = cfg.file_size;
    header.store_id = store_id;
    header.state = on9rstore_def::store_header_state_ready;
    header.checksum = calc_crc32(reinterpret_cast<const uint8_t *>(&header), sizeof(header));

    for (uint32_t slot = 0; slot < on9rstore_def::store_header_slot_count; slot += 1) {
        auto ret = write_exact(slot * on9rstore_def::store_header_slot_size, &header, sizeof(header));
        if (ret != ESP_OK) {
            return ret;
        }

        ret = sync_file();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t on9rstore::load_store_header_unsafe()
{
    bool found = false;
    uint16_t candidate_store_id = 0;

    for (uint32_t slot = 0; slot < on9rstore_def::store_header_slot_count; slot += 1) {
        on9rstore_def::store_header candidate = {};
        auto ret = read_exact(slot * on9rstore_def::store_header_slot_size, &candidate, sizeof(candidate));
        if (ret != ESP_OK) {
            return ret;
        }

        if (!is_store_header_valid(candidate)) {
            continue;
        }

        if (found && candidate.store_id != candidate_store_id) {
            ESP_LOGE(TAG, "Init: store header replicas disagree");
            return ESP_ERR_INVALID_STATE;
        }

        candidate_store_id = candidate.store_id;
        found = true;
    }

    if (!found) {
        ESP_LOGE(TAG, "Init: existing store has no valid v3 format header");
        return ESP_ERR_INVALID_STATE;
    }

    store_id = candidate_store_id;
    return ESP_OK;
}

esp_err_t on9rstore::load_metadata()
{
    bool found = false;
    on9rstore_def::metadata newest_state = {};
    uint32_t newest_slot = on9rstore_def::metadata_slot_count - 1;

    for (uint32_t slot = 0; slot < on9rstore_def::metadata_slot_count; slot += 1) {
        on9rstore_def::metadata candidate = {};
        auto ret = read_exact(on9rstore_def::store_header_region_size + slot * on9rstore_def::metadata_slot_size,
                              &candidate, sizeof(candidate));
        if (ret != ESP_OK) {
            return ret;
        }

        if (!is_metadata_valid(candidate)) {
            continue;
        }

        if (!found || candidate.generation > newest_state.generation) {
            newest_state = candidate;
            newest_slot = slot;
            found = true;
        }
    }

    if (found) {
        state = newest_state;
        metadata_slot = newest_slot;
        newest_entry_id = (static_cast<uint64_t>(state.boot_counter) << 40ULL) | state.next_entry_sequence;
        metadata_dirty = false;
        return ESP_OK;
    }

    auto ret = recover_from_entries_unsafe();
    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "Init: metadata unavailable; recovered state from entries");
        return persist_state_unsafe();
    }

    if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "Init: existing store has no valid metadata or recoverable v3 entries");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGE(TAG, "Init: metadata recovery failed: ret=0x%x", ret);
    return ret;
}

esp_err_t on9rstore::recover_from_entries_unsafe()
{
    if (write_buf == nullptr || data_size == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    bool found_normal = false;
    uint64_t oldest_entry_id = 0;
    uint64_t oldest_offset = 0;
    uint64_t newest_entry_id_found = 0;
    uint64_t newest_entry_end = 0;
    bool newest_marker_is_padding = false;

    uint64_t cache_offset = UINT64_MAX;
    size_t cache_len = 0;
    auto read_scanned_header = [&](uint64_t offset, on9rstore_def::entry_header *header_out) -> esp_err_t {
        if (header_out == nullptr || offset >= data_size ||
            (data_size - offset) < sizeof(on9rstore_def::entry_header)) {
            return ESP_ERR_INVALID_ARG;
        }

        if (cache_offset == UINT64_MAX || offset < cache_offset ||
            (offset - cache_offset) > cache_len ||
            sizeof(*header_out) > (cache_len - static_cast<size_t>(offset - cache_offset))) {
            cache_offset = offset;
            const uint64_t available = data_size - cache_offset;
            cache_len = available < cfg.write_buffer_size ? static_cast<size_t>(available) : cfg.write_buffer_size;
            auto ret = read_exact(data_file_offset(cache_offset), write_buf.get(), cache_len);
            if (ret != ESP_OK) {
                return ret;
            }
        }

        const size_t in_cache = static_cast<size_t>(offset - cache_offset);
        if (sizeof(*header_out) <= (cache_len - in_cache)) {
            memcpy(header_out, write_buf.get() + in_cache, sizeof(*header_out));
            return ESP_OK;
        }

        return read_entry_header_unsafe(offset, header_out);
    };

    for (uint64_t offset = 0; (data_size - offset) >= sizeof(on9rstore_def::entry_header);) {
        on9rstore_def::entry_header header = {};
        auto ret = read_scanned_header(offset, &header);
        if (ret != ESP_OK) {
            return ret;
        }

        if (!is_entry_header_valid(header)) {
            offset += on9rstore_def::entry_alignment;
            continue;
        }

        const uint64_t entry_size = get_entry_size(header.len);
        if (entry_size == 0 || entry_size > (data_size - offset)) {
            offset += on9rstore_def::entry_alignment;
            continue;
        }

        ret = validate_entry_unsafe(offset, header, entry_size);
        if (ret != ESP_OK) {
            if (ret != ESP_ERR_INVALID_CRC) {
                return ret;
            }

            offset += on9rstore_def::entry_alignment;
            continue;
        }

        const uint64_t entry_end = offset + entry_size;
        if (header.type == on9rstore_def::ENTRY_PADDING) {
            if (!is_entry_id_valid(header.entry_id) || entry_end != data_size) {
                offset += on9rstore_def::entry_alignment;
                continue;
            }

            if (found_normal && header.entry_id == newest_entry_id_found) {
                newest_entry_end = entry_end;
                newest_marker_is_padding = true;
            }
        } else if (is_entry_id_valid(header.entry_id)) {
            if (!found_normal || header.entry_id < oldest_entry_id) {
                oldest_entry_id = header.entry_id;
                oldest_offset = offset;
            }

            if (!found_normal || header.entry_id > newest_entry_id_found) {
                newest_entry_id_found = header.entry_id;
                newest_entry_end = entry_end;
                newest_marker_is_padding = false;
            }

            found_normal = true;
        }

        offset += entry_size;
    }

    if (!found_normal) {
        return ESP_ERR_NOT_FOUND;
    }

    const uint64_t recovered_write_offset = newest_marker_is_padding ? 0 : newest_entry_end;
    uint64_t recovered_used_size = 0;
    uint64_t walk_offset = oldest_offset;
    uint64_t previous_entry_id = 0;
    bool walked_any = false;

    while (!walked_any || walk_offset != recovered_write_offset) {
        walked_any = true;
        if (recovered_used_size >= data_size) {
            return ESP_ERR_INVALID_STATE;
        }

        const uint64_t remaining = data_size - walk_offset;
        if (remaining < on9rstore_def::min_entry_size) {
            recovered_used_size += remaining;
            walk_offset = 0;
            continue;
        }

        on9rstore_def::entry_header header = {};
        auto ret = read_entry_header_unsafe(walk_offset, &header);
        if (ret != ESP_OK) {
            return ret;
        }

        const uint64_t entry_size = get_entry_size(header.len);
        const bool entry_fits = is_entry_header_valid(header) && entry_size != 0 &&
                                 entry_size <= remaining && entry_size <= (data_size - recovered_used_size);
        const bool is_padding = entry_fits && header.type == on9rstore_def::ENTRY_PADDING &&
                                previous_entry_id != 0 && header.entry_id == previous_entry_id &&
                                (walk_offset + entry_size) == data_size;
        const bool is_normal = entry_fits && header.type != on9rstore_def::ENTRY_PADDING &&
                               is_entry_id_valid(header.entry_id) &&
                               (previous_entry_id == 0 || header.entry_id > previous_entry_id);

        if (is_padding || is_normal) {
            ret = validate_entry_unsafe(walk_offset, header, entry_size);
            if (ret != ESP_OK && ret != ESP_ERR_INVALID_CRC) {
                return ret;
            }

            if (ret == ESP_OK) {
                if (is_normal) {
                    previous_entry_id = header.entry_id;
                }

                recovered_used_size += entry_size;
                walk_offset += entry_size;
                if (walk_offset == data_size) {
                    walk_offset = 0;
                }
                continue;
            }
        }

        // A metadata-less recovery is deliberately conservative: bytes between
        // the oldest and newest complete entries remain occupied even when their
        // header is torn. Runtime head resynchronisation retires those spans once
        // they become oldest, so stale or damaged bytes cannot block future writes.
        recovered_used_size += on9rstore_def::entry_alignment;
        walk_offset += on9rstore_def::entry_alignment;
        if (walk_offset == data_size) {
            walk_offset = 0;
        }
    }

    if (previous_entry_id != newest_entry_id_found) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&state, 0, sizeof(state));
    state.file_size = cfg.file_size;
    state.store_id = store_id;
    state.oldest_offset = oldest_offset;
    state.write_offset = recovered_write_offset;
    state.used_size = recovered_used_size;
    state.boot_counter = static_cast<uint32_t>(newest_entry_id_found >> 40ULL);
    state.next_entry_sequence = newest_entry_id_found & on9rstore_def::entry_id_sequence_mask;
    metadata_slot = on9rstore_def::metadata_slot_count - 1;
    newest_entry_id = newest_entry_id_found;
    metadata_dirty = true;
    return ESP_OK;
}

esp_err_t on9rstore::initialise_empty_metadata_unsafe()
{
    memset(&state, 0, sizeof(state));
    state.file_size = cfg.file_size;
    state.store_id = store_id;
    metadata_slot = on9rstore_def::metadata_slot_count - 1;
    newest_entry_id = 0;
    metadata_dirty = true;

    auto ret = persist_state_unsafe();
    if (ret != ESP_OK) {
        return ret;
    }

    return persist_state_unsafe();
}

bool on9rstore::is_store_header_valid(const on9rstore_def::store_header &candidate) const
{
    const uint32_t expected_checksum = candidate.checksum;
    on9rstore_def::store_header crc_header = candidate;
    crc_header.checksum = 0;

    return candidate.magic == on9rstore_def::store_header_magic &&
           candidate.revision == on9rstore_def::store_header_revision && candidate.size == sizeof(candidate) &&
           candidate.file_size == cfg.file_size && candidate.store_id != 0 &&
           candidate.state == on9rstore_def::store_header_state_ready &&
           calc_crc32(reinterpret_cast<const uint8_t *>(&crc_header), sizeof(crc_header)) == expected_checksum;
}

bool on9rstore::is_metadata_valid(const on9rstore_def::metadata &candidate) const
{
    on9rstore_def::metadata crc_state = candidate;
    const uint32_t expected_checksum = crc_state.checksum;
    crc_state.checksum = 0;

    return store_id != 0 && candidate.magic == on9rstore_def::metadata_magic &&
           candidate.revision == on9rstore_def::metadata_revision && candidate.size == sizeof(candidate) &&
           candidate.file_size == cfg.file_size && candidate.store_id == store_id &&
           candidate.next_entry_sequence <= on9rstore_def::entry_id_sequence_mask &&
           candidate.boot_counter <= on9rstore_def::entry_id_boot_mask && is_metadata_geometry_valid(candidate) &&
           calc_crc32(reinterpret_cast<const uint8_t *>(&crc_state), sizeof(crc_state)) == expected_checksum;
}

bool on9rstore::is_metadata_geometry_valid(const on9rstore_def::metadata &candidate) const
{
    if (candidate.oldest_offset >= data_size || candidate.write_offset >= data_size || candidate.used_size > data_size ||
        (candidate.oldest_offset % on9rstore_def::entry_alignment) != 0 ||
        (candidate.write_offset % on9rstore_def::entry_alignment) != 0 ||
        (candidate.used_size % on9rstore_def::entry_alignment) != 0) {
        return false;
    }

    if (candidate.used_size == 0 || candidate.used_size == data_size) {
        return candidate.oldest_offset == candidate.write_offset;
    }

    const uint64_t expected_used_size = candidate.write_offset >= candidate.oldest_offset
                                            ? candidate.write_offset - candidate.oldest_offset
                                            : data_size - candidate.oldest_offset + candidate.write_offset;
    return candidate.used_size == expected_used_size;
}

esp_err_t on9rstore::commit_metadata_unsafe()
{
    const uint32_t next_slot = (metadata_slot + 1) % on9rstore_def::metadata_slot_count;
    on9rstore_def::metadata next_state = state;
    next_state.magic = on9rstore_def::metadata_magic;
    next_state.revision = on9rstore_def::metadata_revision;
    next_state.size = sizeof(next_state);
    next_state.file_size = cfg.file_size;
    next_state.store_id = store_id;
    next_state.generation += 1;
    next_state.checksum = 0;
    next_state.checksum = calc_crc32(reinterpret_cast<const uint8_t *>(&next_state), sizeof(next_state));

    auto ret = write_exact(on9rstore_def::store_header_region_size + next_slot * on9rstore_def::metadata_slot_size,
                           &next_state, sizeof(next_state));
    if (ret != ESP_OK) {
        return ret;
    }

    ret = sync_file();
    if (ret != ESP_OK) {
        return ret;
    }

    state = next_state;
    metadata_slot = next_slot;
    return ESP_OK;
}

esp_err_t on9rstore::persist_state_unsafe()
{
    auto ret = commit_metadata_unsafe();
    metadata_dirty = ret != ESP_OK;
    return ret;
}

esp_err_t on9rstore::increase_boot_counter()
{
    uint32_t next_boot_counter = state.boot_counter + 1;
    if (next_boot_counter > on9rstore_def::entry_id_boot_mask) {
        next_boot_counter = 1;
    }

    state.boot_counter = next_boot_counter;
    state.next_entry_sequence = 0;
    newest_entry_id = 0;
    metadata_dirty = true;
    return persist_state_unsafe();
}

esp_err_t on9rstore::append_boot_entry_unsafe()
{
    on9rstore_def::boot_event event = {};
    event.reset_reason = static_cast<uint32_t>(esp_reset_reason());

#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    if (cfg.copy_coredump && esp_core_dump_image_check() == ESP_OK) {
        size_t coredump_address = 0;
        size_t coredump_size = 0;
        auto ret = esp_core_dump_image_get(&coredump_address, &coredump_size);
        if (ret == ESP_OK && coredump_size <= (UINT32_MAX - sizeof(event))) {
            const auto *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                                             ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
            if (partition == nullptr || coredump_address < partition->address ||
                coredump_size > (partition->size - (coredump_address - partition->address))) {
                ESP_LOGW(TAG, "Boot: coredump image is outside the coredump partition");
            } else {
                uint8_t chunk[256] = {};
                uint32_t coredump_crc = UINT32_MAX;
                size_t remaining = coredump_size;
                size_t coredump_offset = coredump_address - partition->address;
                while (remaining > 0) {
                    const size_t chunk_len = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
                    ret = esp_partition_read(partition, coredump_offset, chunk, chunk_len);
                    if (ret != ESP_OK) {
                        break;
                    }

                    coredump_crc = calc_crc32_update(coredump_crc, chunk, chunk_len);
                    coredump_offset += chunk_len;
                    remaining -= chunk_len;
                }

                coredump_crc = ~coredump_crc;
                if (ret == ESP_OK && state.coredump_size == coredump_size && state.coredump_crc32 == coredump_crc) {
                    return append_entry_unsafe(on9rstore_def::ENTRY_BOOT_EVENT,
                                                reinterpret_cast<const uint8_t *>(&event), sizeof(event), nullptr, true);
                }

                if (ret == ESP_OK) {
                    event.coredump_len = static_cast<uint32_t>(coredump_size);
                    const uint32_t payload_len = sizeof(event) + static_cast<uint32_t>(coredump_size);
                    on9rstore_def::entry_header header = {};
                    uint64_t entry_offset = 0;
                    uint64_t entry_size = 0;
                    ret = reserve_direct_entry_unsafe(on9rstore_def::ENTRY_BOOT_EVENT, payload_len, &header,
                                                       &entry_offset, &entry_size);
                    if (ret == ESP_OK) {
                        ret = write_exact(data_file_offset(entry_offset), &header, sizeof(header));
                    }

                    uint32_t entry_crc = calc_crc32_update(UINT32_MAX,
                                                             reinterpret_cast<const uint8_t *>(&header), sizeof(header));
                    if (ret == ESP_OK) {
                        ret = write_exact(data_file_offset(entry_offset + sizeof(header)), &event, sizeof(event));
                        entry_crc = calc_crc32_update(entry_crc, reinterpret_cast<const uint8_t *>(&event), sizeof(event));
                    }

                    remaining = coredump_size;
                    coredump_offset = coredump_address - partition->address;
                    uint64_t file_payload_offset = entry_offset + sizeof(header) + sizeof(event);
                    while (ret == ESP_OK && remaining > 0) {
                        const size_t chunk_len = remaining < sizeof(chunk) ? remaining : sizeof(chunk);
                        ret = esp_partition_read(partition, coredump_offset, chunk, chunk_len);
                        if (ret == ESP_OK) {
                            ret = write_exact(data_file_offset(file_payload_offset), chunk, chunk_len);
                        }

                        if (ret == ESP_OK) {
                            entry_crc = calc_crc32_update(entry_crc, chunk, chunk_len);
                            coredump_offset += chunk_len;
                            file_payload_offset += chunk_len;
                            remaining -= chunk_len;
                        }
                    }

                    if (ret == ESP_OK) {
                        ret = write_entry_trailer_unsafe(entry_offset, payload_len, ~entry_crc, entry_size);
                    }

                    if (ret == ESP_OK) {
                        state.coredump_crc32 = coredump_crc;
                        state.coredump_size = static_cast<uint32_t>(coredump_size);
                        ret = finish_direct_entry_unsafe(entry_offset, entry_size);
                    }

                    if (ret == ESP_OK) {
                        return ESP_OK;
                    }

                    ESP_LOGW(TAG, "Boot: failed to append coredump: ret=0x%x", ret);
                } else {
                    ESP_LOGW(TAG, "Boot: failed to read coredump: ret=0x%x", ret);
                }
            }
        } else if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Boot: failed to find coredump: ret=0x%x", ret);
        } else {
            ESP_LOGW(TAG, "Boot: coredump too large: %u", static_cast<unsigned>(coredump_size));
        }
    }
#endif

    return append_entry_unsafe(on9rstore_def::ENTRY_BOOT_EVENT,
                                reinterpret_cast<const uint8_t *>(&event), sizeof(event), nullptr, true);
}

esp_err_t on9rstore::append_entry(uint16_t type, const uint8_t *payload, size_t payload_len,
                                   on9rstore_def::entry_header *entry_info_out,
                                   uint32_t timeout_ticks, bool force_flush)
{
    // The 0xfff0-0xffff range is reserved for the store itself. Internal writers
    // (boot event, time sync, padding) use append_entry_unsafe() directly.
    if ((payload == nullptr && payload_len > 0) || payload_len > UINT32_MAX ||
        type >= on9rstore_def::ENTRY_RESERVED_START) {
        return ESP_ERR_INVALID_ARG;
    }

    auto ret = acquire_operation_lock(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = append_entry_unsafe(type, payload, static_cast<uint32_t>(payload_len), entry_info_out, force_flush);
    release_operation_lock();
    return ret;
}

esp_err_t on9rstore::append_entry_unsafe(uint16_t type, const uint8_t *payload, uint32_t payload_len,
                                          on9rstore_def::entry_header *entry_info_out, bool force_flush)
{
    if (metadata_dirty) {
        auto ret = flush_unsafe();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    uint64_t entry_size = get_entry_size(payload_len);
    if (entry_size == 0 || entry_size > data_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    const bool write_directly = entry_size > cfg.write_buffer_size;
    esp_err_t ret = ESP_OK;
    if (write_directly || (write_buf_pos + static_cast<size_t>(entry_size)) > cfg.write_buffer_size) {
        ret = flush_unsafe();
    }

    if (ret != ESP_OK) {
        return ret;
    }

    if (write_directly) {
        on9rstore_def::entry_header header = {};
        uint64_t entry_offset = 0;
        ret = reserve_direct_entry_unsafe(type, payload_len, &header, &entry_offset, &entry_size);
        if (ret == ESP_OK) {
            ret = write_exact(data_file_offset(entry_offset), &header, sizeof(header));
        }

        uint32_t crc = calc_crc32_update(UINT32_MAX, reinterpret_cast<const uint8_t *>(&header), sizeof(header));
        if (ret == ESP_OK && payload_len > 0) {
            ret = write_exact(data_file_offset(entry_offset + sizeof(header)), payload, payload_len);
            crc = calc_crc32_update(crc, payload, payload_len);
        }

        if (ret == ESP_OK) {
            ret = write_entry_trailer_unsafe(entry_offset, payload_len, ~crc, entry_size);
        }

        if (ret == ESP_OK) {
            ret = finish_direct_entry_unsafe(entry_offset, entry_size);
        }

        if (ret == ESP_OK && entry_info_out != nullptr) {
            *entry_info_out = header;
        }

        return ret;
    }

    ret = prepare_entry_space_unsafe(entry_size);
    if (ret != ESP_OK) {
        return ret;
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

    if (write_buf_pos == 0) {
        write_buf_offset = state.write_offset;
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
    state.write_offset += entry_size;
    if (state.write_offset == data_size) {
        state.write_offset = 0;
    }
    state.used_size += entry_size;
    if (state.used_size == entry_size) {
        state.oldest_offset = write_buf_offset;
    }

    if (force_flush) {
        ret = flush_unsafe();
    }

    if (ret == ESP_OK && entry_info_out != nullptr) {
        *entry_info_out = header;
    }

    return ret;
}

esp_err_t on9rstore::append_time_sync(on9rstore_def::time_sync_type sync_type, uint64_t ts_millisec,
                                      uint32_t timeout_ticks, bool force_flush)
{
    on9rstore_def::time_sync event = {};
    event.type = sync_type;
    event.ts_millisec = ts_millisec;

    // ENTRY_TIME_SYNCED is a reserved type, so this bypasses the public
    // append_entry() type check and takes the operation lock itself.
    auto ret = acquire_operation_lock(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = append_entry_unsafe(on9rstore_def::ENTRY_TIME_SYNCED, reinterpret_cast<const uint8_t *>(&event),
                               sizeof(event), nullptr, force_flush);
    release_operation_lock();
    return ret;
}

esp_err_t on9rstore::flush_write(uint32_t timeout_ticks)
{
    auto ret = acquire_operation_lock(timeout_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = flush_unsafe();
    release_operation_lock();
    return ret;
}

esp_err_t on9rstore::flush_unsafe()
{
    if (write_buf_pos == 0) {
        return metadata_dirty ? persist_state_unsafe() : ESP_OK;
    }

    auto ret = write_exact(data_file_offset(write_buf_offset), write_buf.get(), write_buf_pos);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = sync_file();
    if (ret != ESP_OK) {
        return ret;
    }

    metadata_dirty = true;
    ret = persist_state_unsafe();
    if (ret != ESP_OK) {
        return ret;
    }

    write_buf_pos = 0;
    write_buf_offset = state.write_offset;
    return ESP_OK;
}

esp_err_t on9rstore::prepare_entry_space_unsafe(uint64_t entry_size)
{
    const uint64_t tail_size = data_size - state.write_offset;
    if (entry_size > tail_size) {
        auto ret = flush_unsafe();
        if (ret != ESP_OK) {
            return ret;
        }

        const uint64_t required_size = tail_size + entry_size;
        ret = discard_until_free_unsafe(required_size > data_size ? data_size : required_size);
        if (ret != ESP_OK) {
            return ret;
        }

        return wrap_write_position_unsafe(tail_size);
    }

    if ((data_size - state.used_size) < entry_size) {
        auto ret = flush_unsafe();
        if (ret != ESP_OK) {
            return ret;
        }

        ret = discard_until_free_unsafe(entry_size);
        if (ret != ESP_OK) {
            return ret;
        }

        // Retire the old head before reusing its physical bytes. A reset after
        // this point may lose capacity/history, but the last committed metadata
        // can no longer reference bytes that the next write will overwrite.
        return metadata_dirty ? persist_state_unsafe() : ESP_OK;
    }

    return ESP_OK;
}

esp_err_t on9rstore::discard_until_free_unsafe(uint64_t required_size)
{
    if (required_size > data_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    while ((data_size - state.used_size) < required_size) {
        auto ret = discard_oldest_entry_unsafe();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t on9rstore::discard_oldest_entry_unsafe()
{
    if (state.used_size == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    const uint64_t remaining = data_size - state.oldest_offset;
    uint64_t entry_size = 0;
    if (remaining < on9rstore_def::min_entry_size) {
        // A short physical tail is a logical padding span. It is only possible
        // after a wrap and contains no complete entry header.
        entry_size = remaining;
    } else {
        on9rstore_def::entry_header header = {};
        auto ret = read_entry_header_unsafe(state.oldest_offset, &header);
        if (ret != ESP_OK) {
            return ret;
        }

        if (!is_entry_header_valid(header)) {
            // A torn header has no trustworthy length. Advance only to positive
            // evidence of the next complete v3 entry inside the metadata-known
            // live range; never let one damaged header wedge all future appends.
            return resynchronise_oldest_unsafe();
        }

        entry_size = get_entry_size(header.len);
        if (entry_size == 0 || entry_size > remaining || entry_size > state.used_size) {
            return ESP_ERR_INVALID_STATE;
        }

        ret = validate_entry_unsafe(state.oldest_offset, header, entry_size);
        if (ret == ESP_ERR_INVALID_CRC) {
            // The sane header length is enough to retire a damaged oldest entry.
            // This matches boot recovery: CRC-bad entries are not trusted, but
            // they must not permanently prevent the ring from making progress.
            ESP_LOGW(TAG, "Discard: skipping CRC-bad entry at %" PRIu64, state.oldest_offset);
        } else if (ret != ESP_OK) {
            return ret;
        }
    }

    if (entry_size == 0 || entry_size > state.used_size) {
        return ESP_ERR_INVALID_STATE;
    }

    state.oldest_offset += entry_size;
    if (state.oldest_offset == data_size) {
        state.oldest_offset = 0;
    }
    state.used_size -= entry_size;
    if (state.used_size == 0) {
        state.oldest_offset = state.write_offset;
    }
    metadata_dirty = true;
    return ESP_OK;
}

esp_err_t on9rstore::resynchronise_oldest_unsafe()
{
    uint64_t skipped_size = 0;
    uint64_t probe_offset = state.oldest_offset;

    while (skipped_size < state.used_size) {
        const uint64_t remaining = data_size - probe_offset;
        if (remaining < on9rstore_def::min_entry_size) {
            if (remaining > (state.used_size - skipped_size)) {
                break;
            }

            probe_offset = 0;
            skipped_size += remaining;
            continue;
        }

        if (skipped_size != 0) {
            on9rstore_def::entry_header header = {};
            auto ret = read_entry_header_unsafe(probe_offset, &header);
            if (ret != ESP_OK) {
                return ret;
            }

            const uint64_t entry_size = get_entry_size(header.len);
            if (is_entry_header_valid(header) && entry_size != 0 && entry_size <= remaining &&
                entry_size <= (state.used_size - skipped_size)) {
                ret = validate_entry_unsafe(probe_offset, header, entry_size);
                if (ret == ESP_OK) {
                    ESP_LOGW(TAG, "Discard: skipped %" PRIu64 " bytes of torn data", skipped_size);
                    state.oldest_offset = probe_offset;
                    state.used_size -= skipped_size;
                    metadata_dirty = true;
                    return ESP_OK;
                }

                if (ret != ESP_ERR_INVALID_CRC) {
                    return ret;
                }
            }
        }

        probe_offset += on9rstore_def::entry_alignment;
        skipped_size += on9rstore_def::entry_alignment;
        if (probe_offset == data_size) {
            probe_offset = 0;
        }
    }

    // Metadata identified this as an on9rstore v3 live range, but no complete
    // entry survived in it. Retire only that known-corrupt range so logging can
    // continue; this path never treats an unknown existing file as fresh.
    ESP_LOGE(TAG, "Discard: no complete entry after torn data; retiring current live range");
    state.used_size = 0;
    state.oldest_offset = state.write_offset;
    metadata_dirty = true;
    return ESP_OK;
}

esp_err_t on9rstore::wrap_write_position_unsafe(uint64_t tail_size)
{
    if (tail_size == 0) {
        state.write_offset = 0;
        if (state.used_size == 0) {
            state.oldest_offset = 0;
        }
        metadata_dirty = true;
        return persist_state_unsafe();
    }

    if (state.used_size == 0) {
        state.write_offset = 0;
        state.oldest_offset = 0;
        metadata_dirty = true;
        return persist_state_unsafe();
    }

    if (tail_size >= on9rstore_def::min_entry_size) {
        on9rstore_def::entry_header header = {};
        header.magic = on9rstore_def::entry_magic;
        header.revision = on9rstore_def::entry_revision;
        header.type = on9rstore_def::ENTRY_PADDING;
        header.entry_id = newest_entry_id;
        header.len = static_cast<uint32_t>(tail_size - sizeof(header) - on9rstore_def::entry_crc_len);
        header.store_id = store_id;

        auto ret = write_exact(data_file_offset(state.write_offset), &header, sizeof(header));
        if (ret != ESP_OK) {
            return ret;
        }

        uint8_t zeroes[256] = {};
        uint64_t zeroes_remaining = header.len;
        uint64_t payload_offset = state.write_offset + sizeof(header);
        uint32_t crc = calc_crc32_update(UINT32_MAX, reinterpret_cast<const uint8_t *>(&header), sizeof(header));
        while (zeroes_remaining > 0) {
            const size_t chunk_len = zeroes_remaining < sizeof(zeroes) ? static_cast<size_t>(zeroes_remaining) : sizeof(zeroes);
            ret = write_exact(data_file_offset(payload_offset), zeroes, chunk_len);
            if (ret != ESP_OK) {
                return ret;
            }

            crc = calc_crc32_update(crc, zeroes, chunk_len);
            payload_offset += chunk_len;
            zeroes_remaining -= chunk_len;
        }

        ret = write_entry_trailer_unsafe(state.write_offset, header.len, ~crc, tail_size);
        if (ret != ESP_OK) {
            return ret;
        }

        ret = sync_file();
        if (ret != ESP_OK) {
            return ret;
        }
    }

    state.used_size += tail_size;
    state.write_offset = 0;
    metadata_dirty = true;
    return persist_state_unsafe();
}

esp_err_t on9rstore::reserve_direct_entry_unsafe(uint16_t type, uint32_t payload_len,
                                                   on9rstore_def::entry_header *header_out,
                                                   uint64_t *entry_offset_out, uint64_t *entry_size_out)
{
    if (header_out == nullptr || entry_offset_out == nullptr || entry_size_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint64_t entry_size = get_entry_size(payload_len);
    if (entry_size == 0 || entry_size > data_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    auto ret = prepare_entry_space_unsafe(entry_size);
    if (ret != ESP_OK) {
        return ret;
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

    *header_out = header;
    *entry_offset_out = state.write_offset;
    *entry_size_out = entry_size;
    return ESP_OK;
}

esp_err_t on9rstore::write_entry_trailer_unsafe(uint64_t entry_offset, uint32_t payload_len, uint32_t crc,
                                                  uint64_t entry_size)
{
    const uint64_t crc_offset = entry_offset + sizeof(on9rstore_def::entry_header) + payload_len;
    auto ret = write_exact(data_file_offset(crc_offset), &crc, sizeof(crc));
    if (ret != ESP_OK) {
        return ret;
    }

    const uint64_t written_size = sizeof(on9rstore_def::entry_header) + static_cast<uint64_t>(payload_len) + sizeof(crc);
    if (entry_size < written_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    return write_zeroes_unsafe(data_file_offset(entry_offset + written_size), entry_size - written_size);
}

esp_err_t on9rstore::finish_direct_entry_unsafe(uint64_t entry_offset, uint64_t entry_size)
{
    auto ret = sync_file();
    if (ret != ESP_OK) {
        return ret;
    }

    state.write_offset = entry_offset + entry_size;
    if (state.write_offset == data_size) {
        state.write_offset = 0;
    }
    state.used_size += entry_size;
    if (state.used_size == entry_size) {
        state.oldest_offset = entry_offset;
    }
    metadata_dirty = true;
    return persist_state_unsafe();
}

esp_err_t on9rstore::write_zeroes_unsafe(uint64_t offset, uint64_t len)
{
    uint8_t zeroes[4] = {};
    while (len > 0) {
        const size_t chunk_len = len < sizeof(zeroes) ? static_cast<size_t>(len) : sizeof(zeroes);
        auto ret = write_exact(offset, zeroes, chunk_len);
        if (ret != ESP_OK) {
            return ret;
        }

        offset += chunk_len;
        len -= chunk_len;
    }

    return ESP_OK;
}

esp_err_t on9rstore::read_entry_header_unsafe(uint64_t offset, on9rstore_def::entry_header *header_out)
{
    if (header_out == nullptr || offset >= data_size ||
        (data_size - offset) < sizeof(on9rstore_def::entry_header)) {
        return ESP_ERR_INVALID_ARG;
    }

    return read_exact(data_file_offset(offset), header_out, sizeof(*header_out));
}

esp_err_t on9rstore::validate_entry_unsafe(uint64_t offset, const on9rstore_def::entry_header &header,
                                             uint64_t entry_size)
{
    if (!is_entry_header_valid(header) || entry_size < on9rstore_def::min_entry_size ||
        header.len > (entry_size - sizeof(header) - on9rstore_def::entry_crc_len)) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t crc = calc_crc32_update(UINT32_MAX, reinterpret_cast<const uint8_t *>(&header), sizeof(header));
    uint8_t buf[256] = {};
    uint32_t payload_remaining = header.len;
    uint64_t payload_offset = offset + sizeof(header);
    while (payload_remaining > 0) {
        const size_t chunk_size = payload_remaining < sizeof(buf) ? payload_remaining : sizeof(buf);
        auto ret = read_exact(data_file_offset(payload_offset), buf, chunk_size);
        if (ret != ESP_OK) {
            return ret;
        }

        crc = calc_crc32_update(crc, buf, chunk_size);
        payload_offset += chunk_size;
        payload_remaining -= chunk_size;
    }

    uint32_t expected_crc = 0;
    auto ret = read_exact(data_file_offset(offset + sizeof(header) + header.len), &expected_crc, sizeof(expected_crc));
    if (ret != ESP_OK) {
        return ret;
    }

    return ~crc == expected_crc ? ESP_OK : ESP_ERR_INVALID_CRC;
}

bool on9rstore::is_entry_header_valid(const on9rstore_def::entry_header &header) const
{
    return store_id != 0 && header.magic == on9rstore_def::entry_magic &&
           header.revision == on9rstore_def::entry_revision &&
           header.store_id == store_id;
}

esp_err_t on9rstore::read_exact(uint64_t offset, void *buf_out, size_t len) const
{
    if (fd < 0 || buf_out == nullptr || len == 0 || len > cfg.file_size || offset > (cfg.file_size - len)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (lseek(fd, static_cast<off_t>(offset), SEEK_SET) < 0) {
        ESP_LOGE(TAG, "Read: lseek failed: errno=%d", errno);
        return ESP_FAIL;
    }

    uint8_t *out = static_cast<uint8_t *>(buf_out);
    size_t remaining = len;
    while (remaining > 0) {
        const ssize_t result = read(fd, out, remaining);
        if (result <= 0) {
            ESP_LOGE(TAG, "Read: failed: errno=%d", errno);
            return ESP_FAIL;
        }

        out += result;
        remaining -= result;
    }

    return ESP_OK;
}

esp_err_t on9rstore::write_exact(uint64_t offset, const void *buf, size_t len) const
{
    if (fd < 0 || buf == nullptr || len == 0 || len > cfg.file_size || offset > (cfg.file_size - len)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (lseek(fd, static_cast<off_t>(offset), SEEK_SET) < 0) {
        ESP_LOGE(TAG, "Write: lseek failed: errno=%d", errno);
        return ESP_FAIL;
    }

    const uint8_t *in = static_cast<const uint8_t *>(buf);
    size_t remaining = len;
    while (remaining > 0) {
        const ssize_t result = write(fd, in, remaining);
        if (result <= 0) {
            ESP_LOGE(TAG, "Write: failed: errno=%d", errno);
            return ESP_FAIL;
        }

        in += result;
        remaining -= result;
    }

    return ESP_OK;
}

esp_err_t on9rstore::sync_file() const
{
    if (fd < 0 || fsync(fd) != 0) {
        ESP_LOGE(TAG, "Sync: fsync failed: errno=%d", errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

uint32_t on9rstore::calc_crc32(const uint8_t *buf, size_t len)
{
    return ~calc_crc32_update(UINT32_MAX, buf, len);
}

uint32_t on9rstore::calc_crc32_update(uint32_t crc, const uint8_t *buf, size_t len)
{
    for (size_t idx = 0; idx < len; idx += 1) {
        crc ^= buf[idx];
        for (uint8_t bit = 0; bit < 8; bit += 1) {
            crc = (crc & 1) ? ((crc >> 1) ^ 0xedb88320UL) : (crc >> 1);
        }
    }

    return crc;
}

uint64_t on9rstore::get_entry_size(uint32_t payload_len)
{
    const uint64_t base_size = sizeof(on9rstore_def::entry_header) + static_cast<uint64_t>(payload_len) +
                               on9rstore_def::entry_crc_len;
    if (base_size > (SIZE_MAX - (on9rstore_def::entry_alignment - 1))) {
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
    if (state.next_entry_sequence == on9rstore_def::entry_id_sequence_mask) {
        return 0;
    }

    state.next_entry_sequence += 1;
    newest_entry_id = (static_cast<uint64_t>(state.boot_counter) << 40ULL) | state.next_entry_sequence;
    return newest_entry_id;
}

uint64_t on9rstore::data_file_offset(uint64_t data_offset) const
{
    return on9rstore_def::control_region_size + data_offset;
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

void on9rstore::close_storage_unsafe()
{
    write_buf.reset();
    write_buf_pos = 0;
    write_buf_offset = 0;
    metadata_dirty = false;
    data_size = 0;
    store_id = 0;
    staging_file_active = false;
    staging_file_path[0] = '\0';

    if (fd >= 0) {
        (void)close(fd);
        fd = -1;
    }
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

    esp_err_t ret = ESP_OK;
    if (!force) {
        ret = flush_unsafe();
    }

    if (ret == ESP_OK || force) {
        initialized = false;
        close_storage_unsafe();
    }

    xSemaphoreGive(write_lock);
    shutting_down = false;
    xSemaphoreGive(lifecycle_lock);
    return ret;
}

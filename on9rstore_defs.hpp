#pragma once

#include <cstddef>
#include <cstdint>

#define ON9RSTORE_PACKED __attribute__((packed, aligned(1)))

namespace on9rstore_def
{
    struct ON9RSTORE_PACKED entry_header {
        uint32_t magic;
        uint16_t revision;
        uint16_t type;
        uint64_t entry_id;
        uint64_t uptime_us;
        uint32_t len;
        uint16_t store_id;
    };

    static_assert(sizeof(entry_header) == 30);
    static_assert(offsetof(entry_header, entry_id) == 8);

    enum reserved_entry_type : uint16_t {
        ENTRY_BOOT_EVENT = 0xffff,
        ENTRY_TIME_SYNCED = 0xfffe,
        ENTRY_PADDING = 0xfffd,
        ENTRY_CORRUPTED = 0xfffc,
        ENTRY_RESERVED_START = 0xfff0,
    };

    struct ON9RSTORE_PACKED boot_event {
        uint32_t reset_reason;
        uint32_t coredump_len;
    };

    enum time_sync_type : uint8_t {
        TIME_SYNC_GPS = 0,
        TIME_SYNC_NTP = 1,
        TIME_SYNC_RTC = 2,
        TIME_SYNC_MANUAL = 3,
    };

    struct ON9RSTORE_PACKED time_sync {
        uint8_t type;
        uint64_t ts_millisec;
    };

    // All on-disk multi-byte fields are little-endian ESP target values. An entry
    // is committed only when its trailing CRC32 validates.
    static const constexpr uint32_t entry_magic = 0x39525352; // "RSR9"
    static const constexpr uint16_t entry_revision = 3;
    static const constexpr uint32_t store_header_magic = 0x39524853; // "SHR9"
    static const constexpr uint16_t store_header_revision = 3;
    static const constexpr uint16_t store_header_state_ready = 0x3953; // "S9"
    static const constexpr uint32_t metadata_magic = 0x3952534f; // "OSR9"
    static const constexpr uint16_t metadata_revision = 3;
    static const constexpr size_t store_header_slot_size = 4096;
    static const constexpr size_t store_header_slot_count = 2;
    static const constexpr size_t store_header_region_size = store_header_slot_size * store_header_slot_count;
    static const constexpr size_t metadata_slot_size = 4096;
    static const constexpr size_t metadata_slot_count = 2;
    static const constexpr size_t metadata_region_size = metadata_slot_size * metadata_slot_count;
    static const constexpr size_t control_region_size = store_header_region_size + metadata_region_size;
    static const constexpr size_t entry_alignment = 4;
    static const constexpr size_t entry_crc_len = sizeof(uint32_t);
    static const constexpr uint64_t entry_id_boot_mask = 0xffffffULL;
    static const constexpr uint64_t entry_id_sequence_mask = 0xffffffffffULL;

    struct ON9RSTORE_PACKED store_header {
        uint32_t magic;
        uint16_t revision;
        uint16_t size;
        uint64_t file_size;
        uint16_t store_id;
        uint16_t state;
        uint32_t checksum;
    };

    static_assert(sizeof(store_header) == 24);

    struct ON9RSTORE_PACKED metadata {
        uint32_t magic;
        uint16_t revision;
        uint16_t size;
        uint64_t generation;
        uint64_t file_size;
        uint64_t oldest_offset;
        uint64_t write_offset;
        uint64_t used_size;
        uint64_t next_entry_sequence;
        uint32_t boot_counter;
        uint32_t coredump_crc32;
        uint32_t coredump_size;
        uint16_t store_id;
        uint32_t checksum;
    };

    static constexpr size_t align_up(size_t val, size_t align)
    {
        return (val + (align - 1)) & ~(align - 1);
    }

    static constexpr size_t min_entry_size = align_up(sizeof(entry_header) + entry_crc_len, entry_alignment);
}

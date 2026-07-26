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
        uint16_t segment_slot;
        uint64_t segment_generation;
    };

    static_assert(sizeof(entry_header) == 40);
    static_assert(offsetof(entry_header, entry_id) == 8);

    enum reserved_entry_type : uint16_t {
        ENTRY_BOOT_EVENT = 0xffff,
        ENTRY_CORRUPTED = 0xfffc,
        ENTRY_RESERVED_START = 0xfff0,
    };

    struct ON9RSTORE_PACKED boot_event {
        uint32_t reset_reason;
        uint32_t coredump_len;
    };

    enum time_source_mask : uint32_t {
        TIME_SOURCE_GPS = 1UL << 0,
        TIME_SOURCE_NTP = 1UL << 1,
        TIME_SOURCE_CELLULAR = 1UL << 2,
        TIME_SOURCE_RTC = 1UL << 3,
        TIME_SOURCE_MANUAL = 1UL << 4,
        TIME_SOURCE_SERVER_ESTIMATE = 1UL << 5,
        TIME_SOURCE_ALL = TIME_SOURCE_GPS | TIME_SOURCE_NTP | TIME_SOURCE_CELLULAR |
                          TIME_SOURCE_RTC | TIME_SOURCE_MANUAL | TIME_SOURCE_SERVER_ESTIMATE,
    };

    enum time_anchor_quality : uint8_t {
        TIME_ANCHOR_QUALITY_PROVISIONAL = 0,
        TIME_ANCHOR_QUALITY_CONFIRMED = 1,
    };

    enum time_anchor_flags : uint16_t {
        TIME_ANCHOR_FLAG_CONSENSUS = 1U << 0,
        TIME_ANCHOR_FLAG_PPS = 1U << 1,
        TIME_ANCHOR_FLAG_CONTINUITY = 1U << 2,
    };

    struct time_anchor {
        uint32_t source_mask;
        uint8_t source_count;
        uint8_t quality;
        uint16_t flags;
        uint64_t monotonic_us;
        uint64_t utc_us;
        uint64_t uncertainty_us;
        uint64_t replace_sequence;
    };

    struct ON9RSTORE_PACKED time_anchor_entry {
        uint32_t magic;
        uint16_t revision;
        uint16_t size;
        uint16_t store_id;
        uint8_t source_count;
        uint8_t quality;
        uint16_t flags;
        uint16_t reserved;
        uint32_t source_mask;
        uint32_t boot_counter;
        uint64_t sequence;
        uint64_t monotonic_us;
        uint64_t utc_us;
        uint64_t uncertainty_us;
        uint64_t max_durable_entry_id;
        uint64_t replace_sequence;
        uint32_t checksum;
    };

    static_assert(sizeof(time_anchor_entry) == 76);

    struct ON9RSTORE_PACKED manifest_superblock {
        uint32_t magic;
        uint16_t revision;
        uint16_t size;
        uint64_t generation;
        uint16_t store_id;
        uint16_t state;
        uint32_t segment_count;
        uint64_t segment_size;
        uint32_t time_anchor_count;
        uint32_t time_anchor_slot_size;
        uint32_t sparse_index_stride;
        uint32_t active_slot;
        uint64_t active_segment_generation;
        uint64_t next_segment_generation;
        uint64_t oldest_segment_generation;
        uint32_t boot_counter;
        uint32_t reserved;
        uint64_t next_entry_sequence;
        uint64_t newest_entry_id;
        uint64_t used_size;
        uint32_t time_anchor_write_index;
        uint32_t time_anchor_used;
        uint64_t next_time_anchor_sequence;
        uint32_t coredump_crc32;
        uint32_t coredump_size;
        uint32_t checksum;
    };

    static_assert(sizeof(manifest_superblock) == 132);

    struct ON9RSTORE_PACKED segment_header {
        uint32_t magic;
        uint16_t revision;
        uint16_t size;
        uint16_t store_id;
        uint16_t state;
        uint32_t slot;
        uint64_t generation;
        uint64_t segment_size;
        uint64_t data_start;
        uint64_t data_end;
        uint64_t index_start;
        uint32_t index_capacity;
        uint32_t index_stride;
        uint32_t checksum;
    };

    static_assert(sizeof(segment_header) == 68);

    struct ON9RSTORE_PACKED sparse_index_entry {
        uint64_t entry_id;
        uint64_t uptime_us;
        uint32_t offset;
        uint16_t type;
        uint16_t reserved;
    };

    static_assert(sizeof(sparse_index_entry) == 24);

    struct ON9RSTORE_PACKED segment_footer {
        uint32_t magic;
        uint16_t revision;
        uint16_t size;
        uint16_t store_id;
        uint16_t state;
        uint32_t slot;
        uint64_t generation;
        uint64_t first_entry_id;
        uint64_t last_entry_id;
        uint64_t entry_count;
        uint64_t data_end;
        uint32_t index_count;
        uint32_t index_stride;
        uint32_t index_checksum;
        uint32_t checksum;
    };

    static_assert(sizeof(segment_footer) == 72);

    static const constexpr uint32_t entry_magic = 0x39525352; // "RSR9"
    static const constexpr uint16_t entry_revision = 4;
    static const constexpr uint32_t manifest_magic = 0x39534d52; // "RMS9"
    static const constexpr uint16_t manifest_revision = 4;
    static const constexpr uint16_t manifest_state_provisioning_unverified =
        0x7001;
    static const constexpr uint16_t manifest_state_ready = 0x7002;
    static const constexpr uint16_t manifest_state_provisioning_owned =
        0x7003;
    static const constexpr uint32_t time_anchor_magic = 0x39415452; // "RTA9"
    static const constexpr uint16_t time_anchor_revision = 1;
    static const constexpr uint32_t segment_header_magic = 0x39485352; // "RSH9"
    static const constexpr uint16_t segment_header_revision = 1;
    static const constexpr uint16_t segment_state_empty = 0x7100;
    static const constexpr uint16_t segment_state_active = 0x7101;
    static const constexpr uint32_t segment_footer_magic = 0x39465352; // "RSF9"
    static const constexpr uint16_t segment_footer_revision = 1;
    static const constexpr uint16_t segment_footer_state_sealed = 0x7201;

    static const constexpr size_t manifest_superblock_slot_size = 4096;
    static const constexpr size_t manifest_superblock_slot_count = 2;
    static const constexpr size_t manifest_superblock_region_size =
        manifest_superblock_slot_size * manifest_superblock_slot_count;
    static const constexpr size_t time_anchor_slot_size = 512;
    static const constexpr size_t segment_header_slot_size = 4096;
    static const constexpr size_t segment_header_slot_count = 2;
    static const constexpr size_t segment_header_region_size =
        segment_header_slot_size * segment_header_slot_count;
    static const constexpr size_t segment_footer_slot_size = 4096;
    static const constexpr size_t segment_footer_slot_count = 2;
    static const constexpr size_t segment_footer_region_size =
        segment_footer_slot_size * segment_footer_slot_count;
    static const constexpr size_t sparse_index_stride = 64;
    static const constexpr size_t entry_alignment = 4;
    static const constexpr size_t entry_crc_len = sizeof(uint32_t);
    static const constexpr uint64_t entry_id_boot_mask = 0xffffffULL;
    static const constexpr uint64_t entry_id_sequence_mask = 0xffffffffffULL;

    static constexpr size_t align_up(size_t val, size_t align)
    {
        return (val + (align - 1)) & ~(align - 1);
    }

    static constexpr size_t min_entry_size =
        align_up(sizeof(entry_header) + entry_crc_len, entry_alignment);
}

# on9rstore

`on9rstore` is a fixed-capacity, segmented FAT store for append-oriented
device data. The constructor path is a mounted FAT base path:

```c++
on9rstore_cfg cfg = {
    .write_buffer_size = 8192,
    .copy_coredump = true,
};

on9rstore store("/data", &cfg);
ESP_ERROR_CHECK(store.init());
```

For a new store, `init()` creates this permanent layout:

```text
/data/
├── manifest.db
├── data_0.db
├── data_1.db
├── ...
└── data_N.db
```

Every file is allocated at its final size by
`esp_vfs_fat_create_contiguous_file(..., true)` and checked with
`esp_vfs_fat_test_contiguous_file()`. Healthy files are never grown,
truncated, renamed, or deleted during rotation. A physical `data_N.db`
filename denotes a reusable slot; the generation stored inside it determines
its logical order.

The creation geometry comes from Kconfig:

```text
CONFIG_ON9RSTORE_SPARSE_FILE_SIZE  default 1048576 bytes
CONFIG_ON9STORE_SPARSE_FILE_CNT    default 5
CONFIG_ON9RSTORE_TIME_ANCHOR_CNT   default 512
```

An existing `manifest.db` is authoritative. Its segment size, segment count,
and time-anchor count override a later firmware build's Kconfig values, so the
component never resizes or reinterprets an existing store.

Before creating a new manifest, `init()` scans the base directory and refuses
creation if any canonical `data_<number>.db` name already exists. It then
writes and syncs two `provisioning_owned` manifest superblocks before creating
the first data file. This durable state proves that data files appearing
during a resumed provisioning attempt belong to this store, rather than to an
older store whose manifest was lost.

Provisioning is resumable after each file creation and replicated header
write. A reset after slot 0 was activated as generation 1 but before the ready
manifest checkpoint preserves that active generation instead of resetting it.
Legacy unverified provisioning manifests fail closed. Ready stores are
unchanged.

## Descriptors and concurrency

The component uses three distinct descriptors:

- `manifest_fd` for manifest checkpoints and time-anchor slots;
- `writer_fd` for the current active data segment;
- `reader_fd` for recovery and the future query path.

This prevents a reader seek from changing the writer's position and keeps
manifest traffic separate from segment traffic. Lifecycle and writes have
dedicated FreeRTOS mutexes. A reader mutex is allocated for the public query
path; initial recovery is performed exclusively before `init()` publishes the
store.

## Entries and rotation

Data files contain sorted append runs. Entries in one generation are written
sequentially from the data-area start. When the active file is full, it is
flushed and sealed with a sparse index and replicated footer. The next
physical slot is durably retired, assigned a greater generation, and reused
from its beginning.

The revision-4 entry header is:

```text
[magic: u32] [revision: u16] [type: u16]
[entry_id: u64] [uptime_us: u64] [payload_len: u32]
[store_id: u16] [segment_slot: u16] [segment_generation: u64]
```

It is followed by the payload, a CRC32 over header and payload, and zero
padding to a four-byte boundary. Binding every entry to the random non-zero
16-bit store ID and segment generation prevents recycled cluster garbage and
stale data from an older use of the same slot from being recovered as current
history.

Entry IDs are `(boot_counter << 40) | sequence`. The persisted upper 24-bit
boot counter increments during `init()`. The lower 40-bit sequence begins at
one on each boot and advances for each reserved append. Every entry also stores
the 64-bit value returned by `esp_timer_get_time()` in microseconds.

Application entry types must be below `0xfff0`; the upper range is reserved.
`init()` appends a boot event. When configured and available, a new ESP-IDF
flash coredump is streamed into that boot event in bounded chunks.

Small entries are batched until `flush_write()` or `force_flush=true`. Entries
larger than the configured write buffer stream directly and are synced before
returning. `flush_write()` syncs entry data before checkpointing the manifest.

## Segment layout

Each fixed-size data file contains:

```text
├── segment header slot 0: 4096 B
├── segment header slot 1: 4096 B
├── sequential entry area
├── reserved sparse index, one item per 64 entries
├── sealed footer slot 0: 4096 B
└── sealed footer slot 1: 4096 B
```

An open segment is recovered by scanning only its generation-matched,
CRC-valid entry run. A sealed segment normally loads from its footer after the
footer and sparse-index CRC both validate. If the footer remains valid but the
index does not, recovery proves that the entry run reaches the footer's
declared boundary, rebuilds the index, and rewrites the replicated footers
without reopening the segment for append. Valid entries found beyond an older
footer are retained in the repaired seal. Corruption before the declared
boundary fails closed instead of silently truncating sealed history.

Index repair uses the normal index-sync-before-footer ordering. A reset during
repair leaves the same scan-and-repair path available on the next boot. The
manifest is a boot accelerator and durable retirement checkpoint; segment
headers, footers, generations, indexes, and entry CRCs remain independently
validated.

## Manifest and time anchors

`manifest.db` contains two rotating, CRC-protected superblock slots followed by
a fixed ring of 512-byte time-anchor slots. Superblocks persist the store
identity, immutable geometry, boot/entry counters, segment generations,
retention floor, time-anchor ring state, and coredump fingerprint.

Time anchors live only in the manifest, not in data segments:

```c++
on9rstore_def::time_anchor anchor = {
    .source_mask = on9rstore_def::TIME_SOURCE_GPS |
                   on9rstore_def::TIME_SOURCE_NTP,
    .source_count = 2,
    .quality = on9rstore_def::TIME_ANCHOR_QUALITY_CONFIRMED,
    .flags = on9rstore_def::TIME_ANCHOR_FLAG_CONSENSUS,
    .monotonic_us = measurement_uptime_us,
    .utc_us = measurement_utc_us,
    .uncertainty_us = estimated_error_us,
    .replace_sequence = 0,
};
ESP_ERROR_CHECK(store.append_time_anchor(anchor));
```

The caller must validate clock samples and submit an accepted model commit,
including source mask, source count, quality, flags, measurement monotonic
instant, UTC, uncertainty, and optional replacement sequence. `on9rstore`
validates the representation and commit ordering, but it does not decide
whether GPS, NTP, cellular, RTC, or another source is trustworthy.

`replace_sequence == 0` means the anchor does not explicitly replace another
time model. A non-zero value names an earlier time-anchor sequence that this
anchor corrects or replaces; it does not erase the older slot.

Time anchors support deriving wall time for entries created before the clock
became known without rewriting those entries. Entry ID and monotonic uptime
remain the authoritative order.

## Durability limits

Contiguous preallocation improves locality, prevents later fragmentation, and
avoids FAT-chain growth during ordinary appends. It also avoids interpreting
uninitialised allocated clusters by requiring valid store identity,
generation, structure bounds, and CRC before accepting persisted data.

It does not make FAT or SD media transactional. `fsync()` can still update
directory metadata, and a card controller may cache or reorder its internal
flash writes. Power-cut validation therefore requires deterministic
write/sync fault injection and real-media testing; a successful firmware build
alone is not evidence of end-to-end sudden-power-loss safety. A future
`esp_jrnl` integration remains a separate task.

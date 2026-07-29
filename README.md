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

## Revision-4 on-disk byte and bit layout

All persisted structures are packed without implicit padding. Multi-byte
integers are stored in the ESP target's little-endian byte order. Offsets below
are byte offsets from the beginning of the enclosing file or structure.

The store is a directory of one manifest plus a fixed number of reusable
physical segment slots:

```text
<FAT base path>/
│
├── manifest.db             fixed manifest and time-anchor ring
│
├── data_0.db               physical segment slot 0
├── data_1.db               physical segment slot 1
├── ...
└── data_<C-1>.db           physical segment slot C-1

C = manifest.segment_count
  = CONFIG_ON9STORE_SPARSE_FILE_CNT when a new store is created
```

The numeric filename is only a physical slot. Chronological order is:

```text
segment_generation first, then entry_id
```

### `manifest.db`

For `A = manifest.time_anchor_count`:

```text
manifest.db total size = 8192 + A * 512 bytes

file offset

0x000000  +----------------------------------------------------------+
          | superblock slot 0                              4096 bytes |
          |   132-byte manifest_superblock                            |
          |   3964-byte uninterpreted slot tail                       |
0x001000  +----------------------------------------------------------+
          | superblock slot 1                              4096 bytes |
          |   132-byte manifest_superblock                            |
          |   3964-byte uninterpreted slot tail                       |
0x002000  +----------------------------------------------------------+
          | time-anchor slot 0                              512 bytes |
          |   76-byte time_anchor_entry                               |
          |   436-byte uninterpreted slot tail                        |
0x002200  +----------------------------------------------------------+
          | time-anchor slot 1                              512 bytes |
0x002400  +----------------------------------------------------------+
          | ...                                                      |
          +----------------------------------------------------------+
          | time-anchor slot A-1                            512 bytes |
          +----------------------------------------------------------+
          end offset = 0x002000 + A * 512
```

Only the structure at the beginning of each fixed slot is meaningful. Slot
tails are not required to be zero and may contain recycled FAT-cluster bytes.

The two superblock slots rotate by generation:

```text
slot = superblock.generation % 2
```

The newest CRC-valid generation is authoritative.

#### `manifest_superblock` — 132 bytes

```text
byte range    size   field

+0x00..0x03     4    magic = 0x39534d52, bytes "RMS9"
+0x04..0x05     2    revision = 4
+0x06..0x07     2    size = 132
+0x08..0x0f     8    generation
+0x10..0x11     2    store_id
+0x12..0x13     2    state
+0x14..0x17     4    segment_count
+0x18..0x1f     8    segment_size
+0x20..0x23     4    time_anchor_count
+0x24..0x27     4    time_anchor_slot_size = 512
+0x28..0x2b     4    sparse_index_stride = 64
+0x2c..0x2f     4    active_slot
+0x30..0x37     8    active_segment_generation
+0x38..0x3f     8    next_segment_generation
+0x40..0x47     8    oldest_segment_generation
+0x48..0x4b     4    boot_counter; only low 24 bits are valid
+0x4c..0x4f     4    reserved
+0x50..0x57     8    next_entry_sequence; only low 40 bits are valid
+0x58..0x5f     8    newest_entry_id
+0x60..0x67     8    used_size
+0x68..0x6b     4    time_anchor_write_index
+0x6c..0x6f     4    time_anchor_used
+0x70..0x77     8    next_time_anchor_sequence
+0x78..0x7b     4    coredump_crc32
+0x7c..0x7f     4    coredump_size
+0x80..0x83     4    checksum
```

Manifest state values:

```text
0x7001  provisioning_unverified; legacy state, rejected
0x7002  ready
0x7003  provisioning_owned; namespace preflight completed
```

`checksum` covers all 132 bytes with the checksum field treated as zero.

#### `time_anchor_entry` — 76 bytes inside each 512-byte slot

The public `time_anchor` argument is converted into this packed persisted
structure:

```text
byte range    size   field

+0x00..0x03     4    magic = 0x39415452, bytes "RTA9"
+0x04..0x05     2    revision = 1
+0x06..0x07     2    size = 76
+0x08..0x09     2    store_id
+0x0a            1    source_count
+0x0b            1    quality
+0x0c..0x0d      2    flags
+0x0e..0x0f      2    reserved
+0x10..0x13      4    source_mask
+0x14..0x17      4    boot_counter
+0x18..0x1f      8    sequence
+0x20..0x27      8    monotonic_us
+0x28..0x2f      8    utc_us
+0x30..0x37      8    uncertainty_us
+0x38..0x3f      8    max_durable_entry_id
+0x40..0x47      8    replace_sequence; 0 means no explicit replacement
+0x48..0x4b      4    checksum
```

Time-source bit mask:

```text
source_mask, uint32_t

 bit 31                                      bit 6  5       4      3    2       1    0
+------------------------------------------------+-------+------+---+--------+----+---+
| must be zero                                   |SERVER |MANUAL|RTC|CELLULAR|NTP |GPS|
|                                                |ESTIMATE|     |   |        |    |   |
+------------------------------------------------+-------+------+---+--------+----+---+
```

Time-anchor flags:

```text
flags, uint16_t

 bit 15                              bit 3  2          1    0
+-----------------------------------------+----------+----+---------+
| must be zero                            |CONTINUITY|PPS |CONSENSUS|
+-----------------------------------------+----------+----+---------+
```

Quality values:

```text
0  provisional
1  confirmed
```

`checksum` covers all 76 bytes with the checksum field treated as zero.

### Each `data_N.db` segment file

Let:

```text
S = segment_size
  = CONFIG_ON9RSTORE_SPARSE_FILE_SIZE for a new store
  = the persisted manifest value for an existing store

H = 8192 bytes of replicated headers
F = 8192 bytes of replicated footers

available      = S - H - F
max_entries    = floor(available / 44)
index_capacity = floor(max_entries / 64) + 1
index_size     = align_up(index_capacity * 24, 4096)
index_start    = S - F - index_size
data_start     = H
data_end       = index_start
```

The exact physical layout is:

```text
file offset

0x000000  +----------------------------------------------------------+
          | segment-header slot 0                          4096 bytes |
          |   68-byte segment_header                                  |
          |   4028-byte uninterpreted slot tail                       |
0x001000  +----------------------------------------------------------+
          | segment-header slot 1                          4096 bytes |
          |   68-byte segment_header                                  |
          |   4028-byte uninterpreted slot tail                       |
0x002000  +----------------------------------------------------------+ data_start
          |                                                          |
          | sequential, four-byte-aligned entries                    |
          |                                                          |
          | unused tail of entry region after the last valid entry   |
index_   +----------------------------------------------------------+ data_end
start     | sparse-index reservation                                  |
          |   index_count * 24 meaningful bytes from its beginning   |
          |   remainder is uninterpreted/stale                        |
S-0x2000 +----------------------------------------------------------+
          | sealed-footer slot 0                           4096 bytes |
          |   72-byte segment_footer                                  |
          |   4024-byte uninterpreted slot tail                       |
S-0x1000 +----------------------------------------------------------+
          | sealed-footer slot 1                           4096 bytes |
          |   72-byte segment_footer                                  |
          |   4024-byte uninterpreted slot tail                       |
S         +----------------------------------------------------------+
```

For the default `S = 1 MiB = 0x100000`:

```text
0x000000..0x000fff   header slot 0
0x001000..0x001fff   header slot 1
0x002000..0x0fafff   entry area          1,019,904 bytes
0x0fb000..0x0fdfff   sparse-index area      12,288 bytes
0x0fe000..0x0fefff   footer slot 0
0x0ff000..0x0fffff   footer slot 1

calculated index_capacity = 367 sparse-index entries
```

#### `segment_header` — 68 bytes

```text
byte range    size   field

+0x00..0x03     4    magic = 0x39485352, bytes "RSH9"
+0x04..0x05     2    revision = 1
+0x06..0x07     2    size = 68
+0x08..0x09     2    store_id
+0x0a..0x0b     2    state
+0x0c..0x0f     4    physical slot number
+0x10..0x17     8    generation
+0x18..0x1f     8    segment_size
+0x20..0x27     8    data_start
+0x28..0x2f     8    data_end
+0x30..0x37     8    index_start
+0x38..0x3b     4    index_capacity
+0x3c..0x3f     4    index_stride = 64
+0x40..0x43     4    checksum
```

Segment-header state values:

```text
0x7100  empty; generation must be 0
0x7101  active generation; sealed status is established by a valid footer
```

`checksum` covers all 68 bytes with the checksum field treated as zero.

#### One variable-length data entry

```text
entry file offset E

E+0x00  +----------------------------------------------------------+
        | entry_header                                      40 bytes |
E+0x28  +----------------------------------------------------------+
        | payload                                         len bytes |
        +----------------------------------------------------------+
        | entry CRC32                                       4 bytes |
        +----------------------------------------------------------+
        | zero padding                                    0..3 bytes|
E+size  +----------------------------------------------------------+

entry_size = align_up(40 + len + 4, 4)
CRC covers = entry_header + payload
CRC excludes alignment padding
```

`entry_header` byte layout:

```text
byte range    size   field

+0x00..0x03     4    magic = 0x39525352, bytes "RSR9"
+0x04..0x05     2    revision = 4
+0x06..0x07     2    type
+0x08..0x0f     8    entry_id
+0x10..0x17     8    uptime_us from esp_timer_get_time()
+0x18..0x1b     4    payload length
+0x1c..0x1d     2    store_id
+0x1e..0x1f     2    physical segment slot
+0x20..0x27     8    segment_generation
```

Entry ID bit layout:

```text
entry_id, uint64_t

 bit 63                         bit 40 39                              bit 0
+-----------------------------------+--------------------------------------+
| boot_counter, 24 bits             | per-boot append sequence, 40 bits    |
+-----------------------------------+--------------------------------------+

entry_id = (boot_counter << 40) | sequence
```

Entry type namespace:

```text
0x0000..0xffef  application entry types
0xfff0..0xffff  reserved by on9rstore
0xfffc          corrupted-entry marker reservation
0xffff          boot event
```

Boot-event payload:

```text
+0x00..0x03   reset_reason
+0x04..0x07   coredump_len
+0x08..       raw coredump bytes, when present
```

#### `sparse_index_entry` — 24 bytes

One index item is emitted for entry numbers 0, 64, 128, and so on:

```text
byte range    size   field

+0x00..0x07     8    entry_id
+0x08..0x0f     8    uptime_us
+0x10..0x13     4    entry file offset
+0x14..0x15     2    entry type
+0x16..0x17     2    reserved
```

Only `footer.index_count * 24` bytes at `header.index_start` are meaningful.

#### `segment_footer` — 72 bytes

```text
byte range    size   field

+0x00..0x03     4    magic = 0x39465352, bytes "RSF9"
+0x04..0x05     2    revision = 1
+0x06..0x07     2    size = 72
+0x08..0x09     2    store_id
+0x0a..0x0b     2    state = 0x7201, sealed
+0x0c..0x0f     4    physical slot number
+0x10..0x17     8    generation
+0x18..0x1f     8    first_entry_id
+0x20..0x27     8    last_entry_id
+0x28..0x2f     8    entry_count
+0x30..0x37     8    data_end; byte after the last sealed entry
+0x38..0x3b     4    index_count
+0x3c..0x3f     4    index_stride = 64
+0x40..0x43     4    index_checksum
+0x44..0x47     4    footer checksum
```

Checksum coverage:

```text
index_checksum  CRC32 of exactly index_count * 24 index bytes
footer checksum CRC32 of all 72 footer bytes with footer checksum set to zero
```

### CRC32 convention

Every CRC32 above uses:

```text
reflected polynomial  0xedb88320
initial value         0xffffffff
final operation       bitwise inversion
reference vector      CRC32("123456789") = 0xcbf43926
```

The 256-entry runtime lookup table is generated at compile time with
`constexpr`; it occupies 1024 bytes of read-only program storage and requires
no runtime initialization or heap memory.

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

## Reading entries

`read_entry()` performs an exact entry-ID lookup:

```c++
uint8_t payload[256] = {};
on9rstore_def::entry_header entry = {};

esp_err_t ret = store.read_entry(
    wanted_entry_id, payload, sizeof(payload), &entry);
```

The lookup selects the retained segment from its recovered descriptor,
binary-searches that segment's sparse index, then validates and scans the
short entry run beginning at the selected index item. The returned entry is
accepted only after its store identity, physical slot, segment generation,
bounds, ordering, and entry CRC validate.

The caller owns the payload buffer. A successful call copies the complete
payload; payloads are never silently truncated. If the buffer is too small,
the call returns `ESP_ERR_INVALID_SIZE`, fills `entry_info_out` when supplied,
and leaves `entry_info_out->len` set to the required payload size. Passing
`nullptr, 0` is therefore a bounded way to query an entry's header and required
payload size.

`read_next_entry()` provides forward inclusive range iteration:

```c++
on9rstore_def::entry_range_cursor cursor = {
    .next_entry_id = first_entry_id,
    .last_entry_id = last_entry_id,
};

while (store.read_next_entry(
           &cursor, payload, sizeof(payload), &entry) == ESP_OK) {
    // Consume entry and its complete payload.
}
```

`next_entry_id == 0` begins at the oldest retained entry. Entry-ID gaps are
skipped. After a successful read the cursor advances beyond the returned ID.
It does not advance after `ESP_ERR_INVALID_SIZE` or another error, so the
caller can retry with a larger buffer. Once no retained entry remains in the
inclusive range, the API returns `ESP_ERR_NOT_FOUND` and sets
`cursor.finished`.

Each cursor call takes a fresh retained-state snapshot; a cursor does not pin
the entire range across calls. Reads include entries still in the active write
buffer at the instant of that per-call snapshot. The write buffer, active
sparse index, and segment descriptors are copied under the writer lock, after
which normal appends may continue. The reader lock prevents segment retirement
and physical-slot reuse until that individual read finishes, and
deinitialisation waits for the same lock.

`read_next_entry_by_uptime()` iterates entries from one boot and can optionally
restrict them to an inclusive monotonic-uptime range:

```c++
on9rstore_def::boot_uptime_range_cursor cursor = {
    .boot_counter = wanted_boot,
    .first_uptime_us = range_start_us,
    .last_uptime_us = range_end_us,
};

while (store.read_next_entry_by_uptime(
           &cursor, payload, sizeof(payload), &entry) == ESP_OK) {
    // Entries are returned in entry-ID order.
}
```

Uptime resets at boot, so the boot counter is mandatory and must be in the
persisted 24-bit range. Leaving the uptime bounds at their defaults
(`0..UINT64_MAX`) iterates the entire boot. The cursor skips entry-ID gaps,
crosses retained segment boundaries, and uses each segment's sparse uptime
index to begin near the requested lower bound. It has the same payload-buffer,
retry, snapshot, and `finished` behaviour as `read_next_entry()`.

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

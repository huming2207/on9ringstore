# on9rstore

`on9rstore` is a fixed-size FAT-file ring store for long-lived device logs.
For a new `/data/log.db` (or another caller-selected path), it first creates
`/data/log.db.on9rstore-new` with
`esp_vfs_fat_create_contiguous_file(..., true)`. It writes the store identity
and empty checkpoints there, syncs them, then renames the staging file to the
final path. It verifies that the resulting file remains contiguous and rejects
an existing final file with a different size or no valid revision-3 format
header.

The final database file never grows, truncates, or deletes itself. Ordinary
ring writes therefore do not allocate clusters or grow the file: the intended
benefits are sustained sequential-ish I/O, no later file fragmentation, and no
FAT-chain updates during normal writes. `fsync()` can still rewrite the file's
directory entry in place, such as for its timestamp.

The constructor is intentionally small and keeps the mount-specific choice at
the call site:

```c++
on9rstore_cfg rstore_cfg = {
    .base_path = "/data",
    .file_size = 1024ULL * 1024ULL * 1024ULL,
    .write_buffer_size = 8192,
    .copy_coredump = true,
};

on9rstore rstore("/data/log.db", &rstore_cfg);
ESP_ERROR_CHECK(rstore.init());
```

`init()` increments the persisted 24-bit boot counter and writes a boot entry.
When ESP-IDF has a new valid flash coredump, `copy_coredump` streams the
boot-event header and raw coredump image into that boot entry in bounded chunks.
It fingerprints the image to avoid copying the same retained coredump after a
later normal reset, and deliberately does not erase ESP-IDF's coredump
partition. Entry IDs are `(boot_counter << 40) | sequence`; the 40-bit
sequence starts at one each boot and advances for every accepted
`append_entry()` call. A 64-bit `esp_timer_get_time()` uptime timestamp is
stored in each entry.

The revision-3 on-disk layout is little-endian and self-identifying. Its first
16 KiB are:

```text
├── store header slot 0: 4096 B
├── store header slot 1: 4096 B
├── metadata slot 0:     4096 B
├── metadata slot 1:     4096 B
└── ring data area: file_size - 16384 B
```

Each replicated store header contains a non-zero 16-bit ID generated from
`esp_random()` when the database incarnation is created. The entry layout is:

```text
[magic: u32] [revision: u16] [type: u16] [entry_id: u64]
[uptime_us: u64] [payload_len: u32] [store_id: u16]
[payload] [crc32] [zero padding to 4 B]
```

`type` is a 16-bit application namespace. Values `0xfff0` through `0xffff`
are reserved by `on9rstore` and are rejected by the public `append_entry()`;
internal writers (boot event, time sync, padding) bypass that check. A complete entry is identified by its header,
revision, matching store ID, sane bounds, and trailing CRC32. The metadata
checkpoints also carry and validate the store ID.

Writes are batched until `flush_write()` unless the entry is larger than
`write_buffer_size` or `force_flush` is selected. Large entries stream directly
from the caller buffer rather than allocating a second entry-sized heap buffer.
When the ring needs to reuse old bytes, `on9rstore` first persists the retired
head, then writes the new data, syncs it, and finally commits the new tail. A
reset can therefore lose the entry being appended but cannot leave committed
metadata pointing at bytes that were already reused.

The two CRC-protected metadata slots are checkpoints, not the only recovery
source. If neither is valid, a pre-existing file with a valid store header is
scanned for complete revision-3 entries, matching store IDs, wrap padding,
entry IDs, and CRCs. A final file without a valid store header is never scanned
or reset automatically. This prevents recycled FAT clusters from becoming an
apparently valid new database. Runtime head removal skips a sane-length CRC-bad
entry and resynchronises past torn bytes, so one corrupt entry does not
permanently stop logging.

Preallocation reduces, but cannot eliminate, FAT sudden-power-loss risk: it
cannot protect against an SD card that lies about completed writes, nor
corruption of FAT structures caused by power loss during the initial allocation
or unrelated filesystem activity. `deinit()` serialises with active append and
flush operations; destroying the C++ object itself still requires the caller to
ensure no other task can call it again.

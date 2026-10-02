#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <unistd.h>

#include <diskio_impl.h>
#include <diskio_wl.h>
#include <esp_partition.h>
#include <esp_timer.h>
#include <esp_vfs_fat.h>
#include <sdkconfig.h>
#include <ff.h>
#include <unity.h>
#include <wear_levelling.h>

#include "on9rstore.hpp"

static const constexpr char BASE_PATH[] = "/store";
static const constexpr size_t PAYLOAD_LEN = 1000;
static const constexpr uint16_t TEST_ENTRY = 1;

static uint8_t payload[PAYLOAD_LEN] = {};
static uint8_t read_buf[PAYLOAD_LEN] = {};
static on9rstore *store = nullptr;

#if CONFIG_IDF_TARGET_LINUX
// esp_timer has no linux implementation; on9rstore only needs uptime
extern "C" int64_t esp_timer_get_time(void)
{
    timespec now = {};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<int64_t>(now.tv_sec) * 1000000 + now.tv_nsec / 1000;
}

static char drive[3] = {};
static FATFS *fs = nullptr;

static void mount_fat()
{
    wl_handle_t wl_handle = WL_INVALID_HANDLE;
    const esp_partition_t *partition =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "storage");
    TEST_ASSERT_NOT_NULL(partition);
    TEST_ASSERT_EQUAL(ESP_OK, wl_mount(partition, &wl_handle));

    BYTE pdrv = 0;
    TEST_ASSERT_EQUAL(ESP_OK, ff_diskio_get_drive(&pdrv));
    TEST_ASSERT_EQUAL(ESP_OK, ff_diskio_register_wl_partition(pdrv, wl_handle));
    drive[0] = static_cast<char>('0' + pdrv);
    drive[1] = ':';

    esp_vfs_fat_conf_t conf = {};
    conf.base_path = BASE_PATH;
    conf.fat_drive = drive;
    conf.max_files = 8;
    TEST_ASSERT_EQUAL(ESP_OK, esp_vfs_fat_register(&conf, &fs));
}

static void format_fat()
{
    BYTE work_buf[FF_MAX_SS] = {};
    const MKFS_PARM opt = {FM_ANY, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL(FR_OK, f_mkfs(drive, &opt, work_buf, sizeof(work_buf)));
    TEST_ASSERT_EQUAL(FR_OK, f_mount(fs, drive, 1));
}
#else
static void mount_fat()
{
    wl_handle_t wl_handle = WL_INVALID_HANDLE;
    esp_vfs_fat_mount_config_t conf = {};
    conf.format_if_mount_failed = true;
    conf.max_files = 8;
    conf.allocation_unit_size = 4096;
    TEST_ASSERT_EQUAL(ESP_OK, esp_vfs_fat_spiflash_mount_rw_wl(BASE_PATH, "storage", &conf, &wl_handle));
}

static void format_fat()
{
    TEST_ASSERT_EQUAL(ESP_OK, esp_vfs_fat_spiflash_format_rw_wl(BASE_PATH, "storage"));
}
#endif

static void append_until_refused(on9rstore &store, size_t payload_len, uint32_t *appended_out)
{
    uint32_t appended = 0;
    while (store.append_entry(TEST_ENTRY, payload, payload_len) == ESP_OK) {
        appended += 1;
        TEST_ASSERT_LESS_THAN(10000, appended);
    }
    *appended_out = appended;
}

static uint64_t read_first_entry_id(on9rstore &store)
{
    on9rstore_def::entry_range_cursor cursor = {};
    on9rstore_def::entry_header header = {};
    TEST_ASSERT_EQUAL(ESP_OK, store.read_next_entry(&cursor, read_buf, sizeof(read_buf), &header));
    return header.entry_id;
}

// A fresh volume per test; f_expand() can fail once FatFs's next-cluster hint
// sits past the free space, which deleting the previous store's files leaves.
void setUp()
{
    format_fat();
}

void tearDown()
{
    delete store;
    store = nullptr;
}

static void open_store(bool protect_unacked)
{
    on9rstore_cfg cfg = {};
    cfg.protect_unacked = protect_unacked;
    store = new on9rstore(BASE_PATH, &cfg);
    TEST_ASSERT_EQUAL(ESP_OK, store->init());
}

static void test_ack_persists_and_only_moves_forward()
{
    open_store(false);
    TEST_ASSERT_EQUAL_UINT64(0, store->get_acked_entry_id());

    on9rstore_def::entry_header first = {};
    TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, payload, sizeof(payload), &first));
    TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, payload, sizeof(payload)));

    TEST_ASSERT_EQUAL(ESP_OK, store->set_acked_entry_id(first.entry_id));
    TEST_ASSERT_EQUAL(ESP_OK, store->set_acked_entry_id(first.entry_id - 1));
    TEST_ASSERT_EQUAL_UINT64(first.entry_id, store->get_acked_entry_id());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, store->set_acked_entry_id(store->get_newest_entry_id() + 1));
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());

    TEST_ASSERT_EQUAL(ESP_OK, store->init());
    TEST_ASSERT_EQUAL_UINT64(first.entry_id, store->get_acked_entry_id());
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());
}

static void test_unprotected_store_overwrites_oldest()
{
    open_store(false);
    const uint64_t oldest = read_first_entry_id(*store);

    for (uint32_t count = 0; count < 300; count += 1) {
        TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, payload, sizeof(payload)));
    }
    TEST_ASSERT_GREATER_THAN_UINT64(oldest, read_first_entry_id(*store));
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());
}

static void test_protected_store_keeps_unacked_entries()
{
    open_store(true);
    const uint64_t oldest = read_first_entry_id(*store);

    uint32_t appended = 0;
    append_until_refused(*store, sizeof(payload), &appended);
    TEST_ASSERT_GREATER_THAN(0, appended);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, store->append_entry(TEST_ENTRY, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_UINT64(oldest, read_first_entry_id(*store));

    // A refused append must leave the active segment writable after an ack
    TEST_ASSERT_EQUAL(ESP_OK, store->set_acked_entry_id(store->get_newest_entry_id()));
    TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, payload, sizeof(payload)));
    TEST_ASSERT_GREATER_THAN_UINT64(oldest, read_first_entry_id(*store));
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());
}

static void test_full_protected_store_still_initialises()
{
    // Fill with entries the size of the boot entry so the next boot entry is refused too
    open_store(true);
    uint32_t appended = 0;
    append_until_refused(*store, sizeof(on9rstore_def::boot_event), &appended);
    const uint64_t newest = store->get_newest_entry_id();
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());

    TEST_ASSERT_EQUAL(ESP_OK, store->init());
    TEST_ASSERT_EQUAL_UINT64(newest, store->get_newest_entry_id());
    TEST_ASSERT_EQUAL(ESP_OK, store->set_acked_entry_id(newest));
    TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, payload, sizeof(payload)));
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());
}

static void test_entry_utc_follows_time_anchor()
{
    open_store(false);
    on9rstore_def::entry_header entry = {};
    TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, payload, 8, &entry));
    on9rstore_def::entry_utc_info utc_info = {};
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, store->get_entry_utc(entry, &utc_info));

    // An anchor set later in the same boot also dates earlier entries
    on9rstore_def::time_anchor anchor = {};
    anchor.source_mask = on9rstore_def::TIME_SOURCE_MANUAL;
    anchor.source_count = 1;
    anchor.monotonic_us = entry.uptime_us + 5000000;
    anchor.utc_us = 1800000000000000ULL;
    TEST_ASSERT_EQUAL(ESP_OK, store->append_time_anchor(anchor));
    TEST_ASSERT_EQUAL(ESP_OK, store->get_entry_utc(entry, &utc_info));
    TEST_ASSERT_EQUAL_UINT64(anchor.utc_us - 5000000, utc_info.utc_us);
}

static void test_can_append_predicts_refusal()
{
    open_store(true);
    uint32_t appended = 0;
    while (store->can_append(sizeof(payload))) {
        TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, payload, sizeof(payload)));
        appended += 1;
        TEST_ASSERT_LESS_THAN(10000, appended);
    }
    TEST_ASSERT_GREATER_THAN(0, appended);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, store->append_entry(TEST_ENTRY, payload, sizeof(payload)));

    TEST_ASSERT_EQUAL(ESP_OK, store->set_acked_entry_id(store->get_newest_entry_id()));
    TEST_ASSERT_TRUE(store->can_append(sizeof(payload)));
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());
}

static esp_err_t append_anchor_now()
{
    on9rstore_def::time_anchor anchor = {};
    anchor.source_mask = on9rstore_def::TIME_SOURCE_MANUAL;
    anchor.source_count = 1;
    anchor.monotonic_us = esp_timer_get_time();
    anchor.utc_us = 1800000000000000ULL + anchor.monotonic_us;
    return store->append_time_anchor(anchor);
}

static void test_acked_boots_free_time_anchors()
{
    // Each boot writes a boot entry and one anchor; nothing rotates out of the ring
    open_store(false);
    TEST_ASSERT_EQUAL(ESP_OK, append_anchor_now());
    for (uint32_t boot = 1; boot < CONFIG_ON9RSTORE_TIME_ANCHOR_CNT; boot += 1) {
        TEST_ASSERT_EQUAL(ESP_OK, store->deinit());
        TEST_ASSERT_EQUAL(ESP_OK, store->init());
        TEST_ASSERT_EQUAL(ESP_OK, append_anchor_now());
    }

    // Every anchor belongs to a boot whose entries are still uncollected
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());
    TEST_ASSERT_EQUAL(ESP_OK, store->init());
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, append_anchor_now());

    TEST_ASSERT_EQUAL(ESP_OK, store->set_acked_entry_id(store->get_newest_entry_id()));
    TEST_ASSERT_EQUAL(ESP_OK, append_anchor_now());
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());
}

#if !CONFIG_IDF_TARGET_LINUX
// Flips the first byte of @p marker in a segment file; false when the file does not hold it.
// esp32 only: on the linux target a second open() of a segment file returns an unusable fd.
static bool corrupt_payload(const char *path, const uint8_t *marker, size_t marker_len)
{
    const int fd = open(path, O_RDWR);
    if (fd < 0) {
        return false;
    }
    const off_t file_len = lseek(fd, 0, SEEK_END);
    auto *content = static_cast<uint8_t *>(malloc(file_len));
    TEST_ASSERT_NOT_NULL(content);
    TEST_ASSERT_EQUAL(0, lseek(fd, 0, SEEK_SET));
    TEST_ASSERT_EQUAL(file_len, read(fd, content, file_len));

    off_t found = -1;
    for (off_t offset = 0; offset + (off_t)marker_len <= file_len && found < 0; offset += 1) {
        if (memcmp(content + offset, marker, marker_len) == 0) {
            found = offset;
        }
    }
    if (found >= 0) {
        const uint8_t flipped = marker[0] ^ 0xff;
        TEST_ASSERT_EQUAL(found, lseek(fd, found, SEEK_SET));
        TEST_ASSERT_EQUAL(1, write(fd, &flipped, 1));
    }
    free(content);
    close(fd);
    return found >= 0;
}

static void test_corrupt_entry_is_reported_and_skipped()
{
    static const uint8_t marker[] = "corrupt this entry";
    open_store(false);
    on9rstore_def::entry_header before = {};
    on9rstore_def::entry_header corrupt = {};
    on9rstore_def::entry_header after = {};
    TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, payload, 8, &before));
    TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, marker, sizeof(marker), &corrupt));
    TEST_ASSERT_EQUAL(ESP_OK, store->append_entry(TEST_ENTRY, payload, 8, &after, portMAX_DELAY, true));

    bool corrupted = false;
    for (uint32_t slot = 0; slot < CONFIG_ON9STORE_SPARSE_FILE_CNT && !corrupted; slot += 1) {
        char path[32] = {};
        snprintf(path, sizeof(path), "%s/data_%lu.db", BASE_PATH, (unsigned long)slot);
        corrupted = corrupt_payload(path, marker, sizeof(marker));
    }
    TEST_ASSERT_TRUE(corrupted);

    // The corrupt entry is named, and reading resumes after it
    on9rstore_def::entry_range_cursor cursor = {};
    cursor.next_entry_id = before.entry_id + 1;
    on9rstore_def::entry_header header = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_CRC, store->read_next_entry(&cursor, read_buf, sizeof(read_buf), &header));
    TEST_ASSERT_EQUAL_UINT64(corrupt.entry_id, header.entry_id);

    cursor.next_entry_id = header.entry_id + 1;
    TEST_ASSERT_EQUAL(ESP_OK, store->read_next_entry(&cursor, read_buf, sizeof(read_buf), &header));
    TEST_ASSERT_EQUAL_UINT64(after.entry_id, header.entry_id);
    TEST_ASSERT_EQUAL(ESP_OK, store->deinit());
}

#endif

extern "C" void app_main(void)
{
    mount_fat();
    memset(payload, 0xa5, sizeof(payload));

    UNITY_BEGIN();
    RUN_TEST(test_ack_persists_and_only_moves_forward);
    RUN_TEST(test_unprotected_store_overwrites_oldest);
    RUN_TEST(test_protected_store_keeps_unacked_entries);
    RUN_TEST(test_full_protected_store_still_initialises);
    RUN_TEST(test_entry_utc_follows_time_anchor);
    RUN_TEST(test_can_append_predicts_refusal);
    RUN_TEST(test_acked_boots_free_time_anchors);
#if !CONFIG_IDF_TARGET_LINUX
    RUN_TEST(test_corrupt_entry_is_reported_and_skipped);
#endif
    const int failures = UNITY_END();
#if CONFIG_IDF_TARGET_LINUX
    exit(failures);
#else
    (void)failures;
    printf("on9rstore tests done\n");
#endif
}

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dirent.h>
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

    char drive[3] = {static_cast<char>('0' + pdrv), ':', 0};
    BYTE work_buf[FF_MAX_SS] = {};
    const MKFS_PARM opt = {FM_ANY, 0, 0, 0, 0};
    TEST_ASSERT_EQUAL(FR_OK, f_mkfs(drive, &opt, work_buf, sizeof(work_buf)));

    FATFS *fs = nullptr;
    esp_vfs_fat_conf_t conf = {};
    conf.base_path = BASE_PATH;
    conf.fat_drive = drive;
    conf.max_files = 8;
    TEST_ASSERT_EQUAL(ESP_OK, esp_vfs_fat_register(&conf, &fs));
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
#endif

static void remove_store_files()
{
    DIR *dir = opendir(BASE_PATH);
    TEST_ASSERT_NOT_NULL(dir);
    char path[300] = {};
    for (dirent *entry = readdir(dir); entry != nullptr; entry = readdir(dir)) {
        snprintf(path, sizeof(path), "%s/%s", BASE_PATH, entry->d_name);
        unlink(path);
    }
    closedir(dir);
}

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

void setUp()
{
    remove_store_files();
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

extern "C" void app_main(void)
{
    mount_fat();
    memset(payload, 0xa5, sizeof(payload));

    UNITY_BEGIN();
    RUN_TEST(test_ack_persists_and_only_moves_forward);
    RUN_TEST(test_unprotected_store_overwrites_oldest);
    RUN_TEST(test_protected_store_keeps_unacked_entries);
    RUN_TEST(test_full_protected_store_still_initialises);
    const int failures = UNITY_END();
#if CONFIG_IDF_TARGET_LINUX
    exit(failures);
#else
    (void)failures;
    printf("on9rstore tests done\n");
#endif
}

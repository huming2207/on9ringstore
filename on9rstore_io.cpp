#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <unistd.h>

#include <esp_log.h>
#include <esp_vfs_fat.h>

#include "on9rstore.hpp"

esp_err_t on9rstore::build_manifest_path()
{
    const size_t len = strlen(file_path);
    const char *separator = len > 0 && file_path[len - 1] == '/' ? "" : "/";
    const int result = snprintf(manifest_path, sizeof(manifest_path),
                                "%s%smanifest.db", file_path, separator);
    if (result < 0 || static_cast<size_t>(result) >= sizeof(manifest_path)) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

esp_err_t on9rstore::build_data_path(uint32_t slot, char *path_out,
                                     size_t path_out_len) const
{
    if (path_out == nullptr || path_out_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t len = strlen(file_path);
    const char *separator = len > 0 && file_path[len - 1] == '/' ? "" : "/";
    const int result = snprintf(path_out, path_out_len, "%s%sdata_%" PRIu32 ".db",
                                file_path, separator, slot);
    if (result < 0 || static_cast<size_t>(result) >= path_out_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}

esp_err_t on9rstore::validate_contiguous_file(const char *path, uint64_t size) const
{
    struct stat file_stat = {};
    if (stat(path, &file_stat) != 0) {
        ESP_LOGE(TAG, "File: stat(%s) failed: errno=%d", path, errno);
        return ESP_FAIL;
    }

    if (!S_ISREG(file_stat.st_mode) ||
        static_cast<uint64_t>(file_stat.st_size) != size) {
        ESP_LOGE(TAG, "File: invalid size/type for %s", path);
        return ESP_ERR_INVALID_SIZE;
    }

    bool contiguous = false;
    const esp_err_t ret =
        esp_vfs_fat_test_contiguous_file(file_path, path, &contiguous);
    if (ret != ESP_OK || !contiguous) {
        ESP_LOGE(TAG, "File: %s is not contiguous: ret=0x%x", path, ret);
        return ret == ESP_OK ? ESP_ERR_INVALID_STATE : ret;
    }

    return ESP_OK;
}

esp_err_t on9rstore::provision_contiguous_file(const char *path, uint64_t size,
                                               bool *created_out) const
{
    if (path == nullptr || created_out == nullptr || size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    *created_out = false;
    struct stat file_stat = {};
    const int stat_result = stat(path, &file_stat);
    if (stat_result == 0 && file_stat.st_size > 0) {
        return validate_contiguous_file(path, size);
    }

    if (stat_result != 0 && errno != ENOENT) {
        ESP_LOGE(TAG, "File: stat(%s) failed: errno=%d", path, errno);
        return ESP_FAIL;
    }

    const esp_err_t ret =
        esp_vfs_fat_create_contiguous_file(file_path, path, size, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "File: contiguous create failed for %s: ret=0x%x errno=%d",
                 path, ret, errno);
        return ret;
    }

    *created_out = true;
    return validate_contiguous_file(path, size);
}

esp_err_t on9rstore::open_file(const char *path, int *fd_out) const
{
    if (path == nullptr || fd_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const int opened_fd = open(path, O_RDWR);
    if (opened_fd < 0) {
        ESP_LOGE(TAG, "File: open(%s) failed: errno=%d", path, errno);
        return ESP_FAIL;
    }

    *fd_out = opened_fd;
    return ESP_OK;
}

esp_err_t on9rstore::read_exact_fd(int file_fd, uint64_t file_size,
                                   uint64_t offset, void *buf_out,
                                   size_t len) const
{
    if (len == 0) {
        return ESP_OK;
    }
    if (file_fd < 0 || buf_out == nullptr || len > file_size ||
        offset > file_size - len) {
        return ESP_ERR_INVALID_ARG;
    }

    if (lseek(file_fd, static_cast<off_t>(offset), SEEK_SET) < 0) {
        ESP_LOGE(TAG, "File: read seek failed: errno=%d", errno);
        return ESP_FAIL;
    }

    uint8_t *out = static_cast<uint8_t *>(buf_out);
    size_t remaining = len;
    while (remaining > 0) {
        const ssize_t result = read(file_fd, out, remaining);
        if (result <= 0) {
            ESP_LOGE(TAG, "File: read failed: errno=%d", errno);
            return ESP_FAIL;
        }

        out += result;
        remaining -= static_cast<size_t>(result);
    }

    return ESP_OK;
}

esp_err_t on9rstore::write_exact_fd(int file_fd, uint64_t file_size,
                                    uint64_t offset, const void *buf,
                                    size_t len) const
{
    if (len == 0) {
        return ESP_OK;
    }
    if (file_fd < 0 || buf == nullptr || len > file_size ||
        offset > file_size - len) {
        return ESP_ERR_INVALID_ARG;
    }

    if (lseek(file_fd, static_cast<off_t>(offset), SEEK_SET) < 0) {
        ESP_LOGE(TAG, "File: write seek failed: errno=%d", errno);
        return ESP_FAIL;
    }

    const uint8_t *in = static_cast<const uint8_t *>(buf);
    size_t remaining = len;
    while (remaining > 0) {
        const ssize_t result = write(file_fd, in, remaining);
        if (result <= 0) {
            ESP_LOGE(TAG, "File: write failed: errno=%d", errno);
            return ESP_FAIL;
        }

        in += result;
        remaining -= static_cast<size_t>(result);
    }

    return ESP_OK;
}

esp_err_t on9rstore::sync_fd(int file_fd) const
{
    if (file_fd < 0 || fsync(file_fd) != 0) {
        ESP_LOGE(TAG, "File: fsync failed: errno=%d", errno);
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t on9rstore::open_reader_segment(uint32_t slot)
{
    close_reader_segment();

    char path[PATH_MAX] = {};
    esp_err_t ret = build_data_path(slot, path, sizeof(path));
    if (ret != ESP_OK) {
        return ret;
    }

    return open_file(path, &reader_fd);
}

void on9rstore::close_reader_segment()
{
    if (reader_fd >= 0) {
        (void)close(reader_fd);
        reader_fd = -1;
    }
}

void on9rstore::close_writer_segment()
{
    if (writer_fd >= 0) {
        (void)close(writer_fd);
        writer_fd = -1;
    }

    active_data_path[0] = '\0';
}

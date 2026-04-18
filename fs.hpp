// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file fs.hpp
 * @brief File system subsystem header for miniOS v1.7.
 */

#pragma once

#include "core.hpp"
#include "hal.hpp"
#include <array>
#include <string_view>
#include <optional>
#include <cstdint>

namespace fs {

constexpr size_t MAX_FILES = 64;
constexpr size_t MAX_NAME_LENGTH = 32;
constexpr size_t MAX_PATH_LENGTH = 128;
constexpr size_t MAX_FILE_SIZE = 65536;

enum class Permissions : uint8_t {
    READ = 1,
    WRITE = 2,
    EXECUTE = 4,
    READ_WRITE = READ | WRITE,
    READ_EXECUTE = READ | EXECUTE,
    WRITE_EXECUTE = WRITE | EXECUTE,
    READ_WRITE_EXECUTE = READ | WRITE | EXECUTE,
};

struct FileEntry {
    std::array<char, MAX_PATH_LENGTH> path{};
    bool is_directory = false;
    Permissions permissions = Permissions::READ_WRITE_EXECUTE;
    size_t size = 0;
    std::array<uint8_t, MAX_FILE_SIZE> data{};
};

class FileSystem {
public:
    FileSystem();

    bool init() noexcept;
    bool create_file(std::string_view path, bool is_directory);
    bool write_file(std::string_view path, const void* data, size_t size);
    size_t read_file(std::string_view path, void* buffer, size_t max_size) const;
    bool read_file(std::string_view path, std::string& content) const;
    bool delete_file(std::string_view path);
    bool file_exists(std::string_view path) const;
    bool list_files(std::string_view path, kernel::hal::UARTDriverOps* uart_ops) const;
    bool set_permissions(std::string_view path, Permissions perms);

private:
    std::string_view normalize_path(std::string_view path) const;
    bool is_valid_path_format(std::string_view path) const;
    std::optional<size_t> find_entry_index(std::string_view path) const;
    FileEntry* find_entry(std::string_view path);
    const FileEntry* find_entry(std::string_view path) const;
    std::string_view get_parent_path(std::string_view path) const;

    std::array<FileEntry, MAX_FILES> entries_;
    size_t entry_count_ = 0;
    mutable kernel::core::Spinlock lock_;
};

extern FileSystem g_file_system;

} // namespace fs

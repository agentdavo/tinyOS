// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file fs.hpp
 * @brief File system subsystem header for miniOS v1.7.
 * @details
 * Defines a simple file system for managing files and directories.
 *
 * @version 1.7
 * @see fs.cpp, core.hpp, hal.hpp
 */

#ifndef FS_HPP
#define FS_HPP

#include "core.hpp"
#include <array>
#include <string_view>

namespace fs {

constexpr size_t MAX_FILES = 64;
constexpr size_t MAX_NAME_LENGTH = 32;
constexpr size_t MAX_CONTENT_SIZE = 1024;

struct FileEntry {
    char name[MAX_NAME_LENGTH];
    bool is_directory;
    std::array<char, MAX_CONTENT_SIZE> content;
    size_t content_size;
};

class FileSystem {
public:
    bool init() noexcept;
    bool create_file(std::string_view path, bool is_directory);
    bool delete_file(std::string_view path);
    bool read_file(std::string_view path, std::array<char, MAX_CONTENT_SIZE>& content);
    bool list_files(std::string_view path, kernel::hal::UARTDriverOps* uart_ops);

private:
    std::array<FileEntry, MAX_FILES> files_;
    size_t num_files_ = 0;
    mutable kernel::core::Spinlock fs_lock_;
};

extern FileSystem g_file_system;

} // namespace fs

#endif // FS_HPP
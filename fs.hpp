// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file fs.hpp
 * @brief RAM File System (RAMFS) subsystem header for miniOS v1.7.
 * @details
 * Defines a lightweight, in-memory file system for miniOS, supporting file and directory creation,
 * read/write operations, and permissions (read/write/execute). Enhanced in v1.7 with increased
 * file size limit (64KB from 16KB), improved error handling, clearer documentation, and modern
 * C++20 practices, retaining all v1.6 functionality including directory support and CLI integration.
 *
 * C++20 features:
 * - std::string_view for path handling
 * - std::span for data buffers
 * - std::optional for safer returns
 *
 * @version 1.7
 * @see fs.cpp, miniOS.hpp, util.hpp, cli.hpp
 */

#ifndef FS_HPP
#define FS_HPP

#include "miniOS.hpp" // For kernel::hal::UARTDriverOps, kernel::Spinlock
#include <string>      // For std::string in read_file overload
#include <string_view>
#include <span>
#include <array>
#include <vector>
#include <optional>
#include <algorithm>   // For std::find_if in delete_file

namespace fs {

constexpr size_t MAX_FILES = 64;
constexpr size_t MAX_PATH_LENGTH = 64; // Includes null terminator
constexpr size_t MAX_FILE_SIZE = 65536; // 64KB

/**
 * @brief File system entry permissions.
 */
enum class Permissions : uint8_t {
    NONE = 0b000,
    EXECUTE = 0b001,
    WRITE = 0b010,
    WRITE_EXECUTE = 0b011,
    READ = 0b100,
    READ_EXECUTE = 0b101,
    READ_WRITE = 0b110,
    READ_WRITE_EXECUTE = 0b111,
};

/**
 * @brief Represents a file or directory entry in the RAMFS.
 */
struct FileEntry {
    std::array<char, MAX_PATH_LENGTH> path; ///< Null-terminated absolute path.
    bool is_directory = false;              ///< True if this entry is a directory.
    Permissions permissions = Permissions::READ_WRITE_EXECUTE; ///< Entry permissions.
    std::vector<uint8_t> data;              ///< File content (empty for directories).
    size_t size = 0;                        ///< Actual size of file data in bytes.
};

/**
 * @brief RAM File System class.
 */
class FileSystem {
public:
    FileSystem();

    bool create_file(std::string_view path, bool is_directory = false);
    bool write_file(std::string_view path, const void* data, size_t size);
    size_t read_file(std::string_view path, void* buffer, size_t max_size) const;
    bool read_file(std::string_view path, std::string& content) const; // Overload for std::string
    bool delete_file(std::string_view path); // Method to delete files/dirs
    bool file_exists(std::string_view path) const;
    bool list_files(std::string_view path, kernel::hal::UARTDriverOps* uart_ops) const;
    bool set_permissions(std::string_view path, Permissions perms);

private:
    std::array<FileEntry, MAX_FILES> entries_; 
    size_t entry_count_ = 0;                   
    mutable kernel::Spinlock fs_lock_;         

    std::optional<size_t> find_entry_index(std::string_view path) const;
    FileEntry* find_entry(std::string_view path); 
    const FileEntry* find_entry(std::string_view path) const; 
    bool is_valid_path_format(std::string_view path) const;
    std::string_view normalize_path(std::string_view path) const;
    std::string_view get_parent_path(std::string_view path) const;
};

// Global instance extern is in miniOS.hpp
// extern FileSystem g_file_system; 

} // namespace fs

#endif // FS_HPP
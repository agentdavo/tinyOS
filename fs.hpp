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

#include "miniOS.hpp"
#include <string_view>
#include <span>
#include <array>
#include <vector>
#include <optional>

namespace fs {

constexpr size_t MAX_FILES = 64;
constexpr size_t MAX_PATH_LENGTH = 64;
constexpr size_t MAX_FILE_SIZE = 65536; // 64KB (increased from 16KB in v1.6)

/**
 * @brief File system entry (file or directory).
 */
struct FileEntry {
    char path[MAX_PATH_LENGTH]; ///< File/directory path
    bool is_directory = false; ///< True if directory
    uint8_t permissions = 0; ///< Permissions (rwx: 0b00000rwx)
    std::vector<uint8_t> data; ///< File data (empty for directories)
    std::vector<size_t> children; ///< Child indices (for directories)
};

/**
 * @brief RAM File System class.
 */
class FileSystem {
public:
    FileSystem() = default;

    /**
     * @brief Creates a file or directory.
     * @param path File/directory path
     * @param is_directory True to create a directory
     * @return True if created, false if path exists or limit reached
     */
    bool create_file(std::string_view path, bool is_directory = false);

    /**
     * @brief Writes data to a file.
     * @param path File path
     * @param data Data to write
     * @param size Data size (up to MAX_FILE_SIZE)
     * @return True if written, false if file not found, read-only, or too large
     */
    bool write_file(std::string_view path, const void* data, size_t size);

    /**
     * @brief Reads data from a file.
     * @param path File path
     * @param buffer Output buffer
     * @param max_size Maximum bytes to read
     * @return Number of bytes read, 0 on failure
     */
    size_t read_file(std::string_view path, void* buffer, size_t max_size) const;

    /**
     * @brief Checks if a file or directory exists.
     * @param path File/directory path
     * @return True if exists, false otherwise
     */
    bool file_exists(std::string_view path) const;

    /**
     * @brief Lists files in a directory.
     * @param path Directory path
     * @param uart_ops UART driver for output
     * @return True if listed, false if not a directory or invalid path
     */
    bool list_files(std::string_view path, kernel::hal::UARTDriverOps* uart_ops) const;

    /**
     * @brief Sets file permissions.
     * @param path File/directory path
     * @param permissions Permission bits (rwx: 0b00000rwx)
     * @return True if set, false if file not found
     */
    bool set_permissions(std::string_view path, uint8_t permissions);

private:
    std::array<FileEntry, MAX_FILES> files_; ///< File system entries
    size_t file_count_ = 0; ///< Number of files/directories

    /**
     * @brief Finds a file by path.
     * @param path File path
     * @return Index in files_ array, or std::nullopt if not found
     */
    std::optional<size_t> find_file(std::string_view path) const;

    /**
     * @brief Validates a path.
     * @param path Path to validate
     * @return True if valid, false otherwise
     */
    bool is_valid_path(std::string_view path) const;
};

extern FileSystem g_file_system; ///< Global file system instance

} // namespace fs

#endif // FS_HPP
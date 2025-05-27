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

// Forward declaration from miniOS.hpp if needed
// namespace kernel { namespace hal { struct UARTDriverOps; }}
// namespace kernel { class Spinlock; class ScopedLock; }


namespace fs {

constexpr size_t MAX_FILES = 64;
constexpr size_t MAX_PATH_LENGTH = 64; // Includes null terminator
constexpr size_t MAX_FILE_SIZE = 65536; // 64KB

/**
 * @brief Represents permissions for a file or directory.
 * @details Uses standard Unix-like rwx bits for owner (miniOS doesn't have users/groups yet, so owner is implicit).
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
 * @brief File system entry (file or directory).
 * @details Stores metadata and data for each entry in the RAMFS.
 */
struct FileEntry {
    std::array<char, MAX_PATH_LENGTH> path; ///< Null-terminated file/directory path.
    bool is_directory = false;              ///< True if this entry is a directory, false if it's a file.
    Permissions permissions = Permissions::READ_WRITE_EXECUTE; ///< Permissions (default rwx for owner).
    std::vector<uint8_t> data;              ///< File data content (empty for directories).
    size_t size = 0;                        ///< Actual size of the file data in bytes.
    // For directories, children could be represented by storing indices to the files_ array,
    // or by iterating through all files and checking their parent path.
    // For simplicity, we can infer parent/child from paths, or add explicit parent_idx.
    // std::vector<size_t> children_indices; // Example if explicitly storing children
};

/**
 * @brief RAM File System class.
 * @details Provides an in-memory file system with basic file and directory operations.
 * Not persistent across reboots.
 */
class FileSystem {
public:
    /**
     * @brief Default constructor. Initializes the file system.
     */
    FileSystem();

    /**
     * @brief Creates a new file or directory.
     * @param path The absolute path for the new file or directory (e.g., "/mydir/myfile.txt").
     * @param is_directory Set to true to create a directory, false to create a file.
     * @return True if the file or directory was successfully created, false otherwise (e.g., path exists, parent not found, max files reached).
     */
    bool create_file(std::string_view path, bool is_directory = false);

    /**
     * @brief Writes data to an existing file.
     * @details Overwrites existing content. If the new size is smaller, the file is truncated.
     * If larger, it's expanded.
     * @param path The absolute path of the file to write to.
     * @param data Pointer to the data buffer to write.
     * @param size Number of bytes to write from the data buffer.
     * @return True if the write was successful, false otherwise (e.g., file not found, not a file, no write permission, size exceeds MAX_FILE_SIZE).
     */
    bool write_file(std::string_view path, const void* data, size_t size);

    /**
     * @brief Reads data from a file into a pre-allocated buffer.
     * @param path The absolute path of the file to read from.
     * @param[out] buffer Pointer to the buffer where data will be copied.
     * @param max_size The maximum number of bytes to read (size of the buffer).
     * @return The number of bytes actually read and copied into the buffer. Returns 0 on failure (e.g., file not found, not a file, no read permission, buffer too small for any data).
     */
    size_t read_file(std::string_view path, void* buffer, size_t max_size) const;

    /**
     * @brief Reads the entire content of a file into a std::string.
     * @param path The absolute path of the file to read from.
     * @param[out] content The std::string to store the file's content.
     * @return True if the file was read successfully, false otherwise (e.g., file not found, not a file, no read permission).
     */
    bool read_file(std::string_view path, std::string& content) const;

    /**
     * @brief Deletes a file or an empty directory.
     * @param path The absolute path of the file or directory to delete.
     * @return True if the deletion was successful, false otherwise (e.g., not found, directory not empty).
     */
    bool delete_file(std::string_view path);

    /**
     * @brief Checks if a file or directory exists at the given path.
     * @param path The absolute path to check.
     * @return True if an entry (file or directory) exists at the path, false otherwise.
     */
    bool file_exists(std::string_view path) const;

    /**
     * @brief Lists the contents of a directory to the UART console.
     * @param path The absolute path of the directory to list.
     * @param uart_ops Pointer to the UART driver operations for printing the listing.
     * @return True if the listing was successful (directory exists and is readable), false otherwise.
     */
    bool list_files(std::string_view path, kernel::hal::UARTDriverOps* uart_ops) const;

    /**
     * @brief Sets the permissions for a file or directory.
     * @param path The absolute path of the file or directory.
     * @param perms The new permissions to set.
     * @return True if permissions were successfully set, false if the file/directory was not found.
     */
    bool set_permissions(std::string_view path, Permissions perms);

private:
    std::array<FileEntry, MAX_FILES> entries_; ///< Storage for all file system entries.
    size_t entry_count_ = 0;                   ///< Current number of active entries (files and directories).
    mutable kernel::Spinlock fs_lock_;         ///< Spinlock to protect concurrent access to file system structures.

    /**
     * @brief Finds a file system entry by its path.
     * @note This is a const method but might lock fs_lock_ internally if ScopedLock is used for read access.
     * @param path The absolute path of the entry to find.
     * @return An std::optional containing the index of the entry in the `entries_` array if found, otherwise std::nullopt.
     */
    std::optional<size_t> find_entry_index(std::string_view path) const;

    /**
     * @brief Gets a pointer to a file system entry by its path.
     * @note This method is not const because it returns a non-const pointer.
     * Caller must handle locking if modifying the entry.
     * @param path The absolute path of the entry.
     * @return Pointer to the FileEntry if found, nullptr otherwise.
     */
    FileEntry* find_entry(std::string_view path); // Non-const version
    const FileEntry* find_entry(std::string_view path) const; // Const version

    /**
     * @brief Validates a path string.
     * @details Checks for basic validity constraints like starting with '/', max length, no empty segments.
     * @param path The path to validate.
     * @return True if the path format is considered valid, false otherwise.
     */
    bool is_valid_path_format(std::string_view path) const;

    /**
     * @brief Helper to normalize a path (e.g., remove trailing slashes unless it's the root).
     * @param path The path to normalize.
     * @return A normalized std::string_view of the path.
     */
    std::string_view normalize_path(std::string_view path) const;

    /**
     * @brief Gets the parent path of a given path.
     * @param path The full path.
     * @return The parent path, or an empty string_view if it's the root or invalid.
     */
    std::string_view get_parent_path(std::string_view path) const;
};

extern FileSystem g_file_system; ///< Global file system instance.

} // namespace fs

#endif // FS_HPP
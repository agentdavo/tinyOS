// SPDX-License-Identifier: MIT OR Apache-2.0
/**
 * @file fs.cpp
 * @brief RAM File System (RAMFS) subsystem implementation for miniOS v1.7.
 * @details
 * Implements an in-memory file system with support for file/directory creation, read/write
 * operations, and permissions. Enhanced in v1.7 with increased file size limit (64KB from 16KB),
 * improved error handling, clearer diagnostics, and modern C++20 practices, retaining all v1.6
 * functionality including directory support and CLI integration.
 *
 * C++20 features:
 * - std::string_view for path handling
 * - std::span for data buffers
 * - std::optional for safer returns
 *
 * @version 1.7
 * @see fs.hpp, miniOS.hpp, util.hpp, cli.hpp
 */

#include "fs.hpp"
#include "util.hpp" // For kernel::util::safe_strcpy, kernel::util::memcpy
#include <cstring>   // For std::strcmp, std::snprintf (used by list_files)
#include <cstdio>    // For std::snprintf
#include <algorithm> // For std::min, std::remove_if

// Global file system instance definition
namespace fs {
FileSystem g_file_system;
} // namespace fs


namespace fs {

FileSystem::FileSystem() : entry_count_(0) {
    // Initialize with a root directory
    if (entry_count_ < MAX_FILES) {
        FileEntry& root = entries_[entry_count_++];
        kernel::util::safe_strcpy(root.path.data(), "/", MAX_PATH_LENGTH);
        root.is_directory = true;
        root.permissions = Permissions::READ_WRITE_EXECUTE; // Root is typically fully accessible
        root.size = 0;
    }
    // Else: MAX_FILES is too small for even the root, critical error (assert or panic in real system)
}

std::string_view FileSystem::normalize_path(std::string_view path) const {
    if (path.length() > 1 && path.back() == '/') {
        return path.substr(0, path.length() - 1);
    }
    return path;
}

bool FileSystem::is_valid_path_format(std::string_view path) const {
    if (path.empty() || path.length() >= MAX_PATH_LENGTH || path[0] != '/') {
        return false;
    }
    // Check for "//" (empty segment) but allow "/" at the end for directories if not normalized yet
    for (size_t i = 0; i < path.length() - 1; ++i) {
        if (path[i] == '/' && path[i+1] == '/') {
            return false;
        }
    }
    // Check for forbidden characters (e.g., null byte within path, though string_view handles this)
    if (path.find('\0') != std::string_view::npos && path.find('\0') < path.length()-1) { // null byte not at end
         return false;
    }
    return true;
}

std::optional<size_t> FileSystem::find_entry_index(std::string_view path) const {
    kernel::ScopedISRLock lock(const_cast<kernel::Spinlock&>(fs_lock_)); // Protect read access
    std::string_view normalized_path = normalize_path(path);
    if (!is_valid_path_format(normalized_path)) { // Validate after normalization
        return std::nullopt;
    }

    // Create a temporary C-style string for strcmp, as FileEntry.path is char array
    std::string path_cstr(normalized_path); 

    for (size_t i = 0; i < entry_count_; ++i) {
        // entries_[i].path is already null-terminated due to safe_strcpy and MAX_PATH_LENGTH init
        if (std::strcmp(entries_[i].path.data(), path_cstr.c_str()) == 0) {
            return i;
        }
    }
    return std::nullopt;
}

FileEntry* FileSystem::find_entry(std::string_view path) {
    // Non-const version, lock is handled by caller or implicitly if part of larger op
    std::optional<size_t> index = find_entry_index(path);
    if (index) {
        return &entries_[*index];
    }
    return nullptr;
}

const FileEntry* FileSystem::find_entry(std::string_view path) const {
    // Const version
    std::optional<size_t> index = find_entry_index(path);
    if (index) {
        return &entries_[*index];
    }
    return nullptr;
}


std::string_view FileSystem::get_parent_path(std::string_view path) const {
    std::string_view norm_path = normalize_path(path);
    if (norm_path.empty() || norm_path == "/") {
        return ""; // Root has no parent string, or handle as special case "/"
    }
    size_t last_slash = norm_path.rfind('/');
    if (last_slash == std::string_view::npos) { // Should not happen for valid absolute paths
        return "";
    }
    if (last_slash == 0) { // Parent is root "/"
        return "/";
    }
    return norm_path.substr(0, last_slash);
}


bool FileSystem::create_file(std::string_view path, bool is_directory) {
    kernel::ScopedISRLock lock(fs_lock_);
    std::string_view normalized_path = normalize_path(path);

    if (entry_count_ >= MAX_FILES || !is_valid_path_format(normalized_path)) {
        return false;
    }
    if (find_entry_index(normalized_path).has_value()) { // Check if already exists
        return false; // Entry already exists
    }

    // Check if parent directory exists
    if (normalized_path != "/") { // Not creating the root itself
        std::string_view parent_sv = get_parent_path(normalized_path);
        std::optional<size_t> parent_idx_opt = find_entry_index(parent_sv);
        if (!parent_idx_opt || !entries_[*parent_idx_opt].is_directory) {
            return false; // Parent directory does not exist or is not a directory
        }
        // If we were explicitly managing children lists:
        // entries_[*parent_idx_opt].children_indices.push_back(entry_count_);
    }


    FileEntry& new_entry = entries_[entry_count_];
    // Use safe_strcpy for char array
    std::string path_str(normalized_path); // Create std::string to get .c_str()
    if (!kernel::util::safe_strcpy(new_entry.path.data(), path_str.c_str(), MAX_PATH_LENGTH)) {
        return false; // Path too long, should have been caught by is_valid_path_format
    }
    
    new_entry.is_directory = is_directory;
    new_entry.permissions = Permissions::READ_WRITE_EXECUTE; // Default permissions
    new_entry.data.clear(); // Ensure data is clear for new files/dirs
    new_entry.size = 0;

    if (!is_directory) {
        // new_entry.data.reserve(1024); // Optional: pre-reserve some capacity for files
    }
    
    entry_count_++;
    return true;
}

bool FileSystem::write_file(std::string_view path, const void* data, size_t size) {
    if (!data || size > MAX_FILE_SIZE) { // Allow size 0 for truncation
        return false;
    }
    kernel::ScopedISRLock lock(fs_lock_);
    std::optional<size_t> entry_idx_opt = find_entry_index(path);
    if (!entry_idx_opt) {
        return false; // File not found
    }
    
    FileEntry& entry = entries_[*entry_idx_opt];
    if (entry.is_directory) {
        return false; // Cannot write to a directory
    }
    if (!(static_cast<uint8_t>(entry.permissions) & static_cast<uint8_t>(Permissions::WRITE))) {
        return false; // No write permission
    }

    try {
        entry.data.resize(size); // Resize vector to the new size
        if (size > 0) {
            kernel::util::memcpy(entry.data.data(), data, size);
        }
        entry.size = size;
    } catch (const std::bad_alloc& e) {
        // Handle potential memory allocation failure if vector resize fails
        // In a bare-metal RTOS, this might panic or return error.
        return false;
    }
    return true;
}

size_t FileSystem::read_file(std::string_view path, void* buffer, size_t max_size) const {
    if (!buffer || max_size == 0) {
        return 0;
    }
    kernel::ScopedISRLock lock(const_cast<kernel::Spinlock&>(fs_lock_));
    std::optional<size_t> entry_idx_opt = find_entry_index(path);

    if (!entry_idx_opt) {
        return 0; // File not found
    }

    const FileEntry& entry = entries_[*entry_idx_opt];
    if (entry.is_directory) {
        return 0; // Cannot read a directory like a file
    }
    if (!(static_cast<uint8_t>(entry.permissions) & static_cast<uint8_t>(Permissions::READ))) {
        return 0; // No read permission
    }

    size_t bytes_to_read = std::min(entry.size, max_size);
    if (bytes_to_read > 0) {
        kernel::util::memcpy(buffer, entry.data.data(), bytes_to_read);
    }
    return bytes_to_read;
}

bool FileSystem::read_file(std::string_view path, std::string& content) const {
    kernel::ScopedISRLock lock(const_cast<kernel::Spinlock&>(fs_lock_));
    std::optional<size_t> entry_idx_opt = find_entry_index(path);
    content.clear();

    if (!entry_idx_opt) {
        return false; // File not found
    }

    const FileEntry& entry = entries_[*entry_idx_opt];
    if (entry.is_directory) {
        return false; // Cannot read a directory's "content" this way
    }
    if (!(static_cast<uint8_t>(entry.permissions) & static_cast<uint8_t>(Permissions::READ))) {
        return false; // No read permission
    }

    try {
        if (entry.size > 0) {
            content.assign(reinterpret_cast<const char*>(entry.data.data()), entry.size);
        }
    } catch (const std::bad_alloc& e) {
        return false; // String assignment failed
    }
    return true;
}

bool FileSystem::delete_file(std::string_view path) {
    kernel::ScopedISRLock lock(fs_lock_);
    std::string_view normalized_path = normalize_path(path);
    if (normalized_path == "/") {
        return false; // Cannot delete root
    }

    auto it = std::find_if(entries_.begin(), entries_.begin() + entry_count_,
                           [&normalized_path](const FileEntry& entry) {
                               // Compare normalized path with entry's path
                               return std::string_view(entry.path.data()) == normalized_path;
                           });

    if (it == entries_.begin() + entry_count_) {
        return false; // Not found
    }

    if (it->is_directory) {
        // Check if directory is empty by seeing if any other entry lists it as a parent
        // This is a simplified check. A robust check would iterate its explicit children list if maintained.
        for (size_t i = 0; i < entry_count_; ++i) {
            if (i == static_cast<size_t>(std::distance(entries_.begin(), it))) continue; // Skip itself
            if (get_parent_path(entries_[i].path.data()) == normalized_path) {
                return false; // Directory not empty
            }
        }
    }

    // "Remove" by shifting subsequent elements if not the last one
    // This is inefficient for arrays; std::vector with erase or a linked list FS structure would be better.
    // For std::array, we can mark as unused or compact. Compacting:
    if (it != entries_.begin() + entry_count_ - 1) { // If not the last element
         std::move(it + 1, entries_.begin() + entry_count_, it);
    }
    entry_count_--;
    // The "moved-from" last element is now garbage, but entry_count_ prevents access.
    // Could optionally clear it: entries_[entry_count_] = FileEntry{};
    return true;
}


bool FileSystem::file_exists(std::string_view path) const {
    // find_entry_index already handles locking and path validation
    return find_entry_index(path).has_value();
}

bool FileSystem::list_files(std::string_view path, kernel::hal::UARTDriverOps* uart_ops) const {
    if (!uart_ops) return false;
    kernel::ScopedISRLock lock(const_cast<kernel::Spinlock&>(fs_lock_));
    std::string_view normalized_path = normalize_path(path);

    std::optional<size_t> dir_idx_opt = find_entry_index(normalized_path);
    if (!dir_idx_opt || !entries_[*dir_idx_opt].is_directory) {
        uart_ops->puts("Error: Not a directory or path not found: ");
        std::string path_str(normalized_path); uart_ops->puts(path_str.c_str()); // Ensure null-terminated
        uart_ops->putc('\n');
        return false;
    }

    uart_ops->puts("Contents of ");
    std::string path_str(normalized_path); uart_ops->puts(path_str.c_str());
    uart_ops->puts(":\n");

    bool found_children = false;
    for (size_t i = 0; i < entry_count_; ++i) {
        // Check if 'path' is the parent of entries_[i].path
        // Avoid comparing if entries_[i].path is shorter or equal to path
        std::string_view entry_path_sv(entries_[i].path.data());
        std::string_view parent_of_entry = get_parent_path(entry_path_sv);

        if (parent_of_entry == normalized_path) {
            found_children = true;
            uart_ops->puts("  ");
            // Get just the filename/dirname part
            size_t last_slash = entry_path_sv.rfind('/');
            std::string_view name_part = (last_slash == std::string_view::npos) ? entry_path_sv : entry_path_sv.substr(last_slash + 1);
            std::string name_part_str(name_part); uart_ops->puts(name_part_str.c_str());

            if (entries_[i].is_directory) {
                uart_ops->puts("/"); // Indicate directory
            }

            char buf[40]; // For size and permissions
            if (!entries_[i].is_directory) {
                std::snprintf(buf, sizeof(buf), " [%zu bytes]", entries_[i].size);
                uart_ops->puts(buf);
            }

            uint8_t p = static_cast<uint8_t>(entries_[i].permissions);
            std::snprintf(buf, sizeof(buf), " (perm: %c%c%c)",
                          (p & static_cast<uint8_t>(Permissions::READ)) ? 'r' : '-',
                          (p & static_cast<uint8_t>(Permissions::WRITE)) ? 'w' : '-',
                          (p & static_cast<uint8_t>(Permissions::EXECUTE)) ? 'x' : '-');
            uart_ops->puts(buf);
            uart_ops->putc('\n');
        }
    }

    if (!found_children) {
        uart_ops->puts("  (empty)\n");
    }
    return true;
}

bool FileSystem::set_permissions(std::string_view path, Permissions perms) {
    kernel::ScopedISRLock lock(fs_lock_);
    std::optional<size_t> entry_idx_opt = find_entry_index(path);
    if (!entry_idx_opt) {
        return false; // File not found
    }
    entries_[*entry_idx_opt].permissions = perms;
    return true;
}

} // namespace fs
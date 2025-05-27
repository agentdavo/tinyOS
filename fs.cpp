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
#include <algorithm> // For std::min, std::remove_if, std::move

// Global file system instance definition is in miniOS.cpp (kernel::g_file_system)

namespace fs {

FileSystem::FileSystem() : entry_count_(0) {
    // Initialize with a root directory. This should be the first entry.
    if (entry_count_ < MAX_FILES) {
        FileEntry& root = entries_[entry_count_]; // entry_count_ is 0
        kernel::util::safe_strcpy(root.path.data(), "/", MAX_PATH_LENGTH);
        root.is_directory = true;
        root.permissions = Permissions::READ_WRITE_EXECUTE; 
        root.size = 0;
        entry_count_++; 
    }
}

std::string_view FileSystem::normalize_path(std::string_view path) const {
    while (path.length() > 1 && path.back() == '/') {
        path.remove_suffix(1);
    }
    return path;
}

bool FileSystem::is_valid_path_format(std::string_view path) const {
    if (path.empty() || path.length() >= MAX_PATH_LENGTH || path[0] != '/') {
        return false;
    }
    for (size_t i = 0; i < path.length() -1; ++i) {
        if (path[i] == '/' && path[i+1] == '/') { 
            return false;
        }
    }
    size_t null_pos = path.find('\0');
    if (null_pos != std::string_view::npos && null_pos < path.length()) { // null byte within, not just as terminator from C-str
         return false;
    }
    return true;
}

std::optional<size_t> FileSystem::find_entry_index(std::string_view path) const {
    kernel::ScopedLock lock(const_cast<kernel::Spinlock&>(fs_lock_)); 
    std::string_view normalized_path_sv = normalize_path(path);

    if (!is_valid_path_format(normalized_path_sv)) { 
        return std::nullopt;
    }
    
    for (size_t i = 0; i < entry_count_; ++i) {
        // Create string_view from the fixed-size char array, respecting its actual length.
        // Find the null terminator to determine the actual length of the stored path.
        const char* entry_c_path = entries_[i].path.data();
        std::string_view entry_path_sv(entry_c_path, kernel::util::strlen(entry_c_path));
        if (entry_path_sv == normalized_path_sv) {
            return i;
        }
    }
    return std::nullopt;
}

FileEntry* FileSystem::find_entry(std::string_view path) {
    std::optional<size_t> index = find_entry_index(path); 
    if (index) {
        return &entries_[*index];
    }
    return nullptr;
}

const FileEntry* FileSystem::find_entry(std::string_view path) const {
    std::optional<size_t> index = find_entry_index(path); 
    if (index) {
        return &entries_[*index];
    }
    return nullptr;
}


std::string_view FileSystem::get_parent_path(std::string_view path) const {
    std::string_view norm_path = normalize_path(path);
    if (norm_path.empty() || norm_path == "/") {
        return {}; 
    }
    size_t last_slash = norm_path.rfind('/');
    if (last_slash == std::string_view::npos) { 
        return {}; 
    }
    if (last_slash == 0) { 
        return "/";
    }
    return norm_path.substr(0, last_slash);
}


bool FileSystem::create_file(std::string_view path, bool is_directory) {
    kernel::ScopedLock lock(fs_lock_);
    std::string_view normalized_path = normalize_path(path);

    if (entry_count_ >= MAX_FILES || !is_valid_path_format(normalized_path) || normalized_path.length() >= MAX_PATH_LENGTH) {
        return false;
    }
    
    // Check if entry already exists by comparing normalized paths
    // find_entry_index already normalizes, so we can use it directly if we handle its lock properly
    // For create, we hold the lock for the whole operation.
    std::string temp_path_str(normalized_path); // For strcmp if needed by find_entry_index's internal logic
    for(size_t i=0; i < entry_count_; ++i) {
        if (std::string_view(entries_[i].path.data()) == normalized_path) {
            return false; // Entry already exists
        }
    }


    if (normalized_path != "/") { 
        std::string_view parent_sv = get_parent_path(normalized_path);
        if (parent_sv.empty() && normalized_path != "/") return false; 

        bool parent_found = false;
        if (!parent_sv.empty()) { // Only check parent if not creating in root (parent_sv would be "/" for /file)
            std::string temp_parent_str(parent_sv);
            for(size_t i=0; i < entry_count_; ++i) {
                 if (std::string_view(entries_[i].path.data()) == parent_sv) {
                    if (!entries_[i].is_directory) return false; 
                    parent_found = true;
                    break;
                 }
            }
            if (!parent_found && parent_sv != "/") return false; // Parent (not root) must exist
        }
    }

    FileEntry& new_entry = entries_[entry_count_];
    std::string path_str(normalized_path); 
    if (!kernel::util::safe_strcpy(new_entry.path.data(), path_str.c_str(), MAX_PATH_LENGTH)) {
        return false; 
    }
    
    new_entry.is_directory = is_directory;
    new_entry.permissions = Permissions::READ_WRITE_EXECUTE; 
    new_entry.data.clear(); 
    new_entry.size = 0;
    
    entry_count_++;
    return true;
}

bool FileSystem::write_file(std::string_view path, const void* data, size_t size) {
    if (!data && size > 0) return false; 
    if (size > MAX_FILE_SIZE) { 
        return false;
    }
    kernel::ScopedLock lock(fs_lock_);
    FileEntry* entry_ptr = find_entry(path); 

    if (!entry_ptr) {
        return false; 
    }
    
    if (entry_ptr->is_directory) {
        return false; 
    }
    if (!(static_cast<uint8_t>(entry_ptr->permissions) & static_cast<uint8_t>(Permissions::WRITE))) {
        return false; 
    }

    try {
        entry_ptr->data.assign(static_cast<const uint8_t*>(data), static_cast<const uint8_t*>(data) + size); 
        entry_ptr->size = size;
    } catch (const std::bad_alloc&) {
        return false;
    }
    return true;
}

size_t FileSystem::read_file(std::string_view path, void* buffer, size_t max_size) const {
    if (!buffer || max_size == 0) {
        return 0;
    }
    const FileEntry* entry_ptr = find_entry(path);

    if (!entry_ptr) {
        return 0; 
    }

    if (entry_ptr->is_directory) {
        return 0; 
    }
    if (!(static_cast<uint8_t>(entry_ptr->permissions) & static_cast<uint8_t>(Permissions::READ))) {
        return 0; 
    }

    size_t bytes_to_read = std::min(entry_ptr->size, max_size);
    if (bytes_to_read > 0) {
        kernel::util::memcpy(buffer, entry_ptr->data.data(), bytes_to_read);
    }
    return bytes_to_read;
}

bool FileSystem::read_file(std::string_view path, std::string& content) const {
    const FileEntry* entry_ptr = find_entry(path);
    content.clear();

    if (!entry_ptr) {
        return false; 
    }

    if (entry_ptr->is_directory) {
        return false; 
    }
    if (!(static_cast<uint8_t>(entry_ptr->permissions) & static_cast<uint8_t>(Permissions::READ))) {
        return false; 
    }

    try {
        if (entry_ptr->size > 0) {
            content.assign(reinterpret_cast<const char*>(entry_ptr->data.data()), entry_ptr->size);
        }
    } catch (const std::bad_alloc&) {
        return false; 
    }
    return true;
}

bool FileSystem::delete_file(std::string_view path) {
    kernel::ScopedLock lock(fs_lock_);
    std::string_view normalized_path = normalize_path(path);
    if (normalized_path == "/") {
        return false; 
    }

    auto it = std::find_if(entries_.begin(), entries_.begin() + entry_count_,
                           [&normalized_path](const FileEntry& entry) {
                               const char* entry_c_path = entry.path.data();
                               return std::string_view(entry_c_path, kernel::util::strlen(entry_c_path)) == normalized_path;
                           });

    if (it == entries_.begin() + entry_count_) {
        return false; 
    }

    if (it->is_directory) {
        for (size_t i = 0; i < entry_count_; ++i) {
            if (std::distance(entries_.begin(), it) == static_cast<ptrdiff_t>(i)) continue; 
            std::string_view current_entry_path(entries_[i].path.data(), kernel::util::strlen(entries_[i].path.data()));
            if (get_parent_path(current_entry_path) == normalized_path) {
                return false; 
            }
        }
    }

    if (std::distance(entries_.begin(), it) < static_cast<ptrdiff_t>(entry_count_ -1) ) {
         *it = std::move(entries_[entry_count_ - 1]);
    }
    
    entry_count_--;
    return true;
}


bool FileSystem::file_exists(std::string_view path) const {
    return find_entry_index(path).has_value();
}

bool FileSystem::list_files(std::string_view path, kernel::hal::UARTDriverOps* uart_ops) const {
    if (!uart_ops) return false;
    
    std::string_view normalized_path_to_list = normalize_path(path);
    std::optional<size_t> dir_idx_opt = find_entry_index(normalized_path_to_list); 

    if (!dir_idx_opt || !entries_[*dir_idx_opt].is_directory) {
        uart_ops->puts("Error: Not a directory or path not found: ");
        std::string path_str_temp(path); uart_ops->puts(path_str_temp.c_str()); 
        uart_ops->putc('\n');
        return false;
    }

    uart_ops->puts("Contents of ");
    std::string path_str_temp(normalized_path_to_list); uart_ops->puts(path_str_temp.c_str());
    uart_ops->puts(":\n");

    bool found_children = false;
    kernel::ScopedLock lock(const_cast<kernel::Spinlock&>(fs_lock_));
    for (size_t i = 0; i < entry_count_; ++i) {
        const char* current_entry_c_path = entries_[i].path.data();
        std::string_view current_entry_path_sv(current_entry_c_path, kernel::util::strlen(current_entry_c_path));
        std::string_view parent_of_current_entry = get_parent_path(current_entry_path_sv);

        if (parent_of_current_entry == normalized_path_to_list) {
            found_children = true;
            uart_ops->puts("  ");
            
            std::string_view name_part = current_entry_path_sv;
            // Get only the last component of the path for display
            size_t last_slash_in_entry = current_entry_path_sv.rfind('/');
            if (last_slash_in_entry != std::string_view::npos && last_slash_in_entry + 1 < current_entry_path_sv.length()) {
                name_part = current_entry_path_sv.substr(last_slash_in_entry + 1);
            } else if (last_slash_in_entry == 0 && current_entry_path_sv.length() > 1) { // e.g. /file
                 name_part = current_entry_path_sv.substr(1);
            }
            // else name_part remains the full path if it's just "/" or malformed

            std::string name_part_str(name_part); uart_ops->puts(name_part_str.c_str());

            if (entries_[i].is_directory) {
                uart_ops->puts("/"); 
            }

            char buf[40]; 
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
    kernel::ScopedLock lock(fs_lock_);
    FileEntry* entry_ptr = find_entry(path); 
    if (!entry_ptr) {
        return false; 
    }
    entry_ptr->permissions = perms;
    return true;
}

} // namespace fs
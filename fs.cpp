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
#include "util.hpp"
#include <cstring>
#include <algorithm>

namespace fs {

FileSystem g_file_system;

bool FileSystem::is_valid_path(std::string_view path) const {
    if (path.empty() || path.size() >= MAX_PATH_LENGTH || path[0] != '/') return false;
    if (path.find("//") != std::string_view::npos || path.find('\0') != std::string_view::npos) return false;
    return true;
}

std::optional<size_t> FileSystem::find_file(std::string_view path) const {
    if (!is_valid_path(path)) return std::nullopt;
    for (size_t i = 0; i < file_count_; ++i) {
        if (util::strcmp(files_[i].path, path.data()) == 0) {
            return i;
        }
    }
    return std::nullopt;
}

bool FileSystem::create_file(std::string_view path, bool is_directory) {
    if (file_count_ >= MAX_FILES || !is_valid_path(path)) return false;
    if (find_file(path).has_value()) return false;

    // Check parent directory exists
    size_t last_slash = path.rfind('/', path.size() - 2);
    if (last_slash != 0 && last_slash != std::string_view::npos) {
        std::string_view parent_path = path.substr(0, last_slash == 0 ? 1 : last_slash);
        auto parent_idx = find_file(parent_path);
        if (!parent_idx.has_value() || !files_[*parent_idx].is_directory) return false;
        files_[*parent_idx].children.push_back(file_count_);
    }

    FileEntry& entry = files_[file_count_];
    if (!util::safe_strcpy(entry.path, path.data(), MAX_PATH_LENGTH)) return false;
    entry.is_directory = is_directory;
    entry.permissions = 0b00000111; // Default: rwx
    if (!is_directory) {
        entry.data.reserve(1024); // Initial capacity
    }
    ++file_count_;
    return true;
}

bool FileSystem::write_file(std::string_view path, const void* data, size_t size) {
    if (!data || size == 0 || size > MAX_FILE_SIZE) return false;
    auto idx = find_file(path);
    if (!idx.has_value() || files_[*idx].is_directory || !(files_[*idx].permissions & 0b00000010)) {
        return false; // Not found, is directory, or not writable
    }
    FileEntry& entry = files_[*idx];
    entry.data.resize(size);
    util::memcpy(entry.data.data(), data, size);
    return true;
}

size_t FileSystem::read_file(std::string_view path, void* buffer, size_t max_size) const {
    if (!buffer || max_size == 0) return 0;
    auto idx = find_file(path);
    if (!idx.has_value() || files_[*idx].is_directory || !(files_[*idx].permissions & 0b00000100)) {
        return 0; // Not found, is directory, or not readable
    }
    const FileEntry& entry = files_[*idx];
    size_t bytes_to_read = std::min(entry.data.size(), max_size);
    util::memcpy(buffer, entry.data.data(), bytes_to_read);
    return bytes_to_read;
}

bool FileSystem::file_exists(std::string_view path) const {
    return find_file(path).has_value();
}

bool FileSystem::list_files(std::string_view path, kernel::hal::UARTDriverOps* uart_ops) const {
    if (!uart_ops) return false;
    auto idx = find_file(path);
    if (!idx.has_value() || !files_[*idx].is_directory) {
        uart_ops->puts("Not a directory: ");
        uart_ops->puts(path.data());
        uart_ops->puts("\n");
        return false;
    }
    const FileEntry& dir = files_[*idx];
    if (dir.children.empty()) {
        uart_ops->puts("Directory empty: ");
        uart_ops->puts(path.data());
        uart_ops->puts("\n");
        return true;
    }
    uart_ops->puts("Contents of ");
    uart_ops->puts(path.data());
    uart_ops->puts(":\n");
    for (size_t child_idx : dir.children) {
        if (child_idx < file_count_) {
            const FileEntry& child = files_[child_idx];
            uart_ops->puts("  ");
            uart_ops->puts(child.path);
            uart_ops->puts(child.is_directory ? " [DIR]" : "");
            char buf[32];
            std::snprintf(buf, sizeof(buf), " %u bytes", static_cast<unsigned>(child.data.size()));
            if (!child.is_directory) {
                uart_ops->puts(buf);
            }
            uart_ops->puts(" (perm: ");
            uart_ops->putc((child.permissions & 0b00000100) ? 'r' : '-');
            uart_ops->putc((child.permissions & 0b00000010) ? 'w' : '-');
            uart_ops->putc((child.permissions & 0b00000001) ? 'x' : '-');
            uart_ops->puts(")\n");
        }
    }
    return true;
}

bool FileSystem::set_permissions(std::string_view path, uint8_t permissions) {
    auto idx = find_file(path);
    if (!idx.has_value()) return false;
    files_[*idx].permissions = permissions & 0b00000111; // Mask to rwx bits
    return true;
}

} // namespace fs
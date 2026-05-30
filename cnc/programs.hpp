// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "../core.hpp"
#include "../render/gles1.hpp"
#include <array>
#include <cstddef>

namespace cnc::programs {

constexpr size_t MAX_PROGRAMS = 8;
constexpr size_t MAX_PROGRAM_SIZE = 2048;
constexpr size_t MAX_PREVIEW_POINTS = 512;
constexpr size_t MAX_CHANNELS = 2;

struct Preview {
    std::array<render::gles1::Vec3f, MAX_PREVIEW_POINTS> points{};
    size_t point_count = 0;
    size_t motion_blocks = 0;
    size_t parsed_lines = 0;
    bool valid = false;
    bool truncated = false;
    render::gles1::Vec3f min{0.0f, 0.0f, 0.0f};
    render::gles1::Vec3f max{0.0f, 0.0f, 0.0f};
};

struct ProgramEntry {
    char name[64]{};
    char path[96]{};
    std::array<char, MAX_PROGRAM_SIZE> text{};
    size_t text_size = 0;
    bool loaded = false;
    Preview preview{};
};

struct ChannelProgramState {
    size_t selected = 0;
    size_t loaded = 0;
};

class Store {
public:
    Store();

    size_t count() const noexcept { return count_; }
    // Precondition: idx < count(). Defensively clamp to the array bound so a
    // stale/out-of-range index can't read past programs_[] (returns slot 0).
    const ProgramEntry& program(size_t idx) const noexcept {
        return programs_[idx < programs_.size() ? idx : 0];
    }
    size_t selected(size_t channel = 0) const noexcept;
    size_t loaded(size_t channel = 0) const noexcept;
    const ProgramEntry* selected_program(size_t channel = 0) const noexcept;
    ProgramEntry* selected_program(size_t channel = 0) noexcept;
    const ProgramEntry* loaded_program(size_t channel = 0) const noexcept;
    ProgramEntry* loaded_program(size_t channel = 0) noexcept;

    bool select(size_t idx) noexcept { return select(0, idx); }
    bool select(size_t channel, size_t idx) noexcept;
    bool find_by_name(const char* name, size_t& out_idx) const noexcept;
    bool write_program(const char* name, const char* text) noexcept;
    bool open_selected() noexcept;
    bool open_selected(size_t channel) noexcept;
    bool rebuild_preview(size_t idx) noexcept;
    size_t channel_count() const noexcept { return MAX_CHANNELS; }
    const ChannelProgramState& channel_state(size_t channel) const noexcept { return channels_[channel]; }

    // Walk the VFS for *.ngc / *.NGC entries and adopt them as program
    // slots, layered behind the boot-time seeds. Filesystem-backed entries
    // are loaded once per call (idempotent on the same path) and capped at
    // MAX_PROGRAMS so the operator never runs out of slots — extra files
    // on the SD card are silently skipped. Safe to call from the UI tick.
    size_t scan_filesystem() noexcept;

private:
    bool add_seed(const char* name, const char* text) noexcept;
    bool parse_preview(ProgramEntry& entry) noexcept;
    // Unlocked search for callers that already hold lock_ (e.g. write_program).
    bool find_by_name_locked(const char* name, size_t& out_idx) const noexcept;

    std::array<ProgramEntry, MAX_PROGRAMS> programs_{};
    size_t count_ = 0;
    std::array<ChannelProgramState, MAX_CHANNELS> channels_{};
    mutable kernel::core::Spinlock lock_;
};

extern Store g_store;

} // namespace cnc::programs

// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "machine/machine_registry.hpp"

#include <cstddef>
#include <cstdint>

namespace macros {

enum class StepKind : uint8_t {
    None = 0,
    Set,
    Delay,
    Wait,
    Move,
    PodSelect,
    ToolAssign,
    CaptureAxis,
    CaptureProbe,
    Compute,
    SetWorkOffset,
    SphereCalibrate,
    Alarm,
};

enum class CompareOp : uint8_t {
    Equal = 0,
    NotEqual,
    Less,
    LessEqual,
    Greater,
    GreaterEqual,
};

enum class ComputeOp : uint8_t {
    None = 0,
    Copy,
    Add,
    Subtract,
    Multiply,
    Divide,
    Average,
};

struct Step {
    StepKind kind = StepKind::None;
    char symbol[32]{};
    char source[32]{};
    char source_b[32]{};
    char pod_id[32]{};
    char axis[8]{};
    char message[64]{};
    machine::SymbolType value_type = machine::SymbolType::Bool;
    machine::SymbolValue value{};
    machine::SymbolValue alt_value{};
    CompareOp compare = CompareOp::Equal;
    ComputeOp compute = ComputeOp::None;
    uint32_t timeout_ms = 0;
    float scale = 1.0f;
    float bias = 0.0f;
    float direction = 0.0f;
    bool relative = true;
};

struct MacroDef {
    char id[32]{};
    char title[48]{};
    uint16_t mcode = 0;
    size_t first_step = 0;
    size_t step_count = 0;
};

class Runtime {
public:
    static constexpr size_t MAX_MACROS = 24;
    static constexpr size_t MAX_STEPS = 192;
    static constexpr size_t MAX_CHANNELS = 2;

    struct ChannelState {
        bool active = false;
        bool fault = false;
        size_t macro_index = 0;
        size_t step_index = 0;
        uint64_t wait_deadline_us = 0;
        char message[64]{};
    };

    bool load_tsv(const char* buf, size_t len) noexcept;
    size_t count() const noexcept { return macro_count_; }
    const MacroDef* macro(size_t idx) const noexcept { return idx < macro_count_ ? &macros_[idx] : nullptr; }

    bool select(size_t idx) noexcept;
    size_t selected() const noexcept { return selected_; }
    bool find_macro(const char* id, size_t& idx_out) const noexcept;
    bool start(size_t channel, size_t idx) noexcept;
    bool start_by_id(size_t channel, const char* id) noexcept;
    bool start_mcode(size_t channel, uint16_t mcode) noexcept;
    bool stop(size_t channel) noexcept;
    bool tick_channel(size_t channel) noexcept;
    void tick() noexcept;
    bool channel_done(size_t channel) const noexcept;
    bool channel_fault(size_t channel) const noexcept;
    const ChannelState& channel_state(size_t channel) const noexcept { return channels_[channel]; }

    static void thread_entry(void* arg);

private:
    bool apply_step(size_t channel, const Step& step) noexcept;
    bool compare_symbol(const Step& step) const noexcept;
    int resolve_axis(const char* axis_name) const noexcept;
    uint64_t now_us() const noexcept;

    MacroDef macros_[MAX_MACROS]{};
    Step steps_[MAX_STEPS]{};
    size_t macro_count_ = 0;
    size_t step_count_ = 0;
    size_t selected_ = 0;
    ChannelState channels_[MAX_CHANNELS]{};
};

extern Runtime g_runtime;

} // namespace macros

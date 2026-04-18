// SPDX-License-Identifier: MIT OR Apache-2.0
#pragma once

#include "machine/machine_registry.hpp"

#include <cstddef>
#include <cstdint>

namespace ladder {

enum class CompareOp : uint8_t {
    Equal = 0,
    NotEqual,
    Less,
    LessEqual,
    Greater,
    GreaterEqual,
};

struct Rung {
    char id[32]{};
    char lhs[32]{};
    char rhs_symbol[32]{};
    char out[32]{};
    machine::SymbolType type = machine::SymbolType::Bool;
    CompareOp compare = CompareOp::Equal;
    machine::SymbolValue rhs_value{};
    machine::SymbolValue true_value{};
    machine::SymbolValue false_value{};
};

class Runtime {
public:
    static constexpr size_t MAX_RUNGS = 48;

    bool load_tsv(const char* buf, size_t len) noexcept;
    void tick() noexcept;
    size_t count() const noexcept { return rung_count_; }
    const Rung* rung(size_t idx) const noexcept { return idx < rung_count_ ? &rungs_[idx] : nullptr; }

    static void thread_entry(void* arg);

private:
    bool evaluate(const Rung& rung) const noexcept;
    void drive_output(const Rung& rung, bool state) noexcept;

    Rung rungs_[MAX_RUNGS]{};
    size_t rung_count_ = 0;
};

extern Runtime g_runtime;

} // namespace ladder

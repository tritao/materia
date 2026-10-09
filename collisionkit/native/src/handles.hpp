#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace ck {

/**
 * Process-local objects behind generation-checked handles: a 12-bit
 * generation above a 20-bit slot. Slot 0 is never used, so a zero handle is
 * always invalid; a slot retires when its generation is exhausted instead of
 * wrapping around.
 */
template <typename T>
class Handles {
public:
    uint32_t store(std::unique_ptr<T> value) {
        std::lock_guard<std::mutex> lock(mutex_);
        uint32_t slot;
        if (!free_.empty()) {
            slot = free_.back();
            free_.pop_back();
        } else {
            if (entries_.size() > kSlotMask) return 0;
            slot = uint32_t(entries_.size());
            entries_.emplace_back();
        }
        entries_[slot].value = std::move(value);
        return (entries_[slot].generation << kSlotBits) | slot;
    }

    T *find(uint32_t handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        Entry *entry = locate(handle);
        return entry ? entry->value.get() : nullptr;
    }

    void release(uint32_t handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        Entry *entry = locate(handle);
        if (!entry) return;
        entry->value.reset();
        if (entry->generation < kGenerationMask) {
            ++entry->generation;
            free_.push_back(handle & kSlotMask);
        }
    }

private:
    static constexpr uint32_t kSlotBits = 20;
    static constexpr uint32_t kSlotMask = (1u << kSlotBits) - 1;
    static constexpr uint32_t kGenerationMask = 0xFFF;

    struct Entry {
        std::unique_ptr<T> value;
        uint32_t generation = 1;
    };

    Entry *locate(uint32_t handle) {
        const uint32_t slot = handle & kSlotMask;
        if (slot == 0 || slot >= entries_.size()) return nullptr;
        Entry &entry = entries_[slot];
        if (entry.generation != (handle >> kSlotBits) || !entry.value) return nullptr;
        return &entry;
    }

    std::mutex mutex_;
    std::vector<Entry> entries_ = std::vector<Entry>(1);
    std::vector<uint32_t> free_;
};

} // namespace ck

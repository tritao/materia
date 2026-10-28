// Internals shared by the C ABI translation units (api.cpp, ring/api_ring.cpp):
// generation-checked handle tables, the exception guard and argument helpers.
#pragma once

#include "stockkit.h"
#include "mesh.hpp"
#include "preview.hpp"
#include "profile.hpp"
#include "ring/rake.hpp"
#include "ring/ring.hpp"
#include "stock.hpp"
#include "tool.hpp"

#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <new>
#include <vector>

namespace stockkit::api {

using namespace stockkit;

// Handle layout: kind in the top 4 bits, then a 12-bit generation, then a
// 16-bit slot. Slot 0 is never used, so a zero handle is always invalid. A
// slot retires when its generation is exhausted instead of wrapping around.
constexpr uint32_t kKindTool = 1, kKindStock = 2, kKindSnapshot = 3, kKindMesh = 4;
constexpr uint32_t kGenerationMask = 0xFFF;

template <typename T, uint32_t Kind>
class Table {
public:
    uint32_t add(std::unique_ptr<T> value) {
        std::lock_guard<std::mutex> lock(mutex_);
        uint32_t slot = 0;
        if (!free_.empty()) {
            slot = free_.back();
            free_.pop_back();
        } else {
            if (entries_.size() >= 0xFFFF) return 0;
            entries_.push_back({});
            slot = static_cast<uint32_t>(entries_.size());
        }
        Entry &entry = entries_[slot - 1];
        entry.value = std::move(value);
        return (Kind << 28) | (entry.generation << 16) | slot;
    }

    T *get(uint32_t handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        Entry *entry = find(handle);
        return entry ? entry->value.get() : nullptr;
    }

    void remove(uint32_t handle) {
        std::unique_ptr<T> doomed;
        std::lock_guard<std::mutex> lock(mutex_);
        Entry *entry = find(handle);
        if (!entry) return;
        doomed = std::move(entry->value);
        if (entry->generation < kGenerationMask) {
            ++entry->generation;
            free_.push_back(handle & 0xFFFF);
        }
    }

private:
    struct Entry {
        uint32_t generation = 1;
        std::unique_ptr<T> value;
    };

    Entry *find(uint32_t handle) {
        uint32_t slot = handle & 0xFFFF;
        if ((handle >> 28) != Kind || slot == 0 || slot > entries_.size()) return nullptr;
        Entry &entry = entries_[slot - 1];
        if (entry.generation != ((handle >> 16) & kGenerationMask) || !entry.value) return nullptr;
        return &entry;
    }

    std::mutex mutex_;
    std::vector<Entry> entries_;
    std::vector<uint32_t> free_;
};

inline Table<Tool, kKindTool> &tools() {
    static Table<Tool, kKindTool> table;
    return table;
}

inline Table<Stock, kKindStock> &stocks() {
    static Table<Stock, kKindStock> table;
    return table;
}

inline Table<Stock::Snapshot, kKindSnapshot> &snapshots() {
    static Table<Stock::Snapshot, kKindSnapshot> table;
    return table;
}

inline Table<RakeTool, 7> &rake_tools() { static Table<RakeTool, 7> table; return table; }
inline Table<RingStock, 5> &ring_stocks() { static Table<RingStock, 5> table; return table; }
inline Table<RingSnapshot, 6> &ring_snapshots() { static Table<RingSnapshot, 6> table; return table; }

/** A preview mesh and the grid size its ray indices refer to. */
struct MeshEntry {
    PreviewMesh mesh;
    uint64_t rays[3] = {0, 0, 0}; // per grid, for colouring by ray
    uint32_t stock = 0;           // the stock it was built from, for colouring by deviation
    uint64_t unmatched = 0;
};

inline Table<MeshEntry, kKindMesh> &meshes() {
    static Table<MeshEntry, kKindMesh> table;
    return table;
}

template <typename T>
sk_result copy_bytes(const std::vector<T> &values, uint8_t *output, uint32_t *byte_capacity) {
    if (!byte_capacity) return SK_ERROR_INVALID_ARGUMENT;
    if (values.size() > 0xFFFFFFFFu / sizeof(T)) return SK_ERROR_LIMIT;
    uint32_t required = uint32_t(values.size() * sizeof(T));
    uint32_t capacity = *byte_capacity;
    *byte_capacity = required;
    if (capacity < required || (required != 0 && !output)) return SK_ERROR_LIMIT;
    if (required != 0) std::memcpy(output, values.data(), required);
    return SK_OK;
}

template <typename F>
sk_result guarded(F &&body) {
    try {
        return body();
    } catch (const std::bad_alloc &) {
        return SK_ERROR_OUT_OF_MEMORY;
    } catch (...) {
        return SK_ERROR_INTERNAL;
    }
}

inline bool finite(const double *values, int n) {
    for (int k = 0; k < n; ++k)
        if (!std::isfinite(values[k])) return false;
    return true;
}

} // namespace stockkit::api

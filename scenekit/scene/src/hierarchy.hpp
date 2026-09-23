#pragma once

#include "node_store.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace nkscene {

/** Slot-indexed parent and sibling links for the live node set. */
class HierarchyIndex {
public:
    explicit HierarchyIndex(const NodeStore &nodes) : nodes_(&nodes) {}

    NodeHandle parent_handle(NodeHandle handle) const noexcept {
        const auto *entry = find(handle);
        return entry ? entry->parent : NodeHandle{};
    }

    NodeId parent(NodeId id) const noexcept {
        return nodes_->id(parent_handle(nodes_->resolve(id)));
    }

    std::vector<NodeId> children(NodeId id) const {
        std::vector<NodeId> result;
        for_each_child(nodes_->resolve(id), [&](NodeId child, NodeHandle) {
            result.push_back(child);
        });
        return result;
    }

    template<class Fn>
    void for_each_child(NodeHandle parent, Fn &&fn) const {
        const auto *entry = find(parent);
        if (!entry)
            return;
        for (auto child = entry->first_child; child.valid();) {
            const auto *child_entry = find(child);
            const auto next = child_entry ? child_entry->next_sibling : NodeHandle{};
            const auto id = nodes_->id(child);
            if (id.valid())
                fn(id, child);
            child = next;
        }
    }

    bool is_descendant(NodeHandle id, NodeHandle ancestor) const noexcept {
        for (auto current = parent_handle(id); current.valid(); current = parent_handle(current)) {
            if (current == ancestor)
                return true;
        }
        return false;
    }

    bool is_descendant(NodeId id, NodeId ancestor) const noexcept {
        return is_descendant(nodes_->resolve(id), nodes_->resolve(ancestor));
    }

    bool reparent(NodeHandle id, NodeHandle new_parent) {
        if (!find(id) || (new_parent.valid() && !find(new_parent)) || id == new_parent ||
            is_descendant(new_parent, id))
            return false;
        const auto old_parent = parent_handle(id);
        if (old_parent == new_parent)
            return true;
        if (old_parent.valid())
            remove_child(old_parent, id);
        auto &entry = require(id);
        entry.parent = new_parent;
        if (new_parent.valid()) {
            auto &parent_entry = require(new_parent);
            const auto previous = parent_entry.last_child;
            if (previous.valid()) {
                require(previous).next_sibling = id;
                entry.prev_sibling = previous;
            } else {
                parent_entry.first_child = id;
            }
            parent_entry.last_child = id;
        }
        return true;
    }

    bool reparent(NodeId id, NodeId new_parent) {
        return reparent(nodes_->resolve(id), nodes_->resolve(new_parent));
    }

    void add(NodeHandle handle) {
        if (!handle.valid())
            throw std::invalid_argument("cannot add an invalid node handle to hierarchy");
        if (handle.slot >= entries_.size())
            entries_.resize(static_cast<std::size_t>(handle.slot) + 1);
        auto &entry = entries_[handle.slot];
        if (entry.generation != handle.generation)
            entry = Entry{handle.generation};
    }

    void add(NodeId id) { add(nodes_->resolve(id)); }

    void remove(NodeHandle handle) noexcept {
        auto *entry = find(handle);
        if (!entry)
            return;
        if (entry->parent.valid())
            remove_child(entry->parent, handle);

        auto child = entry->first_child;
        while (child.valid()) {
            auto *child_entry = find(child);
            const auto next = child_entry ? child_entry->next_sibling : NodeHandle{};
            if (child_entry) {
                child_entry->parent = {};
                child_entry->prev_sibling = {};
                child_entry->next_sibling = {};
            }
            child = next;
        }
        entry->first_child = {};
        entry->last_child = {};
        *entry = {};
    }

    void remove(NodeId id) noexcept { remove(nodes_->resolve(id)); }

private:
    struct Entry {
        std::uint32_t generation = 0;
        NodeHandle parent;
        NodeHandle first_child;
        NodeHandle last_child;
        NodeHandle next_sibling;
        NodeHandle prev_sibling;
    };

    Entry *find(NodeHandle handle) noexcept {
        if (!handle.valid() || handle.slot >= entries_.size())
            return nullptr;
        auto &entry = entries_[handle.slot];
        return entry.generation == handle.generation ? &entry : nullptr;
    }

    const Entry *find(NodeHandle handle) const noexcept {
        if (!handle.valid() || handle.slot >= entries_.size())
            return nullptr;
        const auto &entry = entries_[handle.slot];
        return entry.generation == handle.generation ? &entry : nullptr;
    }

    Entry &require(NodeHandle handle) {
        auto *entry = find(handle);
        if (!entry)
            throw std::invalid_argument("node handle is not live in hierarchy");
        return *entry;
    }

    void remove_child(NodeHandle parent, NodeHandle child) noexcept {
        auto *parent_entry = find(parent);
        auto *child_entry = find(child);
        if (!parent_entry || !child_entry)
            return;
        if (child_entry->prev_sibling.valid())
            if (auto *previous = find(child_entry->prev_sibling))
                previous->next_sibling = child_entry->next_sibling;
        if (child_entry->next_sibling.valid())
            if (auto *next = find(child_entry->next_sibling))
                next->prev_sibling = child_entry->prev_sibling;
        if (parent_entry->first_child == child)
            parent_entry->first_child = child_entry->next_sibling;
        if (parent_entry->last_child == child)
            parent_entry->last_child = child_entry->prev_sibling;
        child_entry->prev_sibling = {};
        child_entry->next_sibling = {};
    }

    const NodeStore *nodes_;
    std::vector<Entry> entries_;
};

} // namespace nkscene

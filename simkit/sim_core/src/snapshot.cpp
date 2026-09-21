#include "internal.hpp"

#include <algorithm>

namespace {

template<class T>
nksim_result copy_value(const std::vector<T> &values, uint64_t index, T *out_value) {
    if (!out_value || !nksim::valid_struct_size(out_value->struct_size, sizeof(*out_value)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    if (index >= values.size())
        return NKSIM_ERROR_INVALID_ARGUMENT;
    *out_value = values[static_cast<std::size_t>(index)];
    return NKSIM_OK;
}

} // namespace

extern "C" {

void NKSIM_CALL nksim_snapshot_destroy(nksim_snapshot snapshot) {
    auto &state = nksim::registry();
    std::lock_guard lock(state.mutex);
    state.snapshots.remove(snapshot);
}

nksim_result NKSIM_CALL nksim_snapshot_get_clock(nksim_snapshot snapshot,
                                                 nksim_clock *out_clock) {
    if (!out_clock || !nksim::valid_struct_size(out_clock->struct_size, sizeof(*out_clock)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_snapshot(snapshot);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    value->clock.write(*out_clock);
    return NKSIM_OK;
}

nksim_result NKSIM_CALL nksim_snapshot_get_body_count(nksim_snapshot snapshot,
                                                      uint64_t *out_count) {
    if (!out_count)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_snapshot(snapshot);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    *out_count = value->bodies.size();
    return NKSIM_OK;
}

nksim_result NKSIM_CALL nksim_snapshot_get_body(nksim_snapshot snapshot, uint64_t index,
                                                nksim_body_state *out_state) {
    const auto value = nksim::resolve_snapshot(snapshot);
    return value ? copy_value(value->bodies, index, out_state) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_snapshot_get_body_page(nksim_snapshot snapshot,
                                                     uint64_t start_index,
                                                     nksim_snapshot_body_page *out_page) {
    if (!out_page || !nksim::valid_struct_size(out_page->struct_size, sizeof(*out_page)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_snapshot(snapshot);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    out_page->start_index = start_index;
    out_page->count = 0;
    if (start_index >= value->bodies.size())
        return NKSIM_OK;
    const auto remaining = value->bodies.size() - static_cast<std::size_t>(start_index);
    const auto count = std::min<std::size_t>(remaining, NKSIM_SNAPSHOT_PAGE_CAPACITY);
    out_page->count = static_cast<uint32_t>(count);
    for (std::size_t index = 0; index < count; ++index)
        out_page->bodies[index] = value->bodies[static_cast<std::size_t>(start_index) + index];
    return NKSIM_OK;
}

nksim_result NKSIM_CALL nksim_snapshot_get_joint_count(nksim_snapshot snapshot,
                                                       uint64_t *out_count) {
    if (!out_count)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_snapshot(snapshot);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    *out_count = value->joints.size();
    return NKSIM_OK;
}

nksim_result NKSIM_CALL nksim_snapshot_get_joint(nksim_snapshot snapshot, uint64_t index,
                                                 nksim_joint_state *out_state) {
    const auto value = nksim::resolve_snapshot(snapshot);
    return value ? copy_value(value->joints, index, out_state) : NKSIM_ERROR_INVALID_HANDLE;
}

nksim_result NKSIM_CALL nksim_snapshot_get_joint_page(nksim_snapshot snapshot,
                                                      uint64_t start_index,
                                                      nksim_snapshot_joint_page *out_page) {
    if (!out_page || !nksim::valid_struct_size(out_page->struct_size, sizeof(*out_page)))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const auto value = nksim::resolve_snapshot(snapshot);
    if (!value)
        return NKSIM_ERROR_INVALID_HANDLE;
    out_page->start_index = start_index;
    out_page->count = 0;
    if (start_index >= value->joints.size())
        return NKSIM_OK;
    const auto remaining = value->joints.size() - static_cast<std::size_t>(start_index);
    const auto count = std::min<std::size_t>(remaining, NKSIM_SNAPSHOT_PAGE_CAPACITY);
    out_page->count = static_cast<uint32_t>(count);
    for (std::size_t index = 0; index < count; ++index)
        out_page->joints[index] = value->joints[static_cast<std::size_t>(start_index) + index];
    return NKSIM_OK;
}

} // extern "C"

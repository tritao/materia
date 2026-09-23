#include "scene_internal.hpp"

#include <atomic>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

int main() {
    constexpr std::size_t node_count = 1'000'000;
    constexpr int reader_count = 4;
    auto scene = std::make_shared<nkscene::Scene>();
    std::vector<nkscene::NodeId> nodes;
    nodes.reserve(node_count);
    nkscene::Transaction create(scene);
    for (std::size_t index = 0; index < node_count; ++index) {
        const auto node = scene->reserve_node_id();
        nodes.push_back(node);
        create.add_create(node);
    }

    nkscene::ChangeSet changes;
    const auto create_start = std::chrono::steady_clock::now();
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();
    const auto create_elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - create_start);
    assert(changes.stats.changed_nodes == node_count);
    assert(scene->snapshot().nodes().size() == node_count);

    nkscene::Transaction hierarchy(scene);
    for (std::size_t index = 2; index < node_count; ++index)
        hierarchy.add_parent(nodes[index], nodes.front());
    const auto hierarchy_start = std::chrono::steady_clock::now();
    assert(scene->commit(hierarchy, changes) == NKS_OK);
    hierarchy.close();
    const auto hierarchy_elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - hierarchy_start);
    assert(changes.stats.changed_nodes == node_count - 2);
    assert(scene->snapshot().children(nodes.front()).size() == node_count - 2);

    constexpr std::size_t batch_mutation_count = 50'000;
    nkscene::Transaction batch_move(scene);
    for (std::size_t index = 2; index < 2 + batch_mutation_count; ++index) {
        nkscene::LocalTransform transform;
        transform.matrix[12] = 1.0f;
        batch_move.add_transform(nodes[index], transform);
    }
    const auto batch_start = std::chrono::steady_clock::now();
    assert(scene->commit(batch_move, changes) == NKS_OK);
    batch_move.close();
    const auto batch_elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - batch_start);
    assert(changes.stats.changed_nodes == batch_mutation_count);
    assert(changes.stats.dirty_world_transforms == batch_mutation_count);

    const auto before = scene->snapshot();
    std::atomic<bool> failed = false;
    std::vector<std::thread> readers;
    for (int index = 0; index < reader_count; ++index) {
        readers.emplace_back([&] {
            for (int iteration = 0; iteration < 1000; ++iteration) {
                const auto snapshot = scene->snapshot();
                if (snapshot.nodes().size() != node_count ||
                    !snapshot.find(nodes[node_count / 2]))
                    failed.store(true, std::memory_order_release);
            }
        });
    }

    nkscene::LocalTransform transform;
    transform.matrix[12] = 3.0f;
    nkscene::Transaction move(scene);
    move.add_transform(nodes.back(), transform);
    const auto move_start = std::chrono::steady_clock::now();
    assert(scene->commit(move, changes) == NKS_OK);
    move.close();
    const auto move_elapsed = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - move_start);
    for (auto &reader : readers)
        reader.join();

    assert(!failed.load(std::memory_order_acquire));
    assert(changes.stats.changed_nodes == 1);
    assert(changes.stats.dirty_world_transforms == 1);
    assert(changes.stats.dirty_bounds == 0);
    assert(changes.world_transform_nodes.size() == 1);
    assert(changes.world_transform_nodes.front() == nodes.back());
    assert(before.revision() != scene->snapshot().revision());

    nkscene::Transaction reparent(scene);
    reparent.add_parent(nodes.front(), nodes[1]);
    const auto reparent_start = std::chrono::steady_clock::now();
    assert(scene->commit(reparent, changes) == NKS_OK);
    reparent.close();
    const auto reparent_elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - reparent_start);
    assert(changes.stats.changed_nodes == 1);
    assert(changes.stats.dirty_world_transforms == node_count - 1);
    assert(scene->hierarchy_index().children(nodes[1]).size() == 1);
    assert(scene->hierarchy_index().children(nodes.front()).size() == node_count - 2);

    nkscene::ComponentStore<std::uint32_t> sparse_components;
    sparse_components.insert_or_assign(
        {static_cast<std::uint32_t>(node_count - 1), 1}, 7);
    std::uint64_t sparse_checksum = 0;
    const auto sparse_start = std::chrono::steady_clock::now();
    for (int iteration = 0; iteration < 512; ++iteration)
        sparse_components.for_each([&](nkscene::NodeHandle, std::uint32_t value) {
            sparse_checksum += value;
        });
    const auto sparse_elapsed = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - sparse_start);
    assert(sparse_checksum == 512 * 7);

    std::printf("scale nodes=%zu create=%.3f ms hierarchy=%.3f ms batch=%zu/%.3f ms "
                "move=%.3f us reparent=%.3f ms sparse=%.3f us readers=%d\n",
                node_count, create_elapsed.count(), hierarchy_elapsed.count(),
                batch_mutation_count, batch_elapsed.count(), move_elapsed.count(),
                reparent_elapsed.count(), sparse_elapsed.count(), reader_count);
    return 0;
}

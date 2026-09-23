#include "scene_internal.hpp"

#include <atomic>
#include <cassert>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

int main() {
    constexpr std::size_t node_count = 10000;
    constexpr int reader_count = 4;
    constexpr int writer_iterations = 100;

    auto scene = std::make_shared<nkscene::Scene>();
    const auto source = nkscene::EntityId{7};
    const auto geometry = scene->create_geometry();
    const auto material = scene->create_material();
    std::vector<nkscene::NodeId> nodes;
    nodes.reserve(node_count);

    nkscene::Transaction create(scene);
    for (std::size_t index = 0; index < node_count; ++index) {
        const auto node = scene->reserve_node_id();
        nodes.push_back(node);
        create.add_create(node);
        create.add_source_entity(node, source);
        create.add_geometry(node, geometry);
        create.add_material(node, material);
    }
    nkscene::ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    const auto initial = scene->snapshot();
    assert(initial.nodes().size() == node_count);
    assert(initial.nodes_for_source(source).size() == node_count);
    assert(initial.nodes_for_geometry(geometry).size() == node_count);
    assert(initial.nodes_for_material(material).size() == node_count);

    std::atomic<bool> stop = false;
    std::atomic<bool> failed = false;
    std::vector<std::thread> readers;
    readers.reserve(reader_count);
    for (int index = 0; index < reader_count; ++index) {
        readers.emplace_back([&] {
            while (!stop.load(std::memory_order_acquire)) {
                const auto snapshot = scene->snapshot();
                if (snapshot.nodes().size() != node_count ||
                    snapshot.nodes_for_source(source).size() != node_count ||
                    snapshot.nodes_for_geometry(geometry).size() != node_count ||
                    snapshot.nodes_for_material(material).size() != node_count ||
                    !snapshot.find(nodes[node_count / 2])) {
                    failed.store(true, std::memory_order_release);
                    return;
                }
            }
        });
    }

    for (int iteration = 1; iteration <= writer_iterations; ++iteration) {
        nkscene::LocalTransform transform;
        transform.matrix[12] = static_cast<float>(iteration);
        nkscene::Transaction move(scene);
        move.add_transform(nodes.front(), transform);
        assert(scene->commit(move, changes) == NKS_OK);
        move.close();
    }

    stop.store(true, std::memory_order_release);
    for (auto &reader : readers)
        reader.join();

    assert(!failed.load(std::memory_order_acquire));
    assert(initial.find(nodes.front())->world_transform.transform.matrix[12] == 0.0f);
    assert(scene->snapshot().find(nodes.front())->world_transform.transform.matrix[12] ==
           static_cast<float>(writer_iterations));
    return 0;
}

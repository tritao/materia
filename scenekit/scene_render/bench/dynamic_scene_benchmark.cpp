#include "nativekit_scene_render.hpp"

#include "scene_internal.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <numeric>
#include <vector>

#if defined(__linux__)
#include <unistd.h>
#endif

namespace {

using Clock = std::chrono::steady_clock;
using nkscene::ChangeSet;
using nkscene::RenderPlan;
using nkscene::Scene;
using nkscene::Transaction;

constexpr std::size_t node_count = 100'000;
constexpr std::size_t transforms_per_frame = 5'000;
constexpr std::size_t material_changes = 128;
constexpr std::size_t material_interval = 60;
constexpr std::size_t geometry_interval = 180;

struct Samples {
    std::vector<double> milliseconds;

    void add(Clock::time_point start) {
        milliseconds.push_back(
            std::chrono::duration<double, std::milli>(Clock::now() - start).count());
    }

    void print(const char *name) const {
        if (milliseconds.empty()) {
            std::printf("%-25s no samples\n", name);
            return;
        }
        auto sorted = milliseconds;
        std::sort(sorted.begin(), sorted.end());
        const auto sum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
        const auto p95 = static_cast<std::size_t>(std::ceil(sorted.size() * 0.95)) - 1;
        std::printf("%-25s mean=%8.3f ms p95=%8.3f ms max=%8.3f ms n=%zu\n", name,
                    sum / sorted.size(), sorted[p95], sorted.back(), sorted.size());
    }
};

nkscene::LocalTransform transform_for(std::size_t index, std::size_t frame) {
    nkscene::LocalTransform transform;
    const auto phase = static_cast<float>(frame) * 0.01f + static_cast<float>(index) * 0.001f;
    transform.matrix[12] = static_cast<float>(index % 1000) * 3.0f + std::sin(phase) * 0.1f;
    transform.matrix[13] = static_cast<float>(index / 1000) * 3.0f + std::cos(phase) * 0.1f;
    return transform;
}

std::size_t resident_bytes() {
#if defined(__linux__)
    std::ifstream statm("/proc/self/statm");
    std::size_t total_pages = 0;
    std::size_t resident_pages = 0;
    long page_size = sysconf(_SC_PAGESIZE);
    if (statm >> total_pages >> resident_pages && page_size > 0)
        return resident_pages * static_cast<std::size_t>(page_size);
#endif
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    const std::size_t frames = argc > 1
                                   ? static_cast<std::size_t>(std::strtoull(argv[1], nullptr, 10))
                                   : 600;
    const std::size_t geometry_edit_interval = argc > 2
        ? static_cast<std::size_t>(std::strtoull(argv[2], nullptr, 10))
        : geometry_interval;
    if (frames == 0 || geometry_edit_interval == 0)
        return 2;

    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &geometry_resource = scene->geometry_store().create(geometry);
    geometry_resource.bounds.valid = true;
    geometry_resource.bounds.minimum = {-0.5f, -0.5f, 0.0f};
    geometry_resource.bounds.maximum = {0.5f, 0.5f, 0.1f};
    auto &initial_payload = geometry_resource.edit_payload();
    initial_payload.vertices = {
        nkscene::GeometryVertex{{-0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.5f, -0.5f, 0.0f}},
        nkscene::GeometryVertex{{0.5f, 0.5f, 0.0f}},
        nkscene::GeometryVertex{{-0.5f, 0.5f, 0.0f}},
        nkscene::GeometryVertex{{-0.5f, -0.5f, 0.1f}},
        nkscene::GeometryVertex{{0.5f, -0.5f, 0.1f}},
        nkscene::GeometryVertex{{0.5f, 0.5f, 0.1f}},
        nkscene::GeometryVertex{{-0.5f, 0.5f, 0.1f}}};
    initial_payload.indices = {0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7};
    scene->publish();

    std::vector<nkscene::MaterialId> materials;
    for (std::size_t index = 0; index < 4; ++index)
        materials.push_back(scene->reserve_material_id());
    for (const auto material : materials)
        scene->material_store().create(material);
    scene->publish();

    std::vector<nkscene::NodeId> nodes;
    nodes.reserve(node_count);
    Transaction create(scene);
    for (std::size_t index = 0; index < node_count; ++index) {
        const auto node = scene->reserve_node_id();
        nodes.push_back(node);
        create.add_create(node);
    }
    ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction configure(scene);
    for (std::size_t index = 0; index < node_count; ++index) {
        configure.add_geometry(nodes[index], geometry);
        configure.add_material(nodes[index], materials[index % materials.size()]);
        configure.add_transform(nodes[index], transform_for(index, 0));
    }
    assert(scene->commit(configure, changes) == NKS_OK);
    configure.close();

    const nkscene::SceneView view;
    auto plan = nkscene::compile(scene->snapshot(), view);
    assert(plan.items().size() == node_count);

    const auto baseline_rss = resident_bytes();
    auto peak_rss = baseline_rss;
    Samples transaction_build;
    Samples commit_publish;
    Samples snapshot_capture;
    Samples renderer_update;
    Samples geometry_publish;
    Samples geometry_renderer_refresh;
    std::size_t geometry_edits = 0;
    std::size_t material_edit_frames = 0;

    for (std::size_t frame = 1; frame <= frames; ++frame) {
        Transaction move(scene);
        auto start = Clock::now();
        for (std::size_t index = 0; index < transforms_per_frame; ++index)
            move.add_transform(nodes[index], transform_for(index, frame));
        if (frame % material_interval == 0) {
            const auto destination = materials[(frame / material_interval) % materials.size()];
            for (std::size_t index = 0; index < material_changes; ++index)
                move.add_material(nodes[index], destination);
            ++material_edit_frames;
        }
        transaction_build.add(start);

        start = Clock::now();
        assert(scene->commit(move, changes) == NKS_OK);
        move.close();
        commit_publish.add(start);

        nkscene::SceneSnapshot snapshot;
        start = Clock::now();
        snapshot = scene->snapshot();
        snapshot_capture.add(start);

        start = Clock::now();
        const auto render_update = nkscene::update(plan, snapshot, changes, view);
        renderer_update.add(start);
        assert(!render_update.plan_rebuilt);

        if (frame % geometry_edit_interval == 0) {
            start = Clock::now();
            auto *resource = scene->geometry_store().find(geometry);
            assert(resource);
            auto &payload = resource->edit_payload();
            const auto height = frame % (geometry_edit_interval * 2) == 0 ? 0.12f : 0.1f;
            for (auto &vertex : payload.vertices)
                vertex.position[2] = vertex.position[2] > 0.0f ? height : 0.0f;
            resource->bounds.maximum[2] = height;
            scene->publish();
            geometry_publish.add(start);

            const auto geometry_snapshot = scene->snapshot();
            start = Clock::now();
            const auto geometry_update = nkscene::refresh(plan, geometry_snapshot, view);
            geometry_renderer_refresh.add(start);
            assert(!geometry_update.plan_rebuilt);
            assert(geometry_update.updated_geometry_resources == 1);
            ++geometry_edits;
        }

        const auto rss = resident_bytes();
        peak_rss = std::max(peak_rss, rss);
        if (frame % 60 == 0 || frame == frames) {
            if (rss != 0)
                std::printf("frame=%zu rss=%zu MiB\n", frame, rss / (1024 * 1024));
            else
                std::printf("frame=%zu rss=unavailable\n", frame);
        }
    }

    std::printf("dynamic scene: nodes=%zu moving/frame=%zu frames=%zu material edits=%zu "
                "geometry edits=%zu interval=%zu frames\n",
                node_count, transforms_per_frame, frames, material_edit_frames,
                geometry_edits, geometry_edit_interval);
    transaction_build.print("transaction construction");
    commit_publish.print("commit + publication");
    snapshot_capture.print("snapshot capture");
    renderer_update.print("renderer update");
    geometry_publish.print("geometry edit + publish");
    geometry_renderer_refresh.print("geometry renderer refresh");
    if (baseline_rss != 0 && peak_rss != 0)
        std::printf("resident memory: baseline=%zu MiB peak=%zu MiB peak growth=%zu MiB\n",
                    baseline_rss / (1024 * 1024), peak_rss / (1024 * 1024),
                    (peak_rss - baseline_rss) / (1024 * 1024));
    else
        std::printf("resident memory: unavailable on this platform\n");
    return 0;
}

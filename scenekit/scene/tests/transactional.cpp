#include "nativekit_scene.h"

#include "component_store.hpp"
#include "scene_internal.hpp"

#include <cassert>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace {

using nkscene::ChangeDomain;
using nkscene::ComponentStore;
using nkscene::LocalTransform;
using nkscene::Scene;
using nkscene::Transaction;

LocalTransform translated(float x) {
    LocalTransform transform;
    transform.matrix[12] = x;
    return transform;
}

void component_store_is_slot_indexed_and_generation_safe() {
    ComponentStore<std::uint32_t> store;
    const nkscene::NodeHandle first{4, 1};
    const nkscene::NodeHandle second{8, 3};
    const nkscene::NodeHandle third{12, 2};
    store.insert_or_assign(first, 10);
    store.insert_or_assign(second, 20);
    store.insert_or_assign(third, 30);
    assert(store.size() == 3);
    assert(*store.find(second) == 20);
    assert(store.erase(second));
    assert(!store.contains(second));
    assert(*store.find(third) == 30);
    store.insert_or_assign(first, 11);
    assert(*store.find(first) == 11);

    const nkscene::NodeHandle reused{8, 4};
    assert(store.find(reused) == nullptr);
    store.insert_or_assign(reused, 40);
    assert(store.find(second) == nullptr);
    assert(*store.find(reused) == 40);

    std::size_t visited = 0;
    store.for_each([&](nkscene::NodeHandle handle, std::uint32_t value) {
        ++visited;
        assert(store.find(handle) && *store.find(handle) == value);
    });
    assert(visited == store.size());

    ComponentStore<std::uint32_t> sparse;
    const nkscene::NodeHandle distant{1'000'000, 7};
    sparse.insert_or_assign(distant, 99);
    for (int iteration = 0; iteration < 64; ++iteration) {
        visited = 0;
        sparse.for_each([&](nkscene::NodeHandle handle, std::uint32_t value) {
            ++visited;
            assert(handle == distant);
            assert(value == 99);
        });
        assert(visited == 1);
    }
}

void node_handles_reject_stale_components() {
    nkscene::NodeStore nodes;
    ComponentStore<std::uint32_t> store;
    const auto first_id = nodes.reserve_id();
    const auto first = nodes.create(first_id);
    store.insert_or_assign(first, 10);
    assert(*store.find(first) == 10);
    assert(nodes.destroy(first_id));

    const auto second_id = nodes.reserve_id();
    const auto second = nodes.create(second_id);
    assert(second.slot == first.slot);
    assert(second.generation != first.generation);
    assert(store.erase(first));
    assert(store.find(first) == nullptr);
    store.insert_or_assign(second, 20);
    assert(store.find(first) == nullptr);
    assert(*store.find(second) == 20);
}

void hierarchy_links_are_slot_indexed() {
    auto scene = std::make_shared<Scene>();
    const auto root = scene->reserve_node_id();
    const auto first = scene->reserve_node_id();
    const auto second = scene->reserve_node_id();
    Transaction create(scene);
    create.add_create(root);
    create.add_create(first);
    create.add_create(second);
    nkscene::ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction group(scene);
    group.add_parent(first, root);
    group.add_parent(second, root);
    assert(scene->commit(group, changes) == NKS_OK);
    group.close();
    auto root_children = scene->hierarchy_index().children(root);
    assert(root_children.size() == 2);
    assert(root_children[0] == first);
    assert(root_children[1] == second);
    assert(scene->hierarchy_index().is_descendant(second, root));

    Transaction nest(scene);
    nest.add_parent(first, second);
    assert(scene->commit(nest, changes) == NKS_OK);
    nest.close();
    root_children = scene->hierarchy_index().children(root);
    assert(root_children.size() == 1 && root_children.front() == second);
    const auto second_children = scene->hierarchy_index().children(second);
    assert(second_children.size() == 1 && second_children.front() == first);
    assert(scene->hierarchy_index().is_descendant(first, root));

    Transaction invalid_destroy(scene);
    invalid_destroy.add_destroy(second);
    assert(scene->commit(invalid_destroy, changes) == NKS_ERROR_INVALID_ARGUMENT);
    invalid_destroy.close();

    Transaction detach_and_destroy(scene);
    detach_and_destroy.add_parent(first, nkscene::invalid_node);
    detach_and_destroy.add_destroy(second);
    assert(scene->commit(detach_and_destroy, changes) == NKS_OK);
    detach_and_destroy.close();
    assert(scene->hierarchy_index().parent(first) == nkscene::invalid_node);

    const auto replacement = scene->reserve_node_id();
    Transaction recreate(scene);
    recreate.add_create(replacement);
    assert(scene->commit(recreate, changes) == NKS_OK);
    recreate.close();
    assert(scene->hierarchy_index().parent(replacement) == nkscene::invalid_node);
}

void changes_are_domain_precise() {
    auto scene = std::make_shared<Scene>();
    const auto first = scene->reserve_node_id();
    const auto second = scene->reserve_node_id();
    Transaction create(scene);
    create.add_create(first);
    create.add_create(second);
    nkscene::ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();
    assert(changes.changes.size() == 2);
    assert(changes.revisions.scene == 1);
    assert(changes.revisions.hierarchy == 1);

    const auto transform = translated(4.0f);
    Transaction move(scene);
    move.add_transform(first, transform);
    assert(scene->commit(move, changes) == NKS_OK);
    move.close();
    assert(changes.changes.size() == 1);
    assert(changes.changes.front().node == first);
    assert(changes.changes.front().domains == ChangeDomain::Transform);
    assert(changes.revisions.scene == 2);
    assert(changes.revisions.transform == 1);
    assert(changes.revisions.hierarchy == 1);
    assert(changes.revisions.geometry == 0);
    assert(changes.revisions.material == 0);
    assert(changes.revisions.visibility == 0);
    assert(changes.revisions.source == 0);
    assert(scene->transforms().find(scene->node_store().resolve(first))->matrix[12] ==
           4.0f);

    Transaction source(scene);
    source.add_source_entity(first, nkscene::EntityId{42});
    assert(scene->commit(source, changes) == NKS_OK);
    source.close();
    assert(changes.changes.size() == 1);
    assert(changes.changes.front().node == first);
    assert(changes.changes.front().domains == ChangeDomain::Source);
    assert(changes.revisions.source == 1);
    assert(changes.revisions.transform == 1);
    assert(scene->snapshot().find(first)->source == nkscene::EntityId{42});

    Transaction clear_source(scene);
    clear_source.add_source_entity(first, nkscene::invalid_entity);
    assert(scene->commit(clear_source, changes) == NKS_OK);
    clear_source.close();
    assert(changes.changes.size() == 1);
    assert(changes.changes.front().domains == ChangeDomain::Source);
    assert(scene->snapshot().find(first)->source == nkscene::invalid_entity);

    Transaction invalid(scene);
    invalid.add_transform(first, translated(9.0f));
    invalid.add_transform({999999}, translated(10.0f));
    assert(scene->commit(invalid, changes) == NKS_ERROR_STALE_ID);
    assert(scene->transforms().find(scene->node_store().resolve(first))->matrix[12] ==
           4.0f);
    assert(scene->revision() == 4);

    Transaction cycle(scene);
    cycle.add_parent(first, second);
    cycle.add_parent(second, first);
    assert(scene->commit(cycle, changes) == NKS_ERROR_HIERARCHY_CYCLE);
    assert(scene->hierarchy_index().parent(first) == nkscene::invalid_node);
    assert(scene->hierarchy_index().parent(second) == nkscene::invalid_node);
}

void one_transaction_reuses_generation_safe_handles() {
    auto scene = std::make_shared<Scene>();
    const auto root = scene->reserve_node_id();
    const auto child = scene->reserve_node_id();
    const auto transient = scene->reserve_node_id();

    Transaction mutation(scene);
    mutation.add_create(root);
    mutation.add_create(child);
    mutation.add_parent(child, root);
    mutation.add_transform(child, translated(8.0f));
    mutation.add_visibility(child, false);
    mutation.add_source_entity(child, nkscene::EntityId{17});
    nkscene::ChangeSet changes;
    assert(scene->commit(mutation, changes) == NKS_OK);
    mutation.close();

    const auto snapshot = scene->snapshot();
    const auto *value = snapshot.find(child);
    assert(value);
    assert(value->parent == root);
    assert(value->world_transform.transform.matrix[12] == 8.0f);
    assert(!value->visible);
    assert(value->source == nkscene::EntityId{17});

    Transaction create_and_destroy(scene);
    create_and_destroy.add_create(transient);
    create_and_destroy.add_transform(transient, translated(4.0f));
    create_and_destroy.add_destroy(transient);
    assert(scene->commit(create_and_destroy, changes) == NKS_OK);
    create_and_destroy.close();
    assert(!scene->contains(transient));
    assert(changes.changes.size() == 1);
    assert(has_domain(changes.changes.front().domains, ChangeDomain::Created));
    assert(has_domain(changes.changes.front().domains, ChangeDomain::Transform));
    assert(has_domain(changes.changes.front().domains, ChangeDomain::Destroyed));
}

void shared_resources_do_not_follow_instance_transforms() {
    constexpr std::size_t count = 50000;
    auto scene = std::make_shared<Scene>();
    const auto geometry = scene->reserve_geometry_id();
    auto &resource = scene->geometry_store().create(geometry);
    resource.bounds.valid = true;
    resource.bounds.minimum = {-1.0f, -1.0f, -1.0f};
    resource.bounds.maximum = {1.0f, 1.0f, 1.0f};
    const auto resource_revision = resource.revision;

    std::vector<nkscene::NodeId> nodes;
    nodes.reserve(count);
    Transaction create(scene);
    for (std::size_t index = 0; index < count; ++index) {
        const auto node = scene->reserve_node_id();
        nodes.push_back(node);
        create.add_create(node);
    }
    nkscene::ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction assign_geometry(scene);
    for (const auto node : nodes)
        assign_geometry.add_geometry(node, geometry);
    assert(scene->commit(assign_geometry, changes) == NKS_OK);
    assign_geometry.close();
    assert(changes.stats.changed_nodes == count);
    assert(scene->geometry_store().find(geometry)->revision == resource_revision);

    Transaction move(scene);
    move.add_transform(nodes.front(), translated(12.0f));
    assert(scene->commit(move, changes) == NKS_OK);
    move.close();
    assert(changes.stats.changed_nodes == 1);
    assert(changes.stats.dirty_world_transforms == 1);
    assert(changes.stats.dirty_bounds == 1);
    assert(scene->geometry_store().find(geometry)->revision == resource_revision);
    const auto handle = scene->node_store().resolve(nodes.front());
    assert(scene->world_transforms().find(handle)->transform.matrix[12] == 12.0f);
    assert(scene->world_transforms().find(handle)->revision != 0);
}

void snapshots_are_immutable() {
    auto scene = std::make_shared<Scene>();
    const auto node = scene->reserve_node_id();
    Transaction create(scene);
    create.add_create(node);
    nkscene::ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    Transaction move(scene);
    move.add_transform(node, translated(3.0f));
    assert(scene->commit(move, changes) == NKS_OK);
    move.close();
    const auto before = scene->snapshot();
    assert(before.revision() == scene->revision());
    assert(before.nodes().size() == 1);
    assert(before.find(node)->world_transform.transform.matrix[12] == 3.0f);

    Transaction move_again(scene);
    move_again.add_transform(node, translated(7.0f));
    assert(scene->commit(move_again, changes) == NKS_OK);
    move_again.close();
    assert(scene->snapshot().find(node)->world_transform.transform.matrix[12] == 7.0f);
    assert(before.find(node)->world_transform.transform.matrix[12] == 3.0f);
    assert(before.revision() != scene->revision());
}

void names_and_bulk_transforms_are_transactional() {
    auto scene = std::make_shared<Scene>();
    std::vector<nkscene::NodeId> nodes;
    for (int index = 0; index < 3; ++index)
        nodes.push_back(scene->reserve_node_id());

    Transaction create(scene);
    for (const auto node : nodes)
        create.add_create(node);
    nkscene::ChangeSet changes;
    assert(scene->commit(create, changes) == NKS_OK);
    create.close();

    std::vector<nkscene::TransformUpdate> updates;
    for (std::size_t index = 0; index < nodes.size(); ++index)
        updates.push_back({nodes[index], translated(static_cast<float>(index + 1))});
    Transaction move(scene);
    move.add_transforms(std::span<const nkscene::TransformUpdate>{updates});
    assert(scene->commit(move, changes) == NKS_OK);
    move.close();
    assert(changes.changes.size() == nodes.size());
    assert(changes.revisions.transform == 1);
    assert(scene->world_transforms()
               .find(scene->node_store().resolve(nodes[2]))
               ->transform.matrix[12] == 3.0f);

    constexpr nkscene::EntityId robot_link{100};
    Transaction names(scene);
    names.add_source_entity(nodes[0], robot_link);
    names.add_name(nodes[0], "panda_link0");
    names.add_entity_name(robot_link, "RobotLink");
    assert(scene->commit(names, changes) == NKS_OK);
    names.close();
    assert(changes.revisions.name == 1);
    const auto snapshot = scene->snapshot();
    assert(snapshot.name(nodes[0]) == "panda_link0");
    assert(snapshot.entity_name(robot_link) == "RobotLink");
}

} // namespace

int main() {
    component_store_is_slot_indexed_and_generation_safe();
    node_handles_reject_stale_components();
    hierarchy_links_are_slot_indexed();
    changes_are_domain_precise();
    one_transaction_reuses_generation_safe_handles();
    shared_resources_do_not_follow_instance_transforms();
    snapshots_are_immutable();
    names_and_bulk_transforms_are_transactional();
    return 0;
}

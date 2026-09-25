#include "cadkit.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

void assert_close(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-9);
}

} // namespace

int main() {
    cad_resource_counts resource_baseline{};
    assert(cad_resource_counts_get(&resource_baseline) == CAD_OK);
    cad_shape diagnostic_shape = 0;
    cad_mesh diagnostic_mesh = 0;
    cad_operation diagnostic_operation = 0;
    cad_mesh_options diagnostic_mesh_options{0.1, 0.5};
    assert(cad_box(1.0, 1.0, 1.0, &diagnostic_shape) == CAD_OK);
    assert(cad_shape_tessellate(diagnostic_shape, &diagnostic_mesh_options, &diagnostic_mesh) == CAD_OK);
    assert(cad_shape_translate_operation(diagnostic_shape, {1.0, 0.0, 0.0},
                                        &diagnostic_operation) == CAD_OK);
    cad_resource_counts resource_live{};
    assert(cad_resource_counts_get(&resource_live) == CAD_OK);
    assert(resource_live.shape_count == resource_baseline.shape_count + 1);
    assert(resource_live.mesh_count == resource_baseline.mesh_count + 1);
    assert(resource_live.operation_count == resource_baseline.operation_count + 1);
    cad_shape_destroy(diagnostic_shape);
    cad_mesh_destroy(diagnostic_mesh);
    cad_operation_destroy(diagnostic_operation);
    cad_resource_counts resource_released{};
    assert(cad_resource_counts_get(&resource_released) == CAD_OK);
    assert(resource_released.shape_count == resource_baseline.shape_count);
    assert(resource_released.mesh_count == resource_baseline.mesh_count);
    assert(resource_released.operation_count == resource_baseline.operation_count);

    cad_shape shape = 0;
    assert(cad_box(10.0, 20.0, 30.0, &shape) == CAD_OK);
    assert(shape != 0);

    cad_bounds bounds{};
    assert(cad_shape_bounds(shape, &bounds) == CAD_OK);
    assert_close(bounds.min.x, 0.0);
    assert_close(bounds.min.y, 0.0);
    assert_close(bounds.min.z, 0.0);
    assert_close(bounds.max.x, 10.0);
    assert_close(bounds.max.y, 20.0);
    assert_close(bounds.max.z, 30.0);

    double area = 0.0;
    double volume = 0.0;
    assert(cad_shape_area(shape, &area) == CAD_OK);
    assert(cad_shape_volume(shape, &volume) == CAD_OK);
    assert_close(area, 2200.0);
    assert_close(volume, 6000.0);
    cad_mass_properties properties{};
    assert(cad_shape_mass_properties(shape, &properties) == CAD_OK);
    assert_close(properties.volume, volume);
    assert_close(properties.surface_area, area);
    assert_close(properties.center_of_mass.x, 5.0);
    assert_close(properties.center_of_mass.y, 10.0);
    assert_close(properties.center_of_mass.z, 15.0);
    assert(cad_shape_mass_properties(shape, nullptr) == CAD_ERROR_INVALID_ARGUMENT);

    const auto step_path = std::filesystem::temp_directory_path() /
        "cadkit-core-smoke.step";
    const auto step_path_string = step_path.string();
    std::filesystem::remove(step_path);
    assert(cad_step_export(shape, step_path_string.c_str()) == CAD_OK);
    assert(std::filesystem::is_regular_file(step_path));
    std::ifstream step_file(step_path, std::ios::binary);
    const std::string step_text((std::istreambuf_iterator<char>(step_file)),
        std::istreambuf_iterator<char>());
    assert(!step_text.empty());
    cad_shape imported_text = 0;
    assert(cad_step_import_text(step_text.c_str(), &imported_text) == CAD_OK);
    assert(imported_text != 0);
    double imported_text_volume = 0.0;
    assert(cad_shape_volume(imported_text, &imported_text_volume) == CAD_OK);
    assert_close(imported_text_volume, volume);
    cad_shape_destroy(imported_text);
    cad_shape invalid_imported_text = 0;
    assert(cad_step_import_text(nullptr, &invalid_imported_text) == CAD_ERROR_INVALID_ARGUMENT);
    cad_shape imported = 0;
    assert(cad_step_import(step_path_string.c_str(), &imported) == CAD_OK);
    assert(imported != 0);
    cad_bounds imported_bounds{};
    assert(cad_shape_bounds(imported, &imported_bounds) == CAD_OK);
    assert_close(imported_bounds.min.x, bounds.min.x);
    assert_close(imported_bounds.min.y, bounds.min.y);
    assert_close(imported_bounds.min.z, bounds.min.z);
    assert_close(imported_bounds.max.x, bounds.max.x);
    assert_close(imported_bounds.max.y, bounds.max.y);
    assert_close(imported_bounds.max.z, bounds.max.z);
    double imported_volume = 0.0;
    assert(cad_shape_volume(imported, &imported_volume) == CAD_OK);
    assert_close(imported_volume, volume);
    cad_shape invalid_imported = 99;
    assert(cad_step_import(nullptr, &invalid_imported) == CAD_ERROR_INVALID_ARGUMENT);
    assert(invalid_imported == 0);
    assert(cad_step_import("", &invalid_imported) == CAD_ERROR_INVALID_ARGUMENT);
    assert(invalid_imported == 0);
    const auto missing_step_path = step_path.string() + ".missing";
    assert(cad_step_import(missing_step_path.c_str(), &invalid_imported) ==
           CAD_ERROR_OPERATION_FAILED);
    assert(invalid_imported == 0);
    assert(cad_step_export(shape, nullptr) == CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_step_export(shape, "") == CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_step_export(0, step_path_string.c_str()) == CAD_ERROR_INVALID_HANDLE);
    cad_shape_destroy(imported);
    std::filesystem::remove(step_path);

    cad_shape boolean_base = 0;
    cad_shape boolean_tool = 0;
    cad_shape boolean_tool_offset = 0;
    assert(cad_box(10.0, 10.0, 10.0, &boolean_base) == CAD_OK);
    assert(cad_box(5.0, 5.0, 5.0, &boolean_tool) == CAD_OK);
    assert(cad_shape_translate(
               boolean_tool,
               {2.5, 2.5, 2.5},
               &boolean_tool_offset) == CAD_OK);

    cad_shape fused = 0;
    cad_shape cut = 0;
    cad_shape common = 0;
    double boolean_base_volume = 0.0;
    double boolean_tool_volume = 0.0;
    assert(cad_shape_volume(boolean_base, &boolean_base_volume) == CAD_OK);
    assert(cad_shape_volume(boolean_tool_offset, &boolean_tool_volume) == CAD_OK);
    assert(cad_fuse(boolean_base, boolean_tool_offset, &fused) == CAD_OK);
    assert(cad_cut(boolean_base, boolean_tool_offset, &cut) == CAD_OK);
    assert(cad_common(boolean_base, boolean_tool_offset, &common) == CAD_OK);
    assert(cad_shape_volume(fused, &volume) == CAD_OK);
    assert_close(volume, 1000.0);
    assert(cad_shape_volume(cut, &volume) == CAD_OK);
    assert_close(volume, 875.0);
    assert(cad_shape_volume(common, &volume) == CAD_OK);
    assert_close(volume, 125.0);
    assert(cad_shape_volume(boolean_base, &volume) == CAD_OK);
    assert_close(volume, boolean_base_volume);
    assert(cad_shape_volume(boolean_tool_offset, &volume) == CAD_OK);
    assert_close(volume, boolean_tool_volume);

    cad_operation cut_operation = 0;
    assert(cad_cut_operation(boolean_base, boolean_tool_offset, &cut_operation) == CAD_OK);
    cad_shape operation_result = 0;
    assert(cad_operation_result_shape(cut_operation, &operation_result) == CAD_OK);
    assert(cad_shape_volume(operation_result, &volume) == CAD_OK);
    assert_close(volume, 875.0);
    assert(cad_shape_volume(boolean_base, &volume) == CAD_OK);
    assert_close(volume, boolean_base_volume);
    assert(cad_shape_volume(boolean_tool_offset, &volume) == CAD_OK);
    assert_close(volume, boolean_tool_volume);
    uint32_t generated_count = 0;
    uint32_t modified_count = 0;
    uint32_t deleted_count = 0;
    assert(cad_operation_history_count(
               cut_operation, CAD_HISTORY_GENERATED, &generated_count) == CAD_OK);
    assert(cad_operation_history_count(
               cut_operation, CAD_HISTORY_MODIFIED, &modified_count) == CAD_OK);
    assert(cad_operation_history_count(
               cut_operation, CAD_HISTORY_DELETED, &deleted_count) == CAD_OK);
    assert(generated_count + modified_count + deleted_count > 0);
    if (modified_count > 0) {
        cad_shape source = 0;
        cad_shape target = 0;
        assert(cad_operation_history_source_at(
                   cut_operation, CAD_HISTORY_MODIFIED, 0, &source) == CAD_OK);
        assert(cad_operation_history_target_at(
                   cut_operation, CAD_HISTORY_MODIFIED, 0, &target) == CAD_OK);
        assert(source != 0);
        assert(target != 0);
        cad_shape_destroy(source);
        cad_shape_destroy(target);
    } else if (generated_count > 0) {
        cad_shape source = 0;
        cad_shape target = 0;
        assert(cad_operation_history_source_at(
                   cut_operation, CAD_HISTORY_GENERATED, 0, &source) == CAD_OK);
        assert(cad_operation_history_target_at(
                   cut_operation, CAD_HISTORY_GENERATED, 0, &target) == CAD_OK);
        assert(source != 0);
        assert(target != 0);
        cad_shape_destroy(source);
        cad_shape_destroy(target);
    } else {
        cad_shape source = 0;
        assert(cad_operation_history_source_at(
                   cut_operation, CAD_HISTORY_DELETED, 0, &source) == CAD_OK);
        assert(source != 0);
        cad_shape_destroy(source);
    }
    cad_shape_destroy(operation_result);
    cad_operation_destroy(cut_operation);

    cad_operation translate_operation = 0;
    assert(cad_shape_translate_operation(
               shape, {2.0, -3.0, 4.0}, &translate_operation) == CAD_OK);
    cad_shape translate_result = 0;
    assert(cad_operation_result_shape(translate_operation, &translate_result) == CAD_OK);
    assert(cad_shape_bounds(translate_result, &bounds) == CAD_OK);
    assert_close(bounds.min.x, 2.0);
    assert_close(bounds.min.y, -3.0);
    assert_close(bounds.min.z, 4.0);
    assert(cad_operation_history_count(
               translate_operation, CAD_HISTORY_MODIFIED, &modified_count) == CAD_OK);
    assert(modified_count > 0);
    cad_shape_kind wrong_kind = CAD_SHAPE_UNKNOWN;
    assert(cad_shape_kind_get(static_cast<cad_shape>(translate_operation), &wrong_kind) ==
           CAD_ERROR_INVALID_HANDLE);
    cad_shape_destroy(translate_result);

    cad_shape_kind shape_kind = CAD_SHAPE_UNKNOWN;
    assert(cad_shape_kind_get(shape, &shape_kind) == CAD_OK);
    assert(shape_kind == CAD_SHAPE_SOLID);

    uint32_t face_count = 0;
    uint32_t edge_count = 0;
    uint32_t vertex_count = 0;
    assert(cad_shape_subshape_count(shape, CAD_SHAPE_FACE, &face_count) == CAD_OK);
    assert(cad_shape_subshape_count(shape, CAD_SHAPE_EDGE, &edge_count) == CAD_OK);
    assert(cad_shape_subshape_count(shape, CAD_SHAPE_VERTEX, &vertex_count) == CAD_OK);
    assert(face_count == 6);
    assert(edge_count == 12);
    assert(vertex_count == 8);

    std::vector<cad_shape> faces(face_count);
    assert(cad_shape_subshapes(shape, CAD_SHAPE_FACE, faces.data(), face_count) == CAD_OK);
    assert(cad_shape_subshapes(shape, CAD_SHAPE_FACE, faces.data(), face_count - 1) ==
           CAD_ERROR_BUFFER_TOO_SMALL);
    for (const auto face : faces) {
        cad_shape_kind face_kind = CAD_SHAPE_UNKNOWN;
        assert(cad_shape_kind_get(face, &face_kind) == CAD_OK);
        assert(face_kind == CAD_SHAPE_FACE);
        cad_shape_destroy(face);
    }
    cad_shape indexed_face = 0;
    assert(cad_shape_subshape_at(shape, CAD_SHAPE_FACE, 0, &indexed_face) == CAD_OK);
    cad_shape_destroy(indexed_face);
    indexed_face = 0;
    assert(cad_shape_subshape_at(shape, CAD_SHAPE_FACE, face_count, &indexed_face) ==
           CAD_ERROR_INVALID_ARGUMENT);
    assert(indexed_face == 0);

    cad_shape inspected_face = 0;
    assert(cad_shape_subshape_at(shape, CAD_SHAPE_FACE, 0, &inspected_face) == CAD_OK);
    cad_surface_kind surface_kind = CAD_SURFACE_UNKNOWN;
    assert(cad_face_surface_kind(inspected_face, &surface_kind) == CAD_OK);
    assert(surface_kind == CAD_SURFACE_PLANE);
    cad_vec3 face_center{};
    assert(cad_face_center(inspected_face, &face_center) == CAD_OK);
    assert(std::isfinite(face_center.x));
    assert(std::isfinite(face_center.y));
    assert(std::isfinite(face_center.z));
    double face_area = 0.0;
    assert(cad_face_area(inspected_face, &face_area) == CAD_OK);
    assert(face_area > 0.0);
    cad_vec3 face_normal{};
    assert(cad_face_normal(inspected_face, &face_normal) == CAD_OK);
    assert_close(
        std::sqrt(
            face_normal.x * face_normal.x +
            face_normal.y * face_normal.y +
            face_normal.z * face_normal.z),
        1.0);
    cad_shape inspected_face_again = 0;
    assert(cad_shape_subshape_at(shape, CAD_SHAPE_FACE, 0, &inspected_face_again) == CAD_OK);
    std::uint8_t same = 0;
    assert(cad_shape_is_same(inspected_face, inspected_face_again, &same) == CAD_OK);
    assert(same == 1);
    cad_shape cloned_face = 0;
    assert(cad_shape_clone(inspected_face, &cloned_face) == CAD_OK);
    assert(cloned_face != 0);
    assert(cad_shape_is_same(inspected_face, cloned_face, &same) == CAD_OK);
    assert(same == 1);
    cad_shape_destroy(cloned_face);
    cad_shape_destroy(inspected_face);
    cad_shape_destroy(inspected_face_again);

    cad_shape edge = 0;
    assert(cad_shape_subshape_at(shape, CAD_SHAPE_EDGE, 0, &edge) == CAD_OK);
    cad_curve_kind curve_kind = CAD_CURVE_UNKNOWN;
    assert(cad_edge_curve_kind(edge, &curve_kind) == CAD_OK);
    assert(curve_kind == CAD_CURVE_LINE);
    double edge_length = 0.0;
    assert(cad_edge_length(edge, &edge_length) == CAD_OK);
    assert(edge_length > 0.0);
    cad_vec3 edge_tangent{};
    assert(cad_edge_tangent_at(edge, 0.5, &edge_tangent) == CAD_OK);
    assert_close(
        std::sqrt(
            edge_tangent.x * edge_tangent.x +
            edge_tangent.y * edge_tangent.y +
            edge_tangent.z * edge_tangent.z),
        1.0);
    cad_shape vertex = 0;
    assert(cad_shape_subshape_at(shape, CAD_SHAPE_VERTEX, 0, &vertex) == CAD_OK);
    cad_vec3 vertex_position{};
    assert(cad_vertex_position(vertex, &vertex_position) == CAD_OK);
    assert(std::isfinite(vertex_position.x));
    assert(std::isfinite(vertex_position.y));
    assert(std::isfinite(vertex_position.z));
    cad_shape_destroy(edge);
    cad_shape_destroy(vertex);
    assert(cad_shape_subshape_count(shape, CAD_SHAPE_UNKNOWN, &face_count) ==
           CAD_ERROR_INVALID_ARGUMENT);

    cad_mesh_options mesh_options{0.1, 0.5};
    cad_mesh mesh = 0;
    assert(cad_shape_tessellate(shape, &mesh_options, &mesh) == CAD_OK);
    assert(cad_shape_kind_get(static_cast<cad_shape>(mesh), &wrong_kind) == CAD_ERROR_INVALID_HANDLE);
    assert(cad_mesh_vertex_count(static_cast<cad_mesh>(shape), &vertex_count) ==
           CAD_ERROR_INVALID_HANDLE);
    assert(cad_shape_subshape_count(shape, CAD_SHAPE_FACE, &face_count) == CAD_OK);

    uint32_t index_count = 0;
    assert(cad_mesh_vertex_count(mesh, &vertex_count) == CAD_OK);
    assert(cad_mesh_index_count(mesh, &index_count) == CAD_OK);
    uint32_t face_range_count = 0;
    assert(cad_mesh_face_range_count(mesh, &face_range_count) == CAD_OK);
    assert(vertex_count > 0);
    assert(vertex_count == index_count);
    assert(face_range_count == face_count);

    std::vector<cad_vec3> vertices(vertex_count);
    std::vector<cad_vec3> normals(vertex_count);
    std::vector<uint32_t> indices(index_count);
    std::vector<cad_mesh_face_range> face_ranges(face_range_count);
    assert(cad_mesh_copy_vertices(mesh, vertices.data(), vertex_count) == CAD_OK);
    assert(cad_mesh_copy_normals(mesh, normals.data(), vertex_count) == CAD_OK);
    assert(cad_mesh_copy_indices(mesh, indices.data(), index_count) == CAD_OK);
    assert(cad_mesh_copy_face_ranges(mesh, face_ranges.data(), face_range_count) == CAD_OK);
    assert(cad_mesh_copy_face_ranges(mesh, face_ranges.data(), face_range_count - 1) ==
           CAD_ERROR_BUFFER_TOO_SMALL);
    uint32_t next_face_index = 0;
    for (const auto& range : face_ranges) {
        assert(range.face_index == next_face_index);
        assert(range.first_index <= index_count);
        assert(range.index_count <= index_count - range.first_index);
        assert(range.first_index == (next_face_index == 0
                                         ? 0
                                         : face_ranges[next_face_index - 1].first_index +
                                               face_ranges[next_face_index - 1].index_count));
        next_face_index++;
    }
    assert(next_face_index == face_range_count);
    assert(face_ranges.back().first_index + face_ranges.back().index_count == index_count);
    cad_mesh_face_range first_face_range{};
    assert(cad_mesh_face_range_at(mesh, 0, &first_face_range) == CAD_OK);
    assert(first_face_range.face_index == 0);
    assert(cad_mesh_face_range_at(mesh, face_range_count, &first_face_range) ==
           CAD_ERROR_INVALID_ARGUMENT);

    std::uint32_t vertex_bytes_size = 0;
    std::uint32_t normal_bytes_size = 0;
    std::uint32_t index_bytes_size = 0;
    assert(cad_mesh_copy_vertices_bytes(mesh, nullptr, &vertex_bytes_size) ==
           CAD_ERROR_BUFFER_TOO_SMALL);
    assert(cad_mesh_copy_normals_bytes(mesh, nullptr, &normal_bytes_size) ==
           CAD_ERROR_BUFFER_TOO_SMALL);
    assert(cad_mesh_copy_indices_bytes(mesh, nullptr, &index_bytes_size) ==
           CAD_ERROR_BUFFER_TOO_SMALL);
    std::uint32_t edge_bytes_size = 0;
    assert(cad_mesh_copy_edge_segments_bytes(mesh, nullptr, &edge_bytes_size) ==
           CAD_ERROR_BUFFER_TOO_SMALL);
    assert(edge_bytes_size > 0 && edge_bytes_size % (2 * sizeof(cad_vec3)) == 0);
    assert(vertex_bytes_size == vertex_count * sizeof(cad_vec3));
    assert(normal_bytes_size == vertex_count * sizeof(cad_vec3));
    assert(index_bytes_size == index_count * sizeof(std::uint32_t));
    std::vector<std::uint8_t> vertex_bytes(vertex_bytes_size);
    std::vector<std::uint8_t> normal_bytes(normal_bytes_size);
    std::vector<std::uint8_t> index_bytes(index_bytes_size);
    std::vector<std::uint8_t> edge_bytes(edge_bytes_size);
    assert(cad_mesh_copy_vertices_bytes(mesh, vertex_bytes.data(), &vertex_bytes_size) == CAD_OK);
    assert(cad_mesh_copy_normals_bytes(mesh, normal_bytes.data(), &normal_bytes_size) == CAD_OK);
    assert(cad_mesh_copy_indices_bytes(mesh, index_bytes.data(), &index_bytes_size) == CAD_OK);
    assert(cad_mesh_copy_edge_segments_bytes(mesh, edge_bytes.data(), &edge_bytes_size) == CAD_OK);
    for (std::size_t offset = 0; offset < edge_bytes.size(); offset += sizeof(cad_vec3)) {
        cad_vec3 endpoint{};
        std::memcpy(&endpoint, edge_bytes.data() + offset, sizeof(endpoint));
        const auto on_face_mesh = std::any_of(vertices.begin(), vertices.end(), [&](const auto& vertex) {
            return std::abs(vertex.x - endpoint.x) < 1e-9 &&
                   std::abs(vertex.y - endpoint.y) < 1e-9 &&
                   std::abs(vertex.z - endpoint.z) < 1e-9;
        });
        assert(on_face_mesh);
    }
    assert(std::memcmp(vertex_bytes.data(), vertices.data(), vertex_bytes.size()) == 0);
    assert(std::memcmp(normal_bytes.data(), normals.data(), normal_bytes.size()) == 0);
    assert(std::memcmp(index_bytes.data(), indices.data(), index_bytes.size()) == 0);
    assert(cad_mesh_copy_vertices(mesh, vertices.data(), vertex_count - 1) ==
           CAD_ERROR_BUFFER_TOO_SMALL);
    for (uint32_t index = 0; index < vertex_count; ++index) {
        assert(indices[index] < vertex_count);
        const auto normal_length = std::sqrt(
            normals[index].x * normals[index].x +
            normals[index].y * normals[index].y +
            normals[index].z * normals[index].z);
        assert_close(normal_length, 1.0);
    }

    cad_mesh invalid_mesh = 99;
    const cad_mesh_options invalid_mesh_options{0.0, 0.5};
    assert(cad_shape_tessellate(shape, &invalid_mesh_options, &invalid_mesh) ==
           CAD_ERROR_INVALID_ARGUMENT);
    assert(invalid_mesh == 0);

    cad_shape cylinder = 0;
    assert(cad_cylinder(5.0, 20.0, &cylinder) == CAD_OK);
    assert(cad_shape_bounds(cylinder, &bounds) == CAD_OK);
    assert_close(bounds.min.x, -5.0);
    assert_close(bounds.min.y, -5.0);
    assert_close(bounds.min.z, 0.0);
    assert_close(bounds.max.x, 5.0);
    assert_close(bounds.max.y, 5.0);
    assert_close(bounds.max.z, 20.0);
    cad_mesh coarse_cylinder = 0;
    cad_mesh fine_cylinder = 0;
    const cad_mesh_options coarse_options{0.5, 0.75};
    const cad_mesh_options fine_options{0.02, 0.2};
    assert(cad_shape_tessellate(cylinder, &coarse_options, &coarse_cylinder) == CAD_OK);
    assert(cad_shape_tessellate(cylinder, &fine_options, &fine_cylinder) == CAD_OK);
    std::uint32_t coarse_edge_bytes = 0;
    std::uint32_t fine_edge_bytes = 0;
    assert(cad_mesh_copy_edge_segments_bytes(coarse_cylinder, nullptr, &coarse_edge_bytes) ==
           CAD_ERROR_BUFFER_TOO_SMALL);
    assert(cad_mesh_copy_edge_segments_bytes(fine_cylinder, nullptr, &fine_edge_bytes) ==
           CAD_ERROR_BUFFER_TOO_SMALL);
    assert(fine_edge_bytes > coarse_edge_bytes);
    cad_mesh_destroy(coarse_cylinder);
    cad_mesh_destroy(fine_cylinder);

    cad_shape sphere = 0;
    assert(cad_sphere(3.0, &sphere) == CAD_OK);
    assert(cad_shape_bounds(sphere, &bounds) == CAD_OK);
    assert_close(bounds.min.x, -3.0);
    assert_close(bounds.min.y, -3.0);
    assert_close(bounds.min.z, -3.0);
    assert_close(bounds.max.x, 3.0);
    assert_close(bounds.max.y, 3.0);
    assert_close(bounds.max.z, 3.0);

    cad_shape translated = 0;
    assert(cad_shape_translate(shape, {2.0, -3.0, 4.0}, &translated) == CAD_OK);
    assert(cad_shape_bounds(translated, &bounds) == CAD_OK);
    assert_close(bounds.min.x, 2.0);
    assert_close(bounds.min.y, -3.0);
    assert_close(bounds.min.z, 4.0);
    assert_close(bounds.max.x, 12.0);
    assert_close(bounds.max.y, 17.0);
    assert_close(bounds.max.z, 34.0);

    cad_shape profile = 0;
    assert(cad_shape_subshape_at(shape, CAD_SHAPE_FACE, 0, &profile) == CAD_OK);
    cad_vec3 profile_normal{};
    assert(cad_face_normal(profile, &profile_normal) == CAD_OK);
    const cad_vec3 profile_delta{
        profile_normal.x * 5.0,
        profile_normal.y * 5.0,
        profile_normal.z * 5.0};

    cad_shape extruded = 0;
    assert(cad_shape_extrude(profile, profile_delta, &extruded) == CAD_OK);
    assert(cad_shape_volume(extruded, &volume) == CAD_OK);
    assert(volume > 0.0);
    assert(cad_shape_bounds(extruded, &bounds) == CAD_OK);
    assert(bounds.max.z > bounds.min.z);
    cad_operation extrude_operation = 0;
    assert(cad_shape_extrude_operation(
               profile, profile_delta, &extrude_operation) == CAD_OK);
    cad_shape extrude_result = 0;
    assert(cad_operation_result_shape(extrude_operation, &extrude_result) == CAD_OK);
    assert(cad_shape_volume(extrude_result, &volume) == CAD_OK);
    assert(volume > 0.0);
    uint32_t extrude_generated = 0;
    uint32_t extrude_modified = 0;
    uint32_t extrude_deleted = 0;
    assert(cad_operation_history_count(
               extrude_operation, CAD_HISTORY_GENERATED, &extrude_generated) == CAD_OK);
    assert(cad_operation_history_count(
               extrude_operation, CAD_HISTORY_MODIFIED, &extrude_modified) == CAD_OK);
    assert(cad_operation_history_count(
               extrude_operation, CAD_HISTORY_DELETED, &extrude_deleted) == CAD_OK);
    assert(extrude_generated + extrude_modified + extrude_deleted > 0);

    cad_shape revolved = 0;
    constexpr double half_pi = 1.57079632679489661923;
    assert(cad_shape_revolve(
               profile,
               {0.0, 0.0, 0.0},
               {0.0, 0.0, 1.0},
               half_pi,
               &revolved) == CAD_OK);
    assert(cad_shape_volume(revolved, &volume) == CAD_OK);
    assert(volume > 0.0);
    cad_operation revolve_operation = 0;
    assert(cad_shape_revolve_operation(
               profile,
               {0.0, 0.0, 0.0},
               {0.0, 0.0, 1.0},
               half_pi,
               &revolve_operation) == CAD_OK);
    cad_shape revolve_result = 0;
    assert(cad_operation_result_shape(revolve_operation, &revolve_result) == CAD_OK);
    assert(cad_shape_volume(revolve_result, &volume) == CAD_OK);
    assert(volume > 0.0);

    cad_shape filleted = 0;
    assert(cad_shape_fillet(shape, 1.0, &filleted) == CAD_OK);
    assert(cad_shape_volume(filleted, &volume) == CAD_OK);
    assert(volume > 0.0 && volume < 6000.0);
    const double all_fillet_volume = volume;
    cad_operation fillet_operation = 0;
    assert(cad_shape_fillet_operation(shape, 1.0, &fillet_operation) == CAD_OK);
    cad_shape fillet_result = 0;
    assert(cad_operation_result_shape(fillet_operation, &fillet_result) == CAD_OK);
    assert(cad_shape_volume(fillet_result, &volume) == CAD_OK);
    assert(volume > 0.0 && volume < 6000.0);
    uint32_t fillet_generated = 0;
    uint32_t fillet_modified = 0;
    uint32_t fillet_deleted = 0;
    assert(cad_operation_history_count(
               fillet_operation, CAD_HISTORY_GENERATED, &fillet_generated) == CAD_OK);
    assert(cad_operation_history_count(
               fillet_operation, CAD_HISTORY_MODIFIED, &fillet_modified) == CAD_OK);
    assert(cad_operation_history_count(
               fillet_operation, CAD_HISTORY_DELETED, &fillet_deleted) == CAD_OK);
    assert(fillet_generated + fillet_modified + fillet_deleted > 0);

    cad_shape chamfered = 0;
    assert(cad_shape_chamfer(shape, 1.0, &chamfered) == CAD_OK);
    assert(cad_shape_volume(chamfered, &volume) == CAD_OK);
    assert(volume > 0.0 && volume < 6000.0);
    cad_operation chamfer_operation = 0;
    assert(cad_shape_chamfer_operation(shape, 1.0, &chamfer_operation) == CAD_OK);
    cad_shape chamfer_result = 0;
    assert(cad_operation_result_shape(chamfer_operation, &chamfer_result) == CAD_OK);
    assert(cad_shape_volume(chamfer_result, &volume) == CAD_OK);
    assert(volume > 0.0 && volume < 6000.0);

    cad_shape selected_edge = 0;
    assert(cad_shape_subshape_at(shape, CAD_SHAPE_EDGE, 0, &selected_edge) == CAD_OK);
    const cad_shape_ref selected_edges[] = {{selected_edge}};
    cad_shape selected_filleted = 0;
    assert(cad_shape_fillet_edges(
               shape, selected_edges, 1, 1.0, &selected_filleted) == CAD_OK);
    double selected_fillet_volume = 0.0;
    assert(cad_shape_volume(selected_filleted, &selected_fillet_volume) == CAD_OK);
    assert(selected_fillet_volume > all_fillet_volume);
    assert(selected_fillet_volume < 6000.0);

    cad_operation selected_fillet_operation = 0;
    assert(cad_shape_fillet_edges_operation(
               shape,
               selected_edges,
               1,
               1.0,
               &selected_fillet_operation) == CAD_OK);
    cad_shape selected_fillet_result = 0;
    assert(cad_operation_result_shape(
               selected_fillet_operation, &selected_fillet_result) == CAD_OK);
    assert(cad_shape_volume(selected_fillet_result, &selected_fillet_volume) == CAD_OK);
    assert(selected_fillet_volume > all_fillet_volume);
    assert(cad_operation_history_count(
               selected_fillet_operation,
               CAD_HISTORY_GENERATED,
               &generated_count) == CAD_OK);
    assert(cad_operation_history_count(
               selected_fillet_operation,
               CAD_HISTORY_MODIFIED,
               &modified_count) == CAD_OK);
    assert(cad_operation_history_count(
               selected_fillet_operation,
               CAD_HISTORY_DELETED,
               &deleted_count) == CAD_OK);
    assert(generated_count + modified_count + deleted_count > 0);

    cad_shape selected_chamfered = 0;
    assert(cad_shape_chamfer_edges(
               shape, selected_edges, 1, 1.0, &selected_chamfered) == CAD_OK);
    assert(cad_shape_volume(selected_chamfered, &volume) == CAD_OK);
    assert(volume > 0.0 && volume < 6000.0);
    cad_operation selected_chamfer_operation = 0;
    assert(cad_shape_chamfer_edges_operation(
               shape,
               selected_edges,
               1,
               1.0,
               &selected_chamfer_operation) == CAD_OK);
    cad_shape selected_chamfer_result = 0;
    assert(cad_operation_result_shape(
               selected_chamfer_operation, &selected_chamfer_result) == CAD_OK);
    assert(cad_shape_volume(selected_chamfer_result, &volume) == CAD_OK);
    assert(volume > 0.0 && volume < 6000.0);
    assert(cad_operation_history_count(
               selected_chamfer_operation,
               CAD_HISTORY_GENERATED,
               &generated_count) == CAD_OK);
    assert(cad_operation_history_count(
               selected_chamfer_operation,
               CAD_HISTORY_MODIFIED,
               &modified_count) == CAD_OK);
    assert(cad_operation_history_count(
               selected_chamfer_operation,
               CAD_HISTORY_DELETED,
               &deleted_count) == CAD_OK);
    assert(generated_count + modified_count + deleted_count > 0);

    cad_shape rotated = 0;
    constexpr double pi = 3.14159265358979323846;
    assert(cad_shape_rotate(shape, {0.0, 0.0, 1.0}, pi / 2.0, &rotated) == CAD_OK);
    assert(cad_shape_bounds(rotated, &bounds) == CAD_OK);
    assert_close(bounds.min.x, -20.0);
    assert_close(bounds.min.y, 0.0);
    assert_close(bounds.min.z, 0.0);
    assert_close(bounds.max.x, 0.0);
    assert_close(bounds.max.y, 10.0);
    assert_close(bounds.max.z, 30.0);

    assert(cad_cylinder(0.0, 1.0, &cylinder) == CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_shape_translate(shape, {NAN, 0.0, 0.0}, &translated) == CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_shape_rotate(shape, {0.0, 0.0, 0.0}, 1.0, &rotated) == CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_shape_extrude(shape, {0.0, 0.0, 0.0}, &extruded) == CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_shape_revolve(
               shape, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1.0, &revolved) ==
           CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_shape_revolve(
               shape, {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 0.0, &revolved) ==
           CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_shape_fillet(shape, 0.0, &filleted) == CAD_ERROR_INVALID_ARGUMENT);
    assert(cad_shape_chamfer(shape, 0.0, &chamfered) == CAD_ERROR_INVALID_ARGUMENT);
    cad_shape invalid_selected_result = 0;
    assert(cad_shape_fillet_edges(
               shape, nullptr, 0, 1.0, &invalid_selected_result) ==
           CAD_ERROR_INVALID_ARGUMENT);
    const cad_shape_ref invalid_selected_edges[] = {{shape}};
    assert(cad_shape_chamfer_edges(
               shape,
               invalid_selected_edges,
               1,
               1.0,
               &invalid_selected_result) == CAD_ERROR_INVALID_ARGUMENT);
    const cad_shape_ref duplicate_selected_edges[] = {{selected_edge}, {selected_edge}};
    assert(cad_shape_fillet_edges(
               shape,
               duplicate_selected_edges,
               2,
               1.0,
               &invalid_selected_result) == CAD_ERROR_INVALID_ARGUMENT);
    cad_shape foreign_edge = 0;
    assert(cad_shape_subshape_at(boolean_base, CAD_SHAPE_EDGE, 0, &foreign_edge) == CAD_OK);
    const cad_shape_ref foreign_selected_edges[] = {{foreign_edge}};
    assert(cad_shape_fillet_edges(
               shape,
               foreign_selected_edges,
               1,
               1.0,
               &invalid_selected_result) == CAD_ERROR_INVALID_ARGUMENT);
    cad_shape_destroy(foreign_edge);

    cad_shape_destroy(extruded);
    cad_shape_destroy(extrude_result);
    cad_shape_destroy(revolved);
    cad_shape_destroy(revolve_result);
    cad_shape_destroy(filleted);
    cad_shape_destroy(fillet_result);
    cad_shape_destroy(chamfered);
    cad_shape_destroy(chamfer_result);
    cad_shape_destroy(selected_edge);
    cad_shape_destroy(selected_filleted);
    cad_shape_destroy(selected_fillet_result);
    cad_shape_destroy(selected_chamfered);
    cad_shape_destroy(selected_chamfer_result);
    cad_shape_destroy(profile);
    cad_shape_destroy(cylinder);
    cad_shape_destroy(sphere);
    cad_shape_destroy(translated);
    cad_shape_destroy(rotated);
    cad_shape_destroy(fused);
    cad_shape_destroy(cut);
    cad_shape_destroy(common);
    cad_shape_destroy(boolean_tool_offset);
    cad_shape_destroy(boolean_tool);
    cad_shape_destroy(boolean_base);
    cad_shape_destroy(shape);
    cad_mesh_destroy(mesh);
    cad_operation_destroy(translate_operation);
    cad_operation_destroy(extrude_operation);
    cad_operation_destroy(revolve_operation);
    cad_operation_destroy(fillet_operation);
    cad_operation_destroy(chamfer_operation);
    cad_operation_destroy(selected_fillet_operation);
    cad_operation_destroy(selected_chamfer_operation);
    assert(cad_shape_bounds(shape, &bounds) == CAD_ERROR_INVALID_HANDLE);
    assert(cad_mesh_vertex_count(mesh, &vertex_count) == CAD_ERROR_INVALID_HANDLE);
    assert(cad_operation_history_count(
               translate_operation, CAD_HISTORY_MODIFIED, &modified_count) ==
           CAD_ERROR_INVALID_HANDLE);
    assert(cad_last_error()[0] != '\0');

    cad_shape stale_after_wrap = 0;
    assert(cad_compound(nullptr, 0, &stale_after_wrap) == CAD_OK);
    cad_shape_destroy(stale_after_wrap);
    // Exercise more releases than the former 16-bit generation could survive.
    for (std::uint32_t index = 0; index <= 65534; ++index) {
        cad_shape churn = 0;
        assert(cad_compound(nullptr, 0, &churn) == CAD_OK);
        if (index == 65534) {
            cad_shape_kind stale_kind = CAD_SHAPE_UNKNOWN;
            assert(cad_shape_kind_get(stale_after_wrap, &stale_kind) == CAD_ERROR_INVALID_HANDLE);
        }
        cad_shape_destroy(churn);
    }

    return 0;
}

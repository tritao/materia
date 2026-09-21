#include "internal.hpp"

#include <algorithm>
#include <cmath>

namespace {

nksim_result create_shape(nksim_world world, const nksim_shape_desc &desc,
                          nksim_shape *out_shape) {
    const auto value = nksim::resolve_world(world);
    return value ? value->create_shape(desc, out_shape) : NKSIM_ERROR_INVALID_HANDLE;
}

} // namespace

extern "C" {

void NKSIM_CALL nksim_shape_destroy(nksim_world world, nksim_shape shape) {
    const auto value = nksim::resolve_world(world);
    if (value)
        value->destroy_shape(shape);
}

nksim_result NKSIM_CALL nksim_shape_create(nksim_world world, const nksim_shape_desc *desc,
                                           nksim_shape *out_shape) {
    if (!desc || !out_shape)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    return create_shape(world, *desc, out_shape);
}

nksim_result NKSIM_CALL nksim_shape_create_box(nksim_world world, const double half_extents[3],
                                               nksim_shape *out_shape) {
    if (!half_extents || !out_shape || !std::isfinite(half_extents[0]) ||
        !std::isfinite(half_extents[1]) || !std::isfinite(half_extents[2]) ||
        half_extents[0] <= 0.0 || half_extents[1] <= 0.0 || half_extents[2] <= 0.0)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    nksim_shape_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.type = NKSIM_SHAPE_BOX;
    std::copy(half_extents, half_extents + 3, desc.parameters);
    return create_shape(world, desc, out_shape);
}

nksim_result NKSIM_CALL nksim_shape_create_sphere(nksim_world world, double radius,
                                                  nksim_shape *out_shape) {
    if (!out_shape || !std::isfinite(radius) || radius <= 0.0)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    nksim_shape_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.type = NKSIM_SHAPE_SPHERE;
    desc.parameters[0] = radius;
    return create_shape(world, desc, out_shape);
}

nksim_result NKSIM_CALL nksim_shape_create_capsule(nksim_world world, double radius,
                                                   double height, nksim_shape *out_shape) {
    if (!out_shape || !std::isfinite(radius) || !std::isfinite(height) || radius <= 0.0 ||
        height <= 0.0)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    nksim_shape_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.type = NKSIM_SHAPE_CAPSULE;
    desc.parameters[0] = radius;
    desc.parameters[1] = height;
    return create_shape(world, desc, out_shape);
}

nksim_result NKSIM_CALL nksim_shape_create_plane(nksim_world world, const double normal[3],
                                                 double offset, nksim_shape *out_shape) {
    if (!normal || !out_shape || !std::isfinite(offset))
        return NKSIM_ERROR_INVALID_ARGUMENT;
    const double length = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
                                     normal[2] * normal[2]);
    if (!std::isfinite(length) || length <= 0.0)
        return NKSIM_ERROR_INVALID_ARGUMENT;
    nksim_shape_desc desc{};
    desc.struct_size = sizeof(desc);
    desc.type = NKSIM_SHAPE_PLANE;
    desc.parameters[0] = normal[0] / length;
    desc.parameters[1] = normal[1] / length;
    desc.parameters[2] = normal[2] / length;
    desc.parameters[3] = offset;
    return create_shape(world, desc, out_shape);
}

} // extern "C"

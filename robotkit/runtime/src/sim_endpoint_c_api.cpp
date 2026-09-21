#include "robotkit_simkit.h"

#include "runtime_registry.hpp"
#include "sim_endpoint.hpp"

#include <memory>

extern "C" {

rk_result RK_CALL rk_runtime_create_sim(const rk_runtime_blueprint *blueprint,
                                        rk_runtime *out_runtime) {
    if (!out_runtime || rk_runtime_blueprint_validate(blueprint) != RK_OK)
        return RK_ERROR_INVALID_ARGUMENT;
    *out_runtime = RK_INVALID_RUNTIME;
    try {
        rk_runtime_layout layout{};
        layout.struct_size = sizeof(layout);
        layout.revision = blueprint->revision;
        layout.joint_count = blueprint->joint_count;
        layout.link_count = blueprint->link_count;
        layout.frame_count = blueprint->frame_count;
        auto endpoint = std::make_unique<robotkit::SimEndpoint>(*blueprint);
        auto runtime = std::make_shared<robotkit::Runtime>(layout,
                                                            std::move(endpoint));
        *out_runtime = robotkit::internal::register_runtime(std::move(runtime));
        return RK_OK;
    } catch (...) {
        return RK_ERROR_BACKEND;
    }
}

} // extern "C"

#ifndef ROBOTKIT_SIM_ENDPOINT_HPP
#define ROBOTKIT_SIM_ENDPOINT_HPP

#include "robotkit_core.h"
#include "robotkit_runtime.hpp"

#include "nativekit_sim.h"
#include "nativekit_sim_host.h"

#include <vector>

namespace robotkit {

class SimEndpoint final : public Endpoint {
public:
    explicit SimEndpoint(const rk_runtime_blueprint &blueprint);
    ~SimEndpoint() override;

    SimEndpoint(const SimEndpoint &) = delete;
    SimEndpoint &operator=(const SimEndpoint &) = delete;

    rk_result apply(const rk_robot_command &command) override;
    rk_result step(uint64_t timestamp_ns, rk_robot_state &state) override;

private:
    void initialize(const rk_runtime_blueprint &blueprint);
    void cleanup() noexcept;

    nkscene_scene scene_ = 0;
    nksim_world world_ = 0;
    nksim_shape shape_ = 0;
    nksim_host host_ = 0;
    std::vector<nkscene_occurrence_id> occurrences_;
    std::vector<nksim_body> bodies_;
    std::vector<nksim_joint> joints_;
    bool stopped_ = false;
};

} // namespace robotkit

#endif

#include "trajectory_core.h"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <initializer_list>

int main(int argc, char **argv) {
    assert(argc == 2);
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.t0_ns = 0;
    segment.duration_ns = 1'000'000'000;
    segment.degree = 5;
    segment.joint_count = 1;
    double coefficients[] = {0.1, 1.25, -0.3, 0.2, -0.05, 0.005};
    for (int i = 0; i <= 5; ++i) segment.coefficients[0].value[i] = coefficients[i];
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    std::ifstream vectors(argv[1]);
    assert(vectors.good());
    int tick = 0, count = 0;
    double position = 0, velocity = 0;
    while (vectors >> tick >> position >> velocity) {
        mk_trajectory_state state{};
        state.struct_size = sizeof(state);
        assert(mk_trajectory_evaluate(trajectory, std::int64_t(tick) * 1'000'000,
                                      &state) == MK_OK);
        assert(std::abs(state.position[0] - position) < 1e-12);
        assert(std::abs(state.velocity[0] - velocity) < 1e-12);
        ++count;
    }
    assert(count == 6);
    mk_trajectory_destroy(trajectory);
}

#include "motionkit.h"

#include <cassert>
#include <cmath>
#include <limits>

namespace {

void near(double actual, double expected) {
    assert(std::abs(actual - expected) <= 1e-12);
}

void cubic_and_boundaries() {
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.t0_ns = 0;
    segment.duration_ns = 1'000'000'000;
    segment.degree = 3;
    segment.joint_count = 1;
    segment.coefficients[0].value[0] = 1.0;
    segment.coefficients[0].value[1] = 2.0;
    segment.coefficients[0].value[2] = 3.0;
    segment.coefficients[0].value[3] = 4.0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    mk_trajectory_state state{};
    state.struct_size = sizeof(state);
    assert(mk_trajectory_evaluate(trajectory, 500'000'000, &state) == MK_OK);
    near(state.position[0], 3.25);
    near(state.velocity[0], 8.0);
    near(state.acceleration[0], 18.0);
    near(state.jerk[0], 24.0);

    segment.t0_ns = 1'000'000'000;
    segment.degree = 1;
    segment.coefficients[0].value[0] = 10.0;
    segment.coefficients[0].value[1] = -2.0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    assert(mk_trajectory_evaluate(trajectory, 1'000'000'000, &state) == MK_OK);
    near(state.position[0], 10.0);
    near(state.velocity[0], -2.0);
    mk_continuity continuity{};
    continuity.struct_size = sizeof(continuity);
    assert(mk_trajectory_boundary_continuity(trajectory, 0, &continuity) == MK_OK);
    near(continuity.c0_jump, 0.0); // First cubic ends at 10.
    near(continuity.c1_jump, 22.0); // Its end velocity is 20.
    near(continuity.c2_jump, 30.0);
    int64_t duration = 0;
    uint32_t segment_count = 0;
    assert(mk_trajectory_duration_ns(trajectory, &duration) == MK_OK);
    assert(duration == 2'000'000'000);
    assert(mk_trajectory_segment_count(trajectory, &segment_count) == MK_OK);
    assert(segment_count == 2);
    mk_trajectory_destroy(trajectory);
}

void samples_and_rejections() {
    mk_sample samples[3]{};
    for (auto &sample : samples) {
        sample.struct_size = sizeof(sample);
        sample.joint_count = 1;
    }
    samples[0].position[0] = 0.0;
    samples[1].time_ns = 1'000'000'000;
    samples[1].position[0] = 2.0;
    samples[2].time_ns = 2'000'000'000;
    samples[2].position[0] = 5.0;
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_from_samples(1, samples, 3, &trajectory) == MK_OK);
    mk_trajectory_state state{};
    state.struct_size = sizeof(state);
    assert(mk_trajectory_evaluate(trajectory, 500'000'000, &state) == MK_OK);
    near(state.position[0], 1.0);
    near(state.velocity[0], 2.0);
    near(state.acceleration[0], 0.0);
    assert(mk_trajectory_evaluate(trajectory, 1'000'000'000, &state) == MK_OK);
    near(state.velocity[0], 3.0); // Right-continuous at the knot.
    mk_trajectory_destroy(trajectory);

    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.joint_count = 1;
    segment.duration_ns = 1;
    segment.degree = 1;
    segment.coefficients[0].value[0] = std::numeric_limits<double>::quiet_NaN();
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_ERROR_INVALID_ARGUMENT);
    segment.coefficients[0].value[0] = 0.0;
    segment.duration_ns = 0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_ERROR_INVALID_ARGUMENT);
    segment.duration_ns = -1;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_ERROR_INVALID_ARGUMENT);
    segment.duration_ns = 1;
    segment.degree = 6;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_ERROR_INVALID_ARGUMENT);
    segment.degree = 1;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    segment.t0_ns = 2;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_ERROR_INVALID_ARGUMENT);
    mk_trajectory_destroy(trajectory);
}

} // namespace

int main() {
    cubic_and_boundaries();
    samples_and_rejections();
}

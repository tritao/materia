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

void path_derivative_estimate_selects_segment_degree() {
    mk_trajectory_handle trajectory{};
    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    mk_segment segment{};
    segment.struct_size = sizeof(segment);
    segment.duration_ns = 100'000'000;
    segment.degree = 1;
    segment.joint_count = 1;
    segment.coefficients[0].value[1] = 1.0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    segment.t0_ns = 100'000'000;
    segment.coefficients[0].value[0] = 0.1;
    segment.coefficients[0].value[1] = 0.0;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    mk_path_derivative_estimate estimate{};
    estimate.struct_size = sizeof(estimate);
    assert(mk_trajectory_estimate_path_derivatives(trajectory, 50'000'000,
        100'000'000, &estimate) == MK_OK);
    near(estimate.velocity[0], 1.0);
    near(estimate.acceleration[0], -10.0);
    mk_trajectory_destroy(trajectory);

    assert(mk_trajectory_create(1, &trajectory) == MK_OK);
    segment = {};
    segment.struct_size = sizeof(segment);
    segment.duration_ns = 1'000'000'000;
    segment.degree = 2;
    segment.joint_count = 1;
    segment.coefficients[0].value[2] = 0.5;
    assert(mk_trajectory_append_segment(trajectory, &segment) == MK_OK);
    assert(mk_trajectory_estimate_path_derivatives(trajectory, 200'000'000,
        100'000'000, &estimate) == MK_OK);
    near(estimate.velocity[0], 0.2);
    near(estimate.acceleration[0], 1.0);
    mk_trajectory_destroy(trajectory);
}

void owned_ranges() {
    mk_trajectory_handle source{};
    assert(mk_trajectory_create(1,&source)==MK_OK);
    mk_segment piece{}; piece.struct_size=sizeof(piece); piece.joint_count=1;
    piece.t0_ns=-125'000'000; piece.duration_ns=1'000'000'000; piece.degree=5;
    const double first[]={1,2,3,4,0.5,0.25}, second[]={10.75,23.25,20.5,8.5,1.75,0.25};
    for (int i=0;i<6;++i) piece.coefficients[0].value[i]=first[i];
    assert(mk_trajectory_append_segment(source,&piece)==MK_OK);
    piece.t0_ns+=piece.duration_ns;
    for (int i=0;i<6;++i) piece.coefficients[0].value[i]=second[i];
    assert(mk_trajectory_append_segment(source,&piece)==MK_OK);
    int64_t interior=-1; uint32_t found=0;
    assert(mk_trajectory_interior_time(source,1'000'000'000,&interior,&found)==MK_OK && found==1 && interior==999'999'999);
    assert(mk_trajectory_interior_time(source,1'000'000'001,&interior,&found)==MK_OK && found==1 && interior==1'000'000'001);
    assert(mk_trajectory_interior_time(source,0,&interior,&found)==MK_OK && found==0 && interior==0);
    const uint32_t joints[]={0,0};
    mk_trajectory_handle selected{},rejected{123};
    assert(mk_trajectory_select_joints(source,joints,2,&selected)==MK_OK);
    const uint32_t invalid_joint=1;
    assert(mk_trajectory_select_joints(source,&invalid_joint,1,&rejected)==MK_ERROR_INVALID_ARGUMENT && rejected.id==0);
    mk_segment selected_piece{}; selected_piece.struct_size=sizeof(selected_piece);
    assert(mk_trajectory_get_segment(selected,0,&selected_piece)==MK_OK);
    assert(selected_piece.t0_ns==piece.t0_ns-piece.duration_ns && selected_piece.joint_count==2);
    for (int p=0;p<6;++p) {
        near(selected_piece.coefficients[0].value[p],first[p]);
        near(selected_piece.coefficients[1].value[p],first[p]);
    }
    mk_trajectory_destroy(selected);
    mk_trajectory_handle sliced{123};
    assert(mk_trajectory_slice(source,250'000'000,1'750'000'000,&sliced)==MK_OK);
    int64_t duration=0;
    assert(mk_trajectory_duration_ns(sliced,&duration)==MK_OK && duration==1'500'000'000);
    for (int i=0;i<=60;++i) {
        const int64_t time=i*25'000'000LL;
        mk_trajectory_state a{},b{}; a.struct_size=sizeof(a); b.struct_size=sizeof(b);
        assert(mk_trajectory_evaluate(source,125'000'000+time,&a)==MK_OK);
        assert(mk_trajectory_evaluate(sliced,time,&b)==MK_OK);
        near(a.position[0],b.position[0]); near(a.velocity[0],b.velocity[0]);
        near(a.acceleration[0],b.acceleration[0]); near(a.jerk[0],b.jerk[0]);
    }
    mk_trajectory_handle invalid{123};
    assert(mk_trajectory_slice(source,-1,1,&invalid)==MK_ERROR_INVALID_ARGUMENT && invalid.id==0);
    invalid.id=123;
    assert(mk_trajectory_slice(source,0,2'000'000'001,&invalid)==MK_ERROR_INVALID_ARGUMENT && invalid.id==0);
    invalid.id=123;
    assert(mk_trajectory_slice({},0,1,&invalid)==MK_ERROR_INVALID_HANDLE && invalid.id==0);
    mk_trajectory_handle joined{},wrong{};
    assert(mk_trajectory_create(1,&joined)==MK_OK && mk_trajectory_create(2,&wrong)==MK_OK);
    assert(mk_trajectory_append_copy(joined,sliced)==MK_OK);
    assert(mk_trajectory_append_copy(joined,wrong)==MK_ERROR_INVALID_ARGUMENT);
    assert(mk_trajectory_append_copy(joined,{})==MK_ERROR_INVALID_HANDLE);
    assert(mk_trajectory_duration_ns(joined,&duration)==MK_OK && duration==1'500'000'000);
    assert(mk_trajectory_append_copy(joined,joined)==MK_OK);
    assert(mk_trajectory_duration_ns(joined,&duration)==MK_OK && duration==3'000'000'000);
    mk_trajectory_destroy(source); mk_trajectory_destroy(sliced); mk_trajectory_destroy(wrong);
    mk_trajectory_state a{},b{}; a.struct_size=sizeof(a); b.struct_size=sizeof(b);
    assert(mk_trajectory_evaluate(joined,250'000'000,&a)==MK_OK);
    assert(mk_trajectory_evaluate(joined,1'750'000'000,&b)==MK_OK);
    near(a.position[0],b.position[0]); near(a.velocity[0],b.velocity[0]);
    mk_trajectory_destroy(joined);
}

} // namespace

int main() {
    owned_ranges();
    cubic_and_boundaries();
    samples_and_rejections();
    path_derivative_estimate_selects_segment_degree();
}

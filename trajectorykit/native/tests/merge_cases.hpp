#pragma once
#include "trajectory_merge.hpp"
#include <cassert>
#include <cmath>
#include <vector>

// The same behavioral corpus runs against compact planner storage and borrowed RobotKit arrays.
template<class Merge> void merge_cases(Merge merge) {
    const double pt[] = {1e-6}, vt[] = {1e-5}, at[] = {1e-4};
    const double vl[] = {1}, al[] = {10}, lo[] = {-1}, hi[] = {1};
    motionkit::MergeOptions options{pt, vt, at, vl, al, lo, hi, 0.1};
    std::vector<mk_segment> source(40);
    for (unsigned i = 0; i < source.size(); ++i) {
        auto &s = source[i];
        s.struct_size = sizeof(s); s.joint_count = 1; s.degree = 2;
        s.t0_ns = i * 1000000; s.duration_ns = 1000000;
        const double t = i * .001;
        s.coefficients[0].value[0] = .5 * t * t;
        s.coefficients[0].value[1] = t;
        s.coefficients[0].value[2] = .5;
    }
    auto out = merge(source, options);
    assert(out.size() == 1 && out[0].duration_ns == 40000000);
    mk_trajectory_state state{};
    for (unsigned i = 0; i <= 100; ++i) {
        const double t = .04 * i / 100;
        motionkit::evaluate_segment(out[0], t, state);
        assert(std::abs(state.position[0] - .5 * t * t) < 1e-12);
        assert(std::abs(state.velocity[0] - t) < 1e-12);
    }
    // A zero allowance is an exact hold, not an invalid or unlimited budget.
    {
        auto held = source;
        for(auto &piece : held){piece.coefficients[0]={};piece.coefficients[0].value[0]=0.125;}
        const double exact[] = {0};
        auto exact_options=options;
        exact_options.position_tolerance=exact_options.velocity_tolerance=exact_options.acceleration_tolerance=exact;
        auto merged=merge(held,exact_options);
        assert(merged.size()==1);
        for(double value : merged[0].coefficients[0].value) assert(std::isfinite(value));
        assert(merged[0].coefficients[0].value[0]==0.125);
        for(unsigned k=1;k<=merged[0].degree;++k)assert(merged[0].coefficients[0].value[k]==0);
    }
    // A short quartic excursion hides between the old strided joins and fifteen
    // uniform samples. Its source velocity/acceleration stay below the limits,
    // but replacing the whole run with zero loses twice the position budget.
    {
        auto pulse_options = options;
        pulse_options.max_duration_seconds = 0.3;
        const int64_t piece_ns = 4'000'000;
        const double h = 0.004, amplitude = 2e-6;
        std::vector<mk_segment> pulse(64);
        for (unsigned i = 0; i < pulse.size(); ++i) {
            auto &s = pulse[i];
            s.struct_size = sizeof(s); s.joint_count = 1; s.degree = 4;
            s.t0_ns = i * piece_ns; s.duration_ns = piece_ns;
        }
        // 16*A*x^2*(1-x)^2, x=t/h: position/velocity agree with rest on both sides.
        auto &c = pulse[41].coefficients[0].value;
        c[2] = 16 * amplitude / (h*h);
        c[3] = -32 * amplitude / (h*h*h);
        c[4] = 16 * amplitude / (h*h*h*h);
        const double unlimited[] = {std::numeric_limits<double>::infinity()};
        for (int order = 0; order < 3; ++order) {
            pulse_options.position_tolerance = order == 0 ? pt : unlimited;
            pulse_options.velocity_tolerance = order == 1 ? vt : unlimited;
            pulse_options.acceleration_tolerance = order == 2 ? at : unlimited;
            const auto preserved = merge(pulse, pulse_options);
            std::size_t selected = 0;
            for (int sample = 0; sample < 4096; ++sample) {
                const int64_t t = 64 * piece_ns * sample / 4096;
                while (selected+1 < preserved.size() && t >= preserved[selected].t0_ns+preserved[selected].duration_ns) ++selected;
                const auto &piece = preserved[selected], &authored = pulse[t/piece_ns];
                mk_trajectory_state expected{};
                motionkit::evaluate_segment(authored, static_cast<double>(t-authored.t0_ns)*1e-9, expected);
                motionkit::evaluate_segment(piece, static_cast<double>(t-piece.t0_ns)*1e-9, state);
                const double error = order == 0 ? std::abs(state.position[0]-expected.position[0]) :
                    order == 1 ? std::abs(state.velocity[0]-expected.velocity[0]) : std::abs(state.acceleration[0]-expected.acceleration[0]);
                assert(error <= (order == 0 ? pt[0] : order == 1 ? vt[0] : at[0]) + 1e-12);
            }
        }
        // An initially inconclusive Bernstein hull must refine and accept a
        // genuinely permitted excursion, rather than refusing every curved run.
        const double allowed_pulse[] = {2.1e-6};
        pulse_options.position_tolerance = allowed_pulse;
        pulse_options.velocity_tolerance = pulse_options.acceleration_tolerance = unlimited;
        assert(merge(pulse, pulse_options).size() == 1);
    }
    // Chords do not claim smooth acceleration and must keep their authored corners.
    for (auto &s : source) s.degree = 1;
    assert(merge(source, options).size() == source.size());
    // A joint cruising on its velocity ceiling must never gain an interior speed peak.
    for (unsigned i = 0; i < source.size(); ++i) {
        auto &s = source[i]; s.degree = 5;
        s.coefficients[0] = {}; s.coefficients[0].value[0] = i * .001;
        s.coefficients[0].value[1] = 1;
    }
    out = merge(source, options);
    assert(out.size() == 1);
    for (unsigned i = 0; i <= 1000; ++i) {
        motionkit::evaluate_segment(out[0], .04 * i / 1000, state);
        assert(state.velocity[0] <= 1 + 1e-9);
    }
    // A hard reversal cannot disappear inside a permissive link window.
    source.resize(2);
    for (unsigned i = 0; i < 2; ++i) {
        source[i].degree = 2; source[i].coefficients[0] = {};
        source[i].coefficients[0].value[0] = i * .001;
        source[i].coefficients[0].value[1] = i ? -1 : 1;
    }
    assert(merge(source, options).size() == 2);
    // A clock sliver is not permission to erase an acceleration transition. These
    // two 2 us parabolas agree in position and velocity but change acceleration.
    // Their endpoint-state quintic violates the acceleration approximation budget.
    for (unsigned i = 0; i < 2; ++i) {
        auto &s = source[i];
        s.t0_ns = i * 2000; s.duration_ns = 2000; s.degree = 2;
        s.coefficients[0] = {};
        s.coefficients[0].value[0] = i ? 2e-12 : 0;
        s.coefficients[0].value[1] = i ? 2e-6 : 0;
        s.coefficients[0].value[2] = i ? -0.5 : 0.5;
    }
    assert(merge(source, options).size() == 2);
    // Even an exact parabola cannot exceed the caller's maximum merge duration.
    source[1].coefficients[0].value[2] = 0.5;
    assert(merge(source, options).size() == 1);
    options.max_duration_seconds = 3e-6;
    assert(merge(source, options).size() == 2);
    // Two monotone halves stay inside [0,1]. Their common endpoint-state quintic reaches 1.25.
    // Even permissive approximation tolerances must not authorize that travel overshoot.
    const double loose_p[] = {10}, loose_v[] = {10}, loose_a[] = {100};
    options.position_tolerance = loose_p; options.velocity_tolerance = loose_v;
    options.acceleration_tolerance = loose_a; options.max_duration_seconds = 2;
    const double fast_v[] = {100}, fast_a[] = {100};
    options.velocity_limit = fast_v; options.acceleration_limit = fast_a;
    for (unsigned i = 0; i < 2; ++i) {
        auto &s = source[i]; s.t0_ns = i * 500000000; s.duration_ns = 500000000; s.degree = 4;
        s.coefficients[0] = {}; s.coefficients[0].value[0] = i;
        s.coefficients[0].value[1] = i ? 0 : 4;
        s.coefficients[0].value[3] = -16; s.coefficients[0].value[4] = 16;
    }
    assert(merge(source, options).size() == 2);

}

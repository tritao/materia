#include "motionkit.h"
// Keep round-trip calls and checks active in the required Release gate.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>

int main() {
    mk_analytic_cartesian_model m = {}; m.struct_size = sizeof(m);
    // Rotated Cartesian basis, translated/displaced rotary head and a pitched
    // zero-pose tool. Neither origin nor initial reference is canonical.
    double angle = 0.37, c = std::cos(angle), s = std::sin(angle);
    double axes[] = {c,s,0, -s,c,0, 0,0,1};
    for (unsigned i = 0; i < 9; ++i) m.translation_axes[i] = axes[i];
    m.rotary_axes[2] = 1; m.rotary_axes[4] = 1;
    m.rotary_origins[0] = 0.12; m.rotary_origins[1] = -0.23; m.rotary_origins[2] = 0.34;
    m.rotary_origins[3] = 0.21; m.rotary_origins[4] = -0.11; m.rotary_origins[5] = 0.46;
    m.home_position[0] = 0.22; m.home_position[1] = -0.33; m.home_position[2] = 0.57;
    m.home_quaternion[1] = std::sin(0.2); m.home_quaternion[3] = std::cos(0.2);
    for (unsigned n = 3; n <= 5; ++n) {
        m.joint_count = n;
        for (unsigned sample = 0; sample < 100; ++sample) {
            double q[6] = {};
            for (unsigned i = 0; i < n; ++i) q[i] = 1.4 * std::sin((sample + 1) * (i + 1) * 1.618);
            mk_analytic_pose target = {};
            assert(mk_analytic_cartesian_forward(&m, q, n, &target) == MK_OK);
            for (unsigned free = 0; free < 2; ++free) {
                mk_analytic_solution answers[2] = {}; uint32_t count = 0;
                assert(mk_analytic_cartesian_inverse(&m, &target, free, q[3], answers, 2, &count) == MK_OK);
                assert(count > 0 && count <= 2);
                if (n == 5 && free) assert(count == 2);
                bool contains_original = false;
                for (unsigned j = 0; j < count; ++j) {
                    bool same = true;
                    for (unsigned i = 0; i < n; ++i) same &= std::abs(answers[j].joints[i] - q[i]) < 1e-7;
                    contains_original |= same;
                    mk_analytic_pose back = {};
                    assert(mk_analytic_cartesian_forward(&m, answers[j].joints, n, &back) == MK_OK);
                    for (unsigned i = 0; i < 3; ++i) assert(std::abs(back.position[i] - target.position[i]) < 1e-8);
                }
                assert(contains_original);
            }
        }
    }
    // Axial tool direction reports a singular C, and honors its seed for free spin.
    double q[] = {0.1,0.2,0.3,0.7,-0.4}; mk_analytic_pose target = {};
    assert(mk_analytic_cartesian_forward(&m, q, 5, &target) == MK_OK);
    mk_analytic_solution answers[2] = {}; uint32_t count = 0;
    assert(mk_analytic_cartesian_inverse(&m, &target, 1, 1.23, answers, 2, &count) == MK_OK);
    assert(count == 1 && answers[0].singular == 1 && std::abs(answers[0].joints[3] - 1.23) < 1e-8);
    assert(mk_analytic_cartesian_inverse(&m, &target, 0, 1.23, answers, 2, &count) == MK_OK);
    assert(count == 1 && answers[0].singular == 1 && std::abs(answers[0].joints[3] - q[3]) < 1e-8);
    m.translation_axes[0] = 0;
    assert(mk_analytic_cartesian_forward(&m, q, 5, &target) == MK_ERROR_INVALID_ARGUMENT);
    std::puts("Cartesian analytic XYZ/XYZ+C/XYZ+C+A: 600 round trips and singular diagnostics passed");
}

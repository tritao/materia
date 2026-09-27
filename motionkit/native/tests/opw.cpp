#include "motionkit.h"

#include <cassert>
#include <cmath>

int main() {
    mk_opw_parameters parameters{};
    parameters.struct_size = sizeof(parameters);
    parameters.a1 = 0.100;
    parameters.a2 = -0.135;
    parameters.b = 0.0;
    parameters.c1 = 0.615;
    parameters.c2 = 0.705;
    parameters.c3 = 0.755;
    parameters.c4 = 0.085;
    parameters.offsets[2] = -std::acos(-1.0) / 2.0;
    for (auto &sign : parameters.sign_corrections) sign = 1;
    for (int sample = 0; sample < 32; ++sample) {
        double joints[6];
        for (int joint = 0; joint < 6; ++joint)
            joints[joint] = 0.65 * std::sin(0.37 * (sample + 1) * (joint + 2));
        mk_opw_pose pose{};
        pose.struct_size = sizeof(pose);
        assert(mk_opw_forward(&parameters, joints, 6, &pose) == MK_OK);
        mk_opw_solution solutions[8]{};
        for (auto &solution : solutions) solution.struct_size = sizeof(solution);
        assert(mk_opw_inverse(&parameters, &pose, solutions, 8) == MK_OK);
        bool found = false;
        for (const auto &solution : solutions) {
            if (!solution.valid) continue;
            double error = 0.0;
            for (int joint = 0; joint < 6; ++joint)
                error += std::abs(std::remainder(solution.joints[joint] - joints[joint],
                    2.0 * std::acos(-1.0)));
            if (error < 1e-9) found = true;
        }
        assert(found);
    }
    const double wrist_singular[6] = {0.2, -0.3, 0.4, 0.5, 1e-7, 0.7};
    mk_opw_pose singular_pose{};
    singular_pose.struct_size = sizeof(singular_pose);
    assert(mk_opw_forward(&parameters, wrist_singular, 6, &singular_pose) == MK_OK);
    mk_opw_solution singular_solutions[8]{};
    assert(mk_opw_inverse(&parameters, &singular_pose, singular_solutions, 8) == MK_OK);
    bool flagged = false;
    for (const auto &solution : singular_solutions)
        flagged = flagged || (solution.valid && solution.singular);
    assert(flagged);
}

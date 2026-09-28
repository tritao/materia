// Runs an MJCF model in plain MuJoCo under the conformance torque sequence and
// writes the trajectory, the reference side of the sim-to-sim conformance
// check (robotkit/plans/HUMANOID.md, HU-D5). humanoid.Conformance runs the
// imported model through RobotKit and compares against this file.
//
// Torque sequence: during control tick k (time t = k * period), actuator i
// applies ctrl = amplitude * sin(2 pi (0.7 + 0.3 i) t + 0.5 i), held for the
// whole tick. The Haxe side computes the same values.
//
// With a keyframe name, the run starts from that keyframe and each actuator's
// control is the keyframe's control plus the sequence above, so position
// servos swing around a pose such as a humanoid's stance.
//
// Solver iteration limits, when given, override the file's: a truncated solve
// depends on constraint order, which differs between compilers of one model.
//
// Usage: robotkit_mjcf_reference <model.xml> <control-period-s> <ticks> <amplitude> <out.csv>
//          [keyframe [solver-iterations line-search-iterations]]
// The first row is the initial state; each further row follows one tick: time,
// root position x y z, root quaternion x y z w, then each actuated joint's
// position, in actuator order, headed by name.

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

int main(int argc, char **argv) {
    if (argc != 6 && argc != 7 && argc != 9) {
        std::fprintf(stderr,
            "usage: %s <model.xml> <control-period-s> <ticks> <amplitude> <out.csv> "
            "[keyframe [solver-iterations line-search-iterations]]\n", argv[0]);
        return 2;
    }
    const double period = std::atof(argv[2]);
    const int ticks = std::atoi(argv[3]);
    const double amplitude = std::atof(argv[4]);
    char error[1024] = "";
    mjModel *m = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!m) {
        std::fprintf(stderr, "mjcf_reference: %s\n", error);
        return 1;
    }
    const int substeps = static_cast<int>(std::lround(period / m->opt.timestep));
    if (substeps < 1 || std::abs(substeps * m->opt.timestep - period) > 1e-12) {
        std::fprintf(stderr, "mjcf_reference: the control period must be a whole number of steps\n");
        mj_deleteModel(m);
        return 1;
    }
    if (m->njnt == 0 || m->jnt_type[0] != mjJNT_FREE) {
        std::fprintf(stderr, "mjcf_reference: the first joint must be the root free joint\n");
        mj_deleteModel(m);
        return 1;
    }
    if (argc == 9) {
        m->opt.iterations = std::atoi(argv[7]);
        m->opt.ls_iterations = std::atoi(argv[8]);
    }
    mjData *d = mj_makeData(m);
    std::vector<double> base_ctrl(m->nu, 0.0);
    if (argc >= 7) {
        const int key = mj_name2id(m, mjOBJ_KEY, argv[6]);
        if (key < 0) {
            std::fprintf(stderr, "mjcf_reference: no keyframe %s\n", argv[6]);
            mj_deleteData(d);
            mj_deleteModel(m);
            return 1;
        }
        mj_resetDataKeyframe(m, d, key);
        mj_forward(m, d);
        for (int i = 0; i < m->nu; ++i) base_ctrl[i] = m->key_ctrl[key * m->nu + i];
    }
    std::ofstream out(argv[5]);
    out << "time,x,y,z,qx,qy,qz,qw";
    for (int i = 0; i < m->nu; ++i)
        out << "," << mj_id2name(m, mjOBJ_JOINT, m->actuator_trnid[2 * i]);
    out << "\n";
    out.precision(17);
    const auto write_row = [&](double time) {
        // MuJoCo stores the free joint's quaternion as w, x, y, z.
        out << time << "," << d->qpos[0] << "," << d->qpos[1] << "," << d->qpos[2] << ","
            << d->qpos[4] << "," << d->qpos[5] << "," << d->qpos[6] << "," << d->qpos[3];
        for (int i = 0; i < m->nu; ++i)
            out << "," << d->qpos[m->jnt_qposadr[m->actuator_trnid[2 * i]]];
        out << "\n";
    };
    write_row(0.0); // The initial state, which the compared simulation starts from.
    for (int tick = 0; tick < ticks; ++tick) {
        const double t = tick * period;
        for (int i = 0; i < m->nu; ++i)
            d->ctrl[i] = base_ctrl[i] + amplitude * std::sin(2.0 * M_PI * (0.7 + 0.3 * i) * t + 0.5 * i);
        for (int step = 0; step < substeps; ++step) mj_step(m, d);
        write_row((tick + 1) * period);
    }
    mj_deleteData(d);
    mj_deleteModel(m);
    return out ? 0 : 1;
}

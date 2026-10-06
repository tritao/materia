#include "EAIK.h"
#include <opw_kinematics/opw_parameters_examples.h>
#include <opw_kinematics/opw_utilities.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool samePose(const Eigen::Matrix4d& actual, const Eigen::Matrix4d& expected) {
    const Eigen::Matrix3d relative = actual.block<3, 3>(0, 0).transpose() * expected.block<3, 3>(0, 0);
    return (actual.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm() < 1e-9
        && Eigen::AngleAxisd(relative).angle() < 1e-9;
}

// Canonical OPW fixture vectors. Authored RobotModel vectors are a separate
// acceptance check; these fixtures include signed joints and zero offsets.
void check(const char* name, const opw_kinematics::Parameters<double>& p) {
    Eigen::Matrix<double, 3, 6> h;
    h << 0, 0, 0, 0, 0, 0,
         0, 1, 1, 0, 1, 0,
         1, 0, 0, 1, 0, 1;
    Eigen::Matrix<double, 3, 7> positions = Eigen::Matrix<double, 3, 7>::Zero();
    positions.col(0) = Eigen::Vector3d(0, 0, p.c1);
    positions.col(1) = Eigen::Vector3d(p.a1, p.b, 0);
    positions.col(2) = Eigen::Vector3d(0, 0, p.c2);
    positions.col(3) = Eigen::Vector3d(p.a2, 0, p.c3);
    positions.col(6) = Eigen::Vector3d(0, 0, p.c4);
    EAIK::Robot robot(h, positions);
    require(robot.has_known_decomposition(), "EAIK lacks a fixture decomposition");
    std::mt19937_64 random(0x50503161);
    std::uniform_real_distribution<double> angle(-2.8, 2.8);
    std::size_t matched = 0;
    for (int sample = 0; sample < 1000; ++sample) {
        std::array<double, 6> physical;
        std::vector<double> canonical(6);
        for (int j = 0; j < 6; ++j) {
            physical[j] = angle(random);
            canonical[j] = p.sign_corrections[j] * physical[j] - p.offsets[j];
        }
        auto target = opw_kinematics::forward(p, physical);
        require(samePose(robot.fwdkin(canonical), target.matrix()),
            "Canonical fixture FK differs from OPW");
        const auto solutions = robot.calculate_IK(target.matrix());
        const auto repeated = robot.calculate_IK(target.matrix());
        require(solutions.Q == repeated.Q && solutions.is_LS_vec == repeated.is_LS_vec,
            "EAIK all-solutions calls are not deterministic");
        for (std::size_t i = 0; i < solutions.Q.size(); ++i) {
            if (solutions.is_LS_vec[i]) continue;
            require(samePose(robot.fwdkin(solutions.Q[i]), target.matrix()),
                "EAIK fixture FK round trip failed");
        }
        for (const auto& expected : opw_kinematics::inverse(p, target)) {
            if (!opw_kinematics::isValid(expected)) continue;
            bool found = false;
            for (std::size_t i = 0; i < solutions.Q.size(); ++i) {
                if (solutions.is_LS_vec[i]) continue;
                const auto& solution = solutions.Q[i];
                double error = 0;
                for (int j = 0; j < 6; ++j)
                    error = std::max(error, std::abs(std::remainder(
                        solution[j] - (p.sign_corrections[j] * expected[j] - p.offsets[j]),
                        2 * std::acos(-1.0))));
                found = found || error < 1e-9;
            }
            require(found, "EAIK does not contain every OPW solution within 1e-9 rad");
            ++matched;
        }
    }
    std::cout << name << ": 1000 deterministic targets, " << matched
              << " OPW solutions contained; " << robot.get_kinematic_family() << '\n';
}
}

int main() {
    try {
        check("IRB2400", opw_kinematics::makeIrb2400_10<double>());
        check("KR6", opw_kinematics::makeKukaKR6_R700_sixx<double>());
        check("R2000", opw_kinematics::makeFanucR2000iB_200R<double>());
        check("TX40", opw_kinematics::makeStaubliTX40<double>());
    } catch (const std::exception& error) {
        std::cerr << "EAIK spike failed: " << error.what() << '\n';
        return 1;
    }
}

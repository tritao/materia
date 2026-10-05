#ifndef NKSIM_SERVO_FEEDFORWARD_HPP
#define NKSIM_SERVO_FEEDFORWARD_HPP

#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <utility>

namespace nksim_mujoco {

using DenseRows = std::vector<std::vector<double>>;

/** Nominal independent coordinates of one connected transmission component.
    Rows name native DOFs; columns span the coupling equations' nullspace. */
struct DriveCoordinateSpace {
    std::vector<int> dofs;
    DenseRows basis;
};

inline DenseRows coupling_nullspace(DenseRows equations, std::size_t columns) {
    std::vector<std::size_t> pivots;
    std::size_t row = 0;
    for (std::size_t column = 0; column < columns && row < equations.size(); ++column) {
        std::size_t best = row;
        for (std::size_t candidate = row + 1; candidate < equations.size(); ++candidate)
            if (std::abs(equations[candidate][column]) > std::abs(equations[best][column])) best = candidate;
        if (std::abs(equations[best][column]) < 1e-12) continue;
        std::swap(equations[best], equations[row]);
        const double divisor = equations[row][column];
        for (double &value : equations[row]) value /= divisor;
        for (std::size_t other = 0; other < equations.size(); ++other) if (other != row) {
            const double multiple = equations[other][column];
            for (std::size_t slot = 0; slot < columns; ++slot)
                equations[other][slot] -= multiple * equations[row][slot];
        }
        pivots.push_back(column);
        ++row;
    }
    DenseRows result(columns);
    for (std::size_t free = 0; free < columns; ++free) {
        if (std::find(pivots.begin(), pivots.end(), free) != pivots.end()) continue;
        std::vector<double> vector(columns, 0.0);
        vector[free] = 1.0;
        for (std::size_t index = 0; index < pivots.size(); ++index)
            vector[pivots[index]] = -equations[index][free];
        // Orthonormal coordinates keep mill leads and arm radians well conditioned.
        for (std::size_t previous = 0; !result.empty() && previous < result[0].size(); ++previous) {
            double projection = 0.0;
            for (std::size_t slot = 0; slot < columns; ++slot) projection += vector[slot] * result[slot][previous];
            for (std::size_t slot = 0; slot < columns; ++slot) vector[slot] -= projection * result[slot][previous];
        }
        double norm = 0.0;
        for (double value : vector) norm += value * value;
        norm = std::sqrt(norm);
        for (std::size_t slot = 0; slot < columns; ++slot) result[slot].push_back(vector[slot] / norm);
    }
    return result;
}

/** The active motor coordinates of a space, cached until drive ownership changes. */
class ServoCoordinateMap {
public:
    std::vector<int> motors;
    std::vector<int> dofs;
    DenseRows body_map;
    DenseRows motor_map;
    std::vector<mjtNum> acceleration_factor;
    std::vector<mjtNum> acceleration_rhs, coordinates;
    std::vector<mjtNum> effort_factor, effort_rhs, effort_coordinates;
    std::vector<double> weights;
    std::size_t dimensions = 0;

    ServoCoordinateMap(const DriveCoordinateSpace &space, const std::vector<int> &active) {
        dofs = space.dofs;
        const std::size_t count = space.basis.empty() ? 0 : space.basis[0].size();
        DenseRows controlled;
        for (std::size_t row = 0; row < dofs.size(); ++row) {
            if (std::find(active.begin(), active.end(), dofs[row]) == active.end()) continue;
            auto direction = space.basis[row];
            double leverage = 0.0;
            for (double value : direction) leverage += value * value;
            if (leverage <= 1e-24) continue;
            motors.push_back(dofs[row]);
            for (const auto &previous : controlled) {
                double projection = 0.0;
                for (std::size_t slot = 0; slot < count; ++slot) projection += direction[slot] * previous[slot];
                for (std::size_t slot = 0; slot < count; ++slot) direction[slot] -= projection * previous[slot];
            }
            double norm = 0.0;
            for (double value : direction) norm += value * value;
            norm = std::sqrt(norm);
            if (norm <= 1e-12) continue;
            for (double &value : direction) value /= norm;
            controlled.push_back(std::move(direction));
        }
        dimensions = controlled.size();
        body_map.assign(dofs.size(), std::vector<double>(dimensions, 0.0));
        for (std::size_t row = 0; row < dofs.size(); ++row)
            for (std::size_t column = 0; column < dimensions; ++column)
                for (std::size_t slot = 0; slot < count; ++slot)
                    body_map[row][column] += space.basis[row][slot] * controlled[column][slot];
        for (int motor : motors) {
            const auto row = std::find(dofs.begin(), dofs.end(), motor) - dofs.begin();
            motor_map.push_back(body_map[row]);
        }
        acceleration_factor.assign(dimensions * dimensions, 0.0);
        for (const auto &motor : motor_map)
            for (std::size_t i = 0; i < dimensions; ++i)
                for (std::size_t j = 0; j < dimensions; ++j)
                    acceleration_factor[i * dimensions + j] += motor[i] * motor[j];
        if (dimensions > 0 && mju_cholFactor(acceleration_factor.data(), static_cast<int>(dimensions), 1e-14) != static_cast<int>(dimensions))
            throw std::runtime_error("Servo reference coordinates are singular");
        acceleration_rhs.resize(dimensions); coordinates.resize(dimensions);
        effort_factor.resize(dimensions * dimensions); effort_rhs.resize(dimensions); effort_coordinates.resize(dimensions);
        weights.resize(motors.size());
    }

    /** Lift exact motor accelerations into the whole nominal coupled motion. */
    void accelerations(const std::vector<mjtNum> &motor_accelerations, std::vector<mjtNum> &all) {
        if (dimensions == 0) return;
        std::fill(acceleration_rhs.begin(), acceleration_rhs.end(), 0.0);
        for (std::size_t motor = 0; motor < motors.size(); ++motor)
            for (std::size_t column = 0; column < dimensions; ++column)
                acceleration_rhs[column] += motor_map[motor][column] * motor_accelerations[motors[motor]];
        mju_cholSolve(coordinates.data(), acceleration_factor.data(), acceleration_rhs.data(), static_cast<int>(dimensions));
        for (std::size_t row = 0; row < dofs.size(); ++row)
            for (std::size_t column = 0; column < dimensions; ++column)
                all[dofs[row]] += body_map[row][column] * coordinates[column];
    }

    /** Project full articulated inertia effort back to the drives by virtual work.
        Parallel motors share by available effort and their mechanical leverage. */
    void efforts(const std::vector<mjtNum> &mass_effort, const std::vector<mjtNum> &limits,
                 std::vector<mjtNum> &result) {
        if (dimensions == 0) return;
        std::fill(effort_rhs.begin(), effort_rhs.end(), 0.0);
        for (std::size_t row = 0; row < dofs.size(); ++row)
            for (std::size_t column = 0; column < dimensions; ++column)
                effort_rhs[column] += body_map[row][column] * mass_effort[dofs[row]];
        double maximum = 0.0;
        for (std::size_t motor = 0; motor < motors.size(); ++motor) {
            double leverage = 0.0;
            for (double value : motor_map[motor]) leverage += value * value;
            weights[motor] = (limits[motors[motor]] > 0.0 ? limits[motors[motor]] : 1.0) / std::sqrt(leverage);
            maximum = std::max(maximum, weights[motor]);
        }
        std::fill(effort_factor.begin(), effort_factor.end(), 0.0);
        for (std::size_t motor = 0; motor < motors.size(); ++motor) {
            weights[motor] /= maximum;
            for (std::size_t i = 0; i < dimensions; ++i)
                for (std::size_t j = 0; j < dimensions; ++j)
                    effort_factor[i * dimensions + j] += weights[motor] * motor_map[motor][i] * motor_map[motor][j];
        }
        if (mju_cholFactor(effort_factor.data(), static_cast<int>(dimensions), 1e-14) != static_cast<int>(dimensions))
            throw std::runtime_error("Servo effort coordinates are singular");
        mju_cholSolve(effort_coordinates.data(), effort_factor.data(), effort_rhs.data(), static_cast<int>(dimensions));
        for (std::size_t motor = 0; motor < motors.size(); ++motor)
            for (std::size_t column = 0; column < dimensions; ++column)
                result[motors[motor]] += weights[motor] * motor_map[motor][column] * effort_coordinates[column];
    }
};

} // namespace nksim_mujoco
#endif

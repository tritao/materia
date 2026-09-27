#include "motionkit.h"

#include <descartes_light/solvers/ladder_graph/ladder_graph_solver.h>
#include <console_bridge/console.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using State = descartes_light::State<double>;
using Sample = descartes_light::StateSample<double>;

class CandidateSampler final : public descartes_light::WaypointSamplerD {
 public:
  CandidateSampler(const mk_configuration_request &request,
      const mk_configuration_candidate *candidates, uint32_t count)
      : request_(request), candidates_(candidates), count_(count) {}

  std::vector<Sample> sample() const override {
    std::vector<Sample> result;
    result.reserve(count_);
    for (uint32_t index = 0; index < count_; ++index) {
      const auto &candidate = candidates_[index];
      bool valid = candidate.struct_size == sizeof(candidate);
      for (uint32_t joint = 0; valid && joint < request_.joint_count; ++joint)
        valid = std::isfinite(candidate.joints[joint]) &&
            candidate.joints[joint] >= request_.lower[joint] &&
            candidate.joints[joint] <= request_.upper[joint];
      if (!valid) continue;
      Eigen::VectorXd values(request_.joint_count);
      for (uint32_t joint = 0; joint < request_.joint_count; ++joint)
        values[joint] = candidate.joints[joint];
      result.emplace_back(std::make_shared<State>(values));
    }
    return result;
  }

 private:
  const mk_configuration_request &request_;
  const mk_configuration_candidate *candidates_;
  uint32_t count_;
};

class WeightedEdge final : public descartes_light::EdgeEvaluatorD {
 public:
  explicit WeightedEdge(const mk_configuration_request &request) : request_(request) {}

  std::pair<bool, double> evaluate(const State &start, const State &end) const override {
    double cost = 0.0;
    for (uint32_t joint = 0; joint < request_.joint_count; ++joint) {
      const double distance = std::abs(end.values[joint] - start.values[joint]);
      if (distance > request_.max_jump[joint] + 1e-12) return {false, 0.0};
      cost += request_.weights[joint] * distance;
    }
    return {true, cost};
  }

 private:
  const mk_configuration_request &request_;
};

class PreferredPosture final : public descartes_light::StateEvaluatorD {
 public:
  explicit PreferredPosture(const mk_configuration_request &request) : request_(request) {}

  std::pair<bool, double> evaluate(const State &state) const override {
    double cost = 0.0;
    for (uint32_t joint = 0; joint < request_.joint_count; ++joint)
      cost += request_.weights[joint] *
          std::abs(state.values[joint] - request_.preferred[joint]);
    return {true, cost};
  }

 private:
  const mk_configuration_request &request_;
};

void diagnostic(mk_configuration_solution &result, uint32_t failed,
    double distance, const std::string &message) {
  result.failed_sample = failed;
  result.failed_distance = distance;
  std::memset(result.diagnostic, 0, sizeof(result.diagnostic));
  const auto length = std::min(message.size(), sizeof(result.diagnostic) - 1);
  for (size_t index = 0; index < length; ++index)
    result.diagnostic[index] = static_cast<int8_t>(message[index]);
}


uint32_t first_disconnected(const std::vector<std::vector<Sample>> &rungs,
    const WeightedEdge &edge) {
  std::vector<bool> reachable(rungs.front().size(), true);
  for (uint32_t rung = 1; rung < rungs.size(); ++rung) {
    std::vector<bool> next(rungs[rung].size(), false);
    for (size_t end = 0; end < next.size(); ++end)
      for (size_t start = 0; start < reachable.size(); ++start)
        if (reachable[start] && edge.evaluate(*rungs[rung - 1][start].state,
                *rungs[rung][end].state).first) {
          next[end] = true;
          break;
        }
    if (std::none_of(next.begin(), next.end(), [](bool value) { return value; }))
      return rung;
    reachable = std::move(next);
  }
  return static_cast<uint32_t>(rungs.size() - 1);
}
}  // namespace

extern "C" mk_result MK_CALL mk_select_configurations(
    const mk_configuration_request *request,
    const mk_configuration_sample *samples, uint32_t sample_count,
    const mk_configuration_candidate *candidates, uint32_t candidate_count,
    mk_configuration_solution *out_sequence) {
  if (!request || request->struct_size != sizeof(*request) ||
      request->joint_count == 0 || request->joint_count > MK_MAX_JOINTS ||
      request->threads == 0 || sample_count == 0 || !samples ||
      (candidate_count > 0 && !candidates) ||
      !out_sequence) return MK_ERROR_INVALID_ARGUMENT;
  if (request->threads > 1) {
#ifndef _OPENMP
    return MK_ERROR_UNSUPPORTED;
#endif
  }
  for (uint32_t joint = 0; joint < request->joint_count; ++joint) {
    if (!std::isfinite(request->lower[joint]) || !std::isfinite(request->upper[joint]) ||
        request->lower[joint] > request->upper[joint] ||
        !std::isfinite(request->max_jump[joint]) || request->max_jump[joint] <= 0.0 ||
        !std::isfinite(request->weights[joint]) || request->weights[joint] <= 0.0 ||
        (request->has_preferred && !std::isfinite(request->preferred[joint])))
      return MK_ERROR_INVALID_ARGUMENT;
  }
  uint32_t expected_first = 0;
  double previous_distance = -std::numeric_limits<double>::infinity();
  for (uint32_t index = 0; index < sample_count; ++index) {
    const auto &sample = samples[index];
    if (sample.struct_size != sizeof(sample) || !std::isfinite(sample.distance) ||
        sample.distance <= previous_distance || sample.first_candidate != expected_first ||
        sample.candidate_count > candidate_count - expected_first)
      return MK_ERROR_INVALID_ARGUMENT;
    previous_distance = sample.distance;
    expected_first += sample.candidate_count;
  }
  if (expected_first != candidate_count) return MK_ERROR_INVALID_ARGUMENT;

  out_sequence[0].struct_size = sizeof(out_sequence[0]);
  out_sequence[0].cost = 0.0;
  diagnostic(out_sequence[0], UINT32_MAX, 0.0, "");
  std::string log;
  std::vector<std::vector<Sample>> rungs;
  try {
    console_bridge::ScopedDiagnosticSink sink(log);
    std::vector<descartes_light::WaypointSamplerD::ConstPtr> samplers;
    samplers.reserve(sample_count);
    rungs.reserve(sample_count);
    for (uint32_t index = 0; index < sample_count; ++index) {
      const auto &sample = samples[index];
      auto sampler = std::make_shared<CandidateSampler>(*request,
          candidates ? candidates + sample.first_candidate : nullptr,
          sample.candidate_count);
      auto valid = sampler->sample();
      if (valid.empty()) {
        diagnostic(out_sequence[0], index, sample.distance,
            "No valid configurations at sample distance " +
            std::to_string(sample.distance));
        return MK_ERROR_GENERATION;
      }
      rungs.push_back(std::move(valid));
      samplers.push_back(sampler);
    }
    if (sample_count == 1) {
      const auto &chosen = rungs.front().front().state->values;
      out_sequence[0].struct_size = sizeof(out_sequence[0]);
      for (uint32_t joint = 0; joint < request->joint_count; ++joint)
        out_sequence[0].joints[joint] = chosen[joint];
      return MK_OK;
    }
    auto edge = std::make_shared<WeightedEdge>(*request);
    std::vector<descartes_light::EdgeEvaluatorD::ConstPtr> edges{edge};
    std::vector<descartes_light::StateEvaluatorD::ConstPtr> states;
    if (request->has_preferred)
      states.push_back(std::make_shared<PreferredPosture>(*request));
    descartes_light::LadderGraphSolverD solver(static_cast<int>(request->threads));
    const auto status = solver.build(samplers, edges, states);
    if (!status) {
      uint32_t failed = first_disconnected(rungs, *edge);
      if (!status.failed_vertices.empty())
        failed = std::min(failed, static_cast<uint32_t>(status.failed_vertices.front()));
      const auto distance = samples[failed].distance;
      diagnostic(out_sequence[0], failed, distance,
          "Disconnected configurations at sample distance " +
          std::to_string(distance) + ": " + log);
      return MK_ERROR_GENERATION;
    }
    const auto result = solver.search();
    if (result.trajectory.size() != sample_count)
      throw std::runtime_error("Descartes returned an incomplete path");
    out_sequence[0].cost = result.cost;
    for (uint32_t index = 0; index < sample_count; ++index) {
      auto &output = out_sequence[index];
      output.struct_size = sizeof(output);
      for (uint32_t joint = 0; joint < request->joint_count; ++joint)
        output.joints[joint] = result.trajectory[index]->values[joint];
    }
    return MK_OK;
  } catch (const std::bad_alloc &) {
    return MK_ERROR_OUT_OF_MEMORY;
  } catch (const std::exception &error) {
    uint32_t failed = sample_count - 1;
    if (rungs.size() == sample_count) {
      const WeightedEdge edge(*request);
      failed = first_disconnected(rungs, edge);
    }
    diagnostic(out_sequence[0], failed, samples[failed].distance,
        std::string("Descartes configuration search failed at sample distance ") +
        std::to_string(samples[failed].distance) + ": " + error.what() +
        (log.empty() ? "" : ": " + log));
    return MK_ERROR_GENERATION;
  }
}

extern "C" mk_result MK_CALL mk_select_opw_configurations(
    const mk_configuration_request *request, const mk_opw_parameters *parameters,
    const mk_opw_path_sample *samples, uint32_t sample_count,
    const double *start_joints, uint32_t joint_count,
    mk_configuration_solution *out_sequence) {
  if (!request || !parameters || parameters->struct_size != sizeof(*parameters) ||
      request->joint_count != 6 || joint_count != 6 || !samples ||
      sample_count == 0 || !start_joints || !out_sequence)
    return MK_ERROR_INVALID_ARGUMENT;
  const double geometry[] = {parameters->a1, parameters->a2, parameters->b,
      parameters->c1, parameters->c2, parameters->c3, parameters->c4};
  for (double value : geometry) if (!std::isfinite(value)) return MK_ERROR_INVALID_ARGUMENT;
  for (uint32_t joint = 0; joint < 6; ++joint) {
    if (!std::isfinite(parameters->offsets[joint]) ||
        (parameters->sign_corrections[joint] != -1 &&
         parameters->sign_corrections[joint] != 1) ||
        !std::isfinite(start_joints[joint])) return MK_ERROR_INVALID_ARGUMENT;
  }
  std::vector<mk_configuration_sample> path;
  std::vector<mk_configuration_candidate> candidates;
  path.reserve(sample_count);
  candidates.reserve(static_cast<size_t>(sample_count) * 16);
  for (uint32_t index = 0; index < sample_count; ++index) {
    const auto &source = samples[index];
    if (source.struct_size != sizeof(source)) return MK_ERROR_INVALID_ARGUMENT;
    mk_configuration_sample sample{};
    sample.struct_size = sizeof(sample);
    sample.distance = source.distance;
    sample.first_candidate = static_cast<uint32_t>(candidates.size());
    if (index == 0) {
      mk_configuration_candidate candidate{};
      candidate.struct_size = sizeof(candidate);
      for (uint32_t joint = 0; joint < 6; ++joint)
        candidate.joints[joint] = start_joints[joint];
      candidates.push_back(candidate);
    } else {
      mk_opw_pose pose{};
      pose.struct_size = sizeof(pose);
      for (uint32_t coordinate = 0; coordinate < 3; ++coordinate)
        pose.position[coordinate] = source.position[coordinate];
      for (uint32_t coordinate = 0; coordinate < 4; ++coordinate)
        pose.quaternion[coordinate] = source.quaternion[coordinate];
      mk_opw_solution solutions[8]{};
      const auto status = mk_opw_inverse(parameters, &pose, solutions, 8);
      if (status != MK_OK) return status;
      std::vector<std::array<double, 6>> all_variants;
      for (const auto &solution : solutions) {
        if (!solution.valid) continue;
        std::vector<std::array<double, 6>> variants(1);
        for (uint32_t joint = 0; joint < 6; ++joint) {
          std::vector<std::array<double, 6>> next;
          for (const auto &variant : variants)
            for (int shift = -1; shift <= 1; ++shift) {
              const double value = solution.joints[joint] +
                  shift * 2.0 * std::acos(-1.0);
              if (value < request->lower[joint] || value > request->upper[joint]) continue;
              auto chosen = variant;
              chosen[joint] = value;
              next.push_back(chosen);
            }
          variants = std::move(next);
          if (variants.empty()) break;
        }
        all_variants.insert(all_variants.end(), variants.begin(), variants.end());
      }
      auto distance_from_start = [&](const std::array<double, 6> &joints) {
        double distance = 0.0;
        for (uint32_t joint = 0; joint < 6; ++joint)
          distance += std::abs(joints[joint] - start_joints[joint]) /
              request->max_jump[joint];
        return distance;
      };
      std::stable_sort(all_variants.begin(), all_variants.end(),
          [&](const auto &left, const auto &right) {
            return distance_from_start(left) < distance_from_start(right);
          });
      for (const auto &variant : all_variants) {
        mk_configuration_candidate candidate{};
        candidate.struct_size = sizeof(candidate);
        for (uint32_t joint = 0; joint < 6; ++joint)
          candidate.joints[joint] = variant[joint];
        candidates.push_back(candidate);
        if (candidates.size() - sample.first_candidate >= 64) break;
      }
    }
    sample.candidate_count = static_cast<uint32_t>(candidates.size()) -
        sample.first_candidate;
    path.push_back(sample);
  }
  return mk_select_configurations(request, path.data(), sample_count,
      candidates.data(), static_cast<uint32_t>(candidates.size()), out_sequence);
}

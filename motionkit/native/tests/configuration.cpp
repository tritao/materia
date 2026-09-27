#include "motionkit.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static mk_configuration_request request(uint32_t threads = 1) {
    mk_configuration_request value{};
    value.struct_size = sizeof(value);
    value.joint_count = 1;
    value.threads = threads;
    value.lower[0] = -100.0;
    value.upper[0] = 100.0;
    value.max_jump[0] = 2.0;
    value.weights[0] = 1.0;
    return value;
}

static mk_configuration_candidate candidate(double joint) {
    mk_configuration_candidate value{};
    value.struct_size = sizeof(value);
    value.joints[0] = joint;
    return value;
}

static mk_configuration_sample sample(double distance, uint32_t first, uint32_t count) {
    mk_configuration_sample value{};
    value.struct_size = sizeof(value);
    value.distance = distance;
    value.first_candidate = first;
    value.candidate_count = count;
    return value;
}

int main() {
    const mk_configuration_sample samples[] = {
        sample(0.0, 0, 2), sample(0.5, 2, 2), sample(1.0, 4, 1)};
    const mk_configuration_candidate candidates[] = {
        candidate(0.0), candidate(10.0), candidate(1.0),
        candidate(9.0), candidate(10.0)};
    mk_configuration_solution output[3]{};
    auto options = request();
    assert(mk_select_configurations(&options, samples, 3, candidates, 5,
        output) == MK_OK);
    assert(output[0].joints[0] == 10.0);
    assert(output[1].joints[0] == 9.0);
    assert(output[2].joints[0] == 10.0);
    assert(output[0].failed_sample == UINT32_MAX);

    options = request(2);
    mk_configuration_solution parallel[3]{};
    const auto threaded = mk_select_configurations(&options, samples, 3,
        candidates, 5, parallel);
    assert(threaded == MK_OK || threaded == MK_ERROR_UNSUPPORTED);
    if (threaded == MK_OK)
        for (int index = 0; index < 3; ++index)
            assert(parallel[index].joints[0] == output[index].joints[0]);

    options = request();
    options.max_jump[0] = 0.5;
    assert(mk_select_configurations(&options, samples, 3, candidates, 5,
        output) == MK_ERROR_GENERATION);
    assert(output[0].failed_sample == 1);
    assert(output[0].failed_distance == 0.5);
    assert(std::string(reinterpret_cast<const char *>(output[0].diagnostic)).find(
        "0.500000") != std::string::npos);
    assert(std::string(reinterpret_cast<const char *>(output[0].diagnostic)).find(
        "Failed edges") != std::string::npos);

    const mk_configuration_sample split_samples[] = {
        sample(0.0, 0, 1), sample(0.5, 1, 2), sample(1.0, 3, 1)};
    const mk_configuration_candidate split_candidates[] = {
        candidate(0.0), candidate(0.0), candidate(10.0), candidate(10.0)};
    options = request();
    assert(mk_select_configurations(&options, split_samples, 3,
        split_candidates, 4, output) == MK_ERROR_GENERATION);
    assert(output[0].failed_sample == 2 && output[0].failed_distance == 1.0);

    const mk_configuration_sample posture_samples[] = {
        sample(0.0, 0, 2), sample(1.0, 2, 2)};
    const mk_configuration_candidate posture_candidates[] = {
        candidate(0.0), candidate(10.0), candidate(0.0), candidate(10.0)};
    options = request();
    options.max_jump[0] = 20.0;
    options.has_preferred = 1;
    options.preferred[0] = 10.0;
    assert(mk_select_configurations(&options, posture_samples, 2,
        posture_candidates, 4, output) == MK_OK);
    assert(output[0].joints[0] == 10.0 && output[1].joints[0] == 10.0);

    options = request();
    std::vector<mk_configuration_sample> many_samples;
    std::vector<mk_configuration_candidate> many_candidates;
    std::vector<mk_configuration_solution> many_output(1000);
    for (uint32_t index = 0; index < 1000; ++index) {
        many_samples.push_back(sample(index * 0.001, index * 8, 8));
        for (int choice = 0; choice < 8; ++choice)
            many_candidates.push_back(candidate(index * 0.001 + choice * 0.01));
    }
    const auto start = std::chrono::steady_clock::now();
    assert(mk_select_configurations(&options, many_samples.data(),
        static_cast<uint32_t>(many_samples.size()), many_candidates.data(),
        static_cast<uint32_t>(many_candidates.size()), many_output.data()) == MK_OK);
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    std::fprintf(stderr, "Descartes 1000 x 8 Debug selection: %.2f ms\n", elapsed);
    assert(elapsed < 50.0);
}

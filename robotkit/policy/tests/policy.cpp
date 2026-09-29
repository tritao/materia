#include "robotkit_policy.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#define CHECK(condition)                                                                 \
    do {                                                                                 \
        if (!(condition)) {                                                              \
            std::fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #condition); \
            std::exit(1);                                                                \
        }                                                                                \
    } while (0)

static bool near(double a, double b, double tolerance) { return std::fabs(a - b) <= tolerance; }

static std::unique_ptr<rk_policy_io> make_io() {
    auto io = std::make_unique<rk_policy_io>();
    std::memset(io.get(), 0, sizeof(rk_policy_io));
    io->struct_size = sizeof(rk_policy_io);
    return io;
}

static rk_policy_tensor tensor(rk_policy policy, uint32_t direction, uint32_t index) {
    rk_policy_tensor value{};
    value.struct_size = sizeof(value);
    CHECK(rk_policy_get_tensor(policy, direction, index, &value) == RK_OK);
    return value;
}

// y = a @ W + b and s_out = s + y, with W = [[1,2],[.5,-1],[-2,.25]] and b = [.1,-.2].
static void affine_model_with_explicit_state() {
    rk_policy policy = RK_INVALID_POLICY;
    CHECK(rk_policy_create(RK_POLICY_FIXTURES "/affine_state.onnx", &policy) == RK_OK);
    rk_policy_info info{};
    info.struct_size = sizeof(info);
    CHECK(rk_policy_get_info(policy, &info) == RK_OK);
    CHECK(info.input_count == 2 && info.output_count == 2);
    CHECK(info.input_values == 5 && info.output_values == 4);
    CHECK(std::strcmp(tensor(policy, RK_POLICY_INPUT, 0).name, "a") == 0);
    CHECK(tensor(policy, RK_POLICY_INPUT, 1).offset == 3 && tensor(policy, RK_POLICY_INPUT, 1).elements == 2);
    CHECK(std::strcmp(tensor(policy, RK_POLICY_OUTPUT, 1).name, "s_out") == 0);
    CHECK(tensor(policy, RK_POLICY_OUTPUT, 1).offset == 2);

    auto io = make_io();
    double state[2] = {0.0, 0.0};
    for (int step = 1; step <= 3; ++step) {
        const double a[3] = {1.0, 2.0 * step, -1.0};
        io->inputs[0] = a[0]; io->inputs[1] = a[1]; io->inputs[2] = a[2];
        io->inputs[3] = state[0]; io->inputs[4] = state[1];
        CHECK(rk_policy_run(policy, io.get()) == RK_OK);
        const double y0 = a[0] * 1.0 + a[1] * 0.5 + a[2] * -2.0 + 0.1;
        const double y1 = a[0] * 2.0 + a[1] * -1.0 + a[2] * 0.25 - 0.2;
        CHECK(near(io->outputs[0], y0, 1e-6) && near(io->outputs[1], y1, 1e-6));
        state[0] += y0; state[1] += y1;
        CHECK(near(io->outputs[2], state[0], 1e-5) && near(io->outputs[3], state[1], 1e-5));
        state[0] = io->outputs[2]; state[1] = io->outputs[3];
    }
    rk_policy_destroy(policy);
    CHECK(rk_policy_run(policy, io.get()) == RK_ERROR_INVALID_HANDLE);
    rk_policy_destroy(policy); // harmless twice
}

static void bad_arguments_and_models() {
    rk_policy policy = 99;
    CHECK(rk_policy_create(nullptr, &policy) == RK_ERROR_INVALID_ARGUMENT);
    CHECK(rk_policy_create("x.onnx", nullptr) == RK_ERROR_INVALID_ARGUMENT);
    CHECK(rk_policy_create(RK_POLICY_FIXTURES "/missing.onnx", &policy) == RK_ERROR_BACKEND);
    CHECK(policy == RK_INVALID_POLICY);
    // Not a model at all.
    CHECK(rk_policy_create(RK_POLICY_FIXTURES "/../../CMakeLists.txt", &policy) == RK_ERROR_BACKEND);
    rk_policy_info info{};
    CHECK(rk_policy_get_info(1234, &info) == RK_ERROR_INVALID_ARGUMENT); // struct_size unset
    info.struct_size = sizeof(info);
    CHECK(rk_policy_get_info(1234, &info) == RK_ERROR_INVALID_HANDLE);
    rk_policy_io small{};
    CHECK(rk_policy_run(1234, &small) == RK_ERROR_INVALID_ARGUMENT);
}

// Golden values from onnxruntime 1.30.0 in Python (tools/humanoid/policies/make_test_fixtures.py) on
// obs_k[i] = 0.5 * sin(0.1 * (47 k + i)), with the LSTM state fed back each step.
static void unitree_g1_policy_matches_python() {
    rk_policy policy = RK_INVALID_POLICY;
    CHECK(rk_policy_create(RK_POLICY_G1_MODEL, &policy) == RK_OK);
    rk_policy_info info{};
    info.struct_size = sizeof(info);
    CHECK(rk_policy_get_info(policy, &info) == RK_OK);
    CHECK(info.input_count == 3 && info.output_count == 3);
    CHECK(info.input_values == 47 + 64 + 64 && info.output_values == 12 + 64 + 64);
    CHECK(std::strcmp(tensor(policy, RK_POLICY_INPUT, 0).name, "obs") == 0);
    CHECK(std::strcmp(tensor(policy, RK_POLICY_OUTPUT, 0).name, "action") == 0);

    static const double first[12] = {2.70954037, -0.696337819, 0.151529938, 1.7566756, 2.12212324,
        0.383644611, -2.40137696, 1.48327303, 1.93907154, -0.27001524, 0.95063132, 1.15230751};
    static const double last[12] = {3.22580767, 1.93022501, -0.342123568, 2.09378767, 3.17845392,
        0.889675498, -1.39973164, 1.8377111, 1.68238902, 1.5478754, 0.570321441, 1.00086355};
    auto io = make_io();
    for (int step = 0; step < 100; ++step) {
        for (int i = 0; i < 47; ++i) io->inputs[i] = 0.5 * std::sin(0.1 * (step * 47 + i));
        CHECK(rk_policy_run(policy, io.get()) == RK_OK);
        const double *expected = step == 0 ? first : step == 99 ? last : nullptr;
        if (expected)
            for (int i = 0; i < 12; ++i) CHECK(near(io->outputs[i], expected[i], 1e-3));
        // h_out and c_out become the next h_in and c_in.
        for (int i = 0; i < 128; ++i) io->inputs[47 + i] = io->outputs[12 + i];
    }
    CHECK(near(io->outputs[12], 0.0761331394, 1e-3) && near(io->outputs[13], 0.171382159, 1e-3));
    rk_policy_destroy(policy);
}

int main() {
    affine_model_with_explicit_state();
    bad_arguments_and_models();
    unitree_g1_policy_matches_python();
    std::puts("robotkit policy tests passed");
    return 0;
}

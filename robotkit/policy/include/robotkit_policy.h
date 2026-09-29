#ifndef ROBOTKIT_POLICY_H
#define ROBOTKIT_POLICY_H

/**
 * @file robotkit_policy.h
 * @brief C ABI for running a learned policy (an ONNX model) on the CPU.
 *
 * A policy is a pure function from tensors to tensors. Whatever a controller
 * needs to remember between calls, such as a recurrent network's state, is an
 * ordinary input the caller feeds back from an output; the model keeps nothing.
 * The library reads each tensor's name and element count from the model and
 * moves all inputs and outputs as flat arrays in the model's own tensor order,
 * so that the ABI does not depend on the model. Every tensor must have a static
 * shape; a dynamic leading (batch) dimension is taken as 1.
 *
 * ONNX Runtime (MIT) does the inference. It is loaded with the library and not
 * exposed here.
 */

#include "robotkit_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque handle for one loaded model and its inference session. */
typedef uint32_t rk_policy RK_HANDLE RK_HANDLE_DESTROY(rk_policy_destroy);
#define RK_INVALID_POLICY ((rk_policy)0)

/** Largest tensor count and largest value count per direction. */
#define RK_POLICY_MAX_TENSORS 16
#define RK_POLICY_MAX_VALUES 1024
#define RK_POLICY_NAME_BYTES 64

/** Which side of the model a tensor is on. */
enum { RK_POLICY_INPUT = 0, RK_POLICY_OUTPUT = 1 };

/** Sizes of a loaded model. */
typedef struct rk_policy_info {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t input_count;
    uint32_t output_count;
    uint32_t input_values;  /**< Elements over all inputs. */
    uint32_t output_values; /**< Elements over all outputs. */
} rk_policy_info;

/** One model tensor. */
typedef struct rk_policy_tensor {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t elements;
    uint32_t offset; /**< Position of the tensor's first element in rk_policy_io's array. */
    char name[RK_POLICY_NAME_BYTES]; /**< NUL-terminated. */
} rk_policy_tensor;

/**
 * Inputs and outputs of one inference. The caller fills `inputs`, the first
 * rk_policy_info.input_values entries, tensor after tensor in model order; the
 * library fills `outputs` the same way. Values are converted to and from the
 * model's 32-bit floats.
 */
typedef struct rk_policy_io {
    uint32_t struct_size RK_STRUCT_SIZE;
    double inputs[RK_POLICY_MAX_VALUES];
    double outputs[RK_POLICY_MAX_VALUES];
} rk_policy_io;

/**
 * Loads a model file and creates a single-threaded inference session.
 *
 * @return RK_OK; RK_ERROR_INVALID_ARGUMENT for a null path or handle;
 * RK_ERROR_BACKEND when the file cannot be read or is not a valid model;
 * RK_ERROR_UNSUPPORTED for a tensor that is not 32-bit float, has a dynamic
 * dimension other than the leading one, or exceeds the limits above.
 */
RK_API rk_result RK_CALL rk_policy_create(const char *path RK_UTF8,
                                          rk_policy *out_policy RK_OUT RK_OWNED);
/** Destroys a policy. Destroying an invalid or destroyed handle is harmless. */
RK_API void RK_CALL rk_policy_destroy(rk_policy policy);
/** Reads a loaded model's tensor counts and total sizes. */
RK_API rk_result RK_CALL rk_policy_get_info(rk_policy policy, rk_policy_info *out_info RK_INOUT);
/** Reads one tensor of a loaded model; `direction` is RK_POLICY_INPUT or RK_POLICY_OUTPUT. */
RK_API rk_result RK_CALL rk_policy_get_tensor(rk_policy policy, uint32_t direction, uint32_t index,
                                              rk_policy_tensor *out_tensor RK_INOUT);
/** Runs the model once. Returns RK_ERROR_BACKEND if inference fails. */
RK_API rk_result RK_CALL rk_policy_run(rk_policy policy, rk_policy_io *io RK_INOUT);

#ifdef __cplusplus
}
#endif

#endif

#ifndef ROBOTKIT_INFERENCE_H
#define ROBOTKIT_INFERENCE_H

#include "robotkit_runtime.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t rk_inference_session RK_HANDLE RK_HANDLE_DESTROY(rk_inference_destroy);
enum { RK_INFERENCE_INPUT = 0, RK_INFERENCE_OUTPUT = 1 };
enum { RK_INFERENCE_FLOAT32 = 1, RK_INFERENCE_UINT8 = 2 };
enum { RK_INFERENCE_NCHW = 1, RK_INFERENCE_NHWC = 2 };

typedef struct rk_inference_options {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t intra_op_threads;
    uint32_t dynamic_width;
    uint32_t dynamic_height;
} rk_inference_options;

typedef struct rk_inference_info {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t input_count;
    uint32_t output_count;
    uint64_t input_bytes;
    uint64_t output_bytes;
} rk_inference_info;

typedef struct rk_inference_tensor {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t element_type;
    uint32_t rank;
    uint64_t byte_offset;
    uint64_t byte_count;
    int64_t shape[8];
    char name[128];
} rk_inference_tensor;

typedef struct rk_inference_image_options {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint32_t target_width;
    uint32_t target_height;
    uint32_t layout;
    float padding_value;
    float scale;
    float mean[3];
    float std[3];
} rk_inference_image_options;

typedef struct rk_inference_result {
    uint32_t struct_size RK_STRUCT_SIZE;
    uint64_t sequence;
    uint64_t dropped;
    uint64_t completed_timestamp_ns;
    uint32_t output_bytes;
    uint32_t source_width;
    uint32_t source_height;
    float image_scale;
    float pad_x;
    float pad_y;
    rk_result status;
} rk_inference_result;

RK_API rk_result RK_CALL rk_inference_create(const char *path RK_UTF8,
    const rk_inference_options *options, rk_inference_session *out_session RK_OUT RK_OWNED);
RK_API void RK_CALL rk_inference_destroy(rk_inference_session session);
RK_API rk_result RK_CALL rk_inference_get_info(rk_inference_session session,
    rk_inference_info *out_info RK_INOUT);
RK_API rk_result RK_CALL rk_inference_get_tensor(rk_inference_session session,
    uint32_t direction, uint32_t index, rk_inference_tensor *out_tensor RK_INOUT);
RK_API rk_result RK_CALL rk_inference_model_sha256(const char *path RK_UTF8,
    char *digest RK_OUT_BUFFER(inout_size), uint32_t *inout_size RK_INOUT);
RK_API rk_result RK_CALL rk_inference_run(rk_inference_session session,
    const uint8_t *input RK_IN_ARRAY(input_size), uint32_t input_size,
    uint8_t *output RK_OUT_BUFFER(inout_output_size), uint32_t *inout_output_size RK_INOUT);
RK_API rk_result RK_CALL rk_inference_submit(rk_inference_session session,
    uint64_t sequence, const uint8_t *input RK_IN_ARRAY(input_size), uint32_t input_size);
RK_API rk_result RK_CALL rk_inference_submit_rgb8(rk_inference_session session,
    uint64_t sequence, uint32_t width, uint32_t height, uint32_t stride,
    const uint8_t *pixels RK_IN_ARRAY(pixel_bytes), uint32_t pixel_bytes,
    const rk_inference_image_options *options);
/** RK_ERROR_STALE_STATE means no result. RK_ERROR_LIMIT reports required bytes without consuming it. */
RK_API rk_result RK_CALL rk_inference_poll_result(rk_inference_session session,
    rk_inference_result *result, uint8_t *output RK_OUT_BUFFER(inout_output_size),
    uint32_t *inout_output_size RK_INOUT);

#ifdef __cplusplus
}
#endif
#endif

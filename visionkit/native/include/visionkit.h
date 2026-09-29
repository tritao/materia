#ifndef VISIONKIT_H
#define VISIONKIT_H

#include <stdint.h>

#if defined(__clang__)
#define VK_IN_ARRAY(count) __attribute__((annotate("hxi:in_array")))
#define VK_OUT_ARRAY(count) __attribute__((annotate("hxi:out_array")))
#define VK_OUT __attribute__((annotate("hxi:out")))
#define VK_OWNED __attribute__((annotate("hxi:owned")))
#define VK_HANDLE __attribute__((annotate("hxi:handle")))
#define VK_HANDLE_DESTROY(symbol) __attribute__((annotate("hxi:handle_destroy")))
#define VK_STRUCT_SIZE __attribute__((annotate("hxi:struct_size")))
#define VK_BORROWED __attribute__((annotate("hxi:borrowed")))
#define VK_LENGTH_FIELD(field) __attribute__((annotate("hxi:length_field")))
#else
#define VK_IN_ARRAY(count)
#define VK_OUT_ARRAY(count)
#define VK_OUT
#define VK_OWNED
#define VK_HANDLE
#define VK_HANDLE_DESTROY(symbol)
#define VK_STRUCT_SIZE
#define VK_BORROWED
#define VK_LENGTH_FIELD(field)
#endif

#if defined(_WIN32)
#  if defined(VK_BUILDING_LIBRARY)
#    define VK_API __declspec(dllexport)
#  else
#    define VK_API __declspec(dllimport)
#  endif
#else
#  define VK_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum vk_result {
    VK_OK = 0,
    VK_ERROR_INVALID_ARGUMENT = -1,
    VK_ERROR_INVALID_HANDLE = -2,
    VK_ERROR_OUT_OF_MEMORY = -3,
    VK_ERROR_BACKEND = -4,
    VK_ERROR_UNSUPPORTED = -5
} vk_result;

/** OpenCV plumb-bob uses k1,k2,p1,p2,k3. Fisheye is unsupported. */
typedef enum vk_distortion_model {
    VK_DISTORTION_NONE = 0,
    VK_DISTORTION_PLUMB_BOB = 1
} vk_distortion_model;

typedef struct vk_camera_model {
    uint32_t struct_size VK_STRUCT_SIZE;
    uint32_t width;
    uint32_t height;
    double fx, fy, cx, cy;
    uint32_t distortion_model;
    double k1, k2, p1, p2, k3;
} vk_camera_model;

/** Pixel centre coordinates, with origin at the top left. */
typedef struct vk_pixel { double x, y; } vk_pixel;
/** Camera-frame point in metres: +X forward, +Y left, +Z up. */
typedef struct vk_point3 { double x, y, z; } vk_point3;
/** Unit ray in the Materia camera frame. */
typedef vk_point3 vk_ray3;

typedef enum vk_pixel_format {
    VK_PIXEL_RGB8 = 1,
    VK_PIXEL_GRAY8 = 2,
    VK_PIXEL_DEPTH32F = 3
} vk_pixel_format;

/** Rows may have padding. stride_bytes must cover a complete pixel row. */
typedef struct vk_image_view {
    uint32_t struct_size VK_STRUCT_SIZE;
    uint32_t width;
    uint32_t height;
    uint32_t stride_bytes;
    uint32_t pixel_format;
    uint32_t buffer_bytes;
    uint8_t *data VK_BORROWED VK_LENGTH_FIELD(buffer_bytes);
} vk_image_view;

typedef enum vk_output_model_policy {
    VK_KEEP_ALL_PIXELS = 0,
    VK_CROP_VALID = 1
} vk_output_model_policy;

/** Nonzero opaque map handle; destroy it when finished. */
typedef uint32_t vk_undistort_map VK_HANDLE VK_HANDLE_DESTROY(vk_undistort_map_destroy);
#define VK_INVALID_UNDISTORT_MAP ((vk_undistort_map)0)

/** Version of the VisionKit C ABI. */
VK_API uint32_t vk_version(void);

/** Projects Materia camera-frame metre points. Points behind the camera are invalid. */
VK_API vk_result vk_project_points(const vk_camera_model *model,
                                   const vk_point3 *points VK_IN_ARRAY(count), uint32_t count,
                                   vk_pixel *out_pixels VK_OUT_ARRAY(count));
/** Undistorts pixels and returns unit rays in Materia's camera frame. */
VK_API vk_result vk_unproject_points(const vk_camera_model *model,
                                     const vk_pixel *pixels VK_IN_ARRAY(count), uint32_t count,
                                     vk_ray3 *out_rays VK_OUT_ARRAY(count));
/** Creates a reusable map and reports its rectified, distortion-free model. */
VK_API vk_result vk_undistort_map_create(const vk_camera_model *model,
                                        uint32_t policy,
                                        vk_undistort_map *out_map VK_OUT VK_OWNED,
                                        vk_camera_model *out_rectified);
VK_API void vk_undistort_map_destroy(vk_undistort_map map);
/** Remaps into caller-owned storage. Depth32f uses nearest-neighbour sampling. */
VK_API vk_result vk_undistort_image(vk_undistort_map map,
                                    const vk_image_view *src,
                                    const vk_image_view *dst);

#ifdef __cplusplus
}
#endif

#endif

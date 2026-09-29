#include "visionkit.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <climits>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace {
struct Map { uint32_t width, height; cv::Mat x, y; };
std::mutex maps_mutex;
std::unordered_map<uint32_t, std::shared_ptr<Map>> maps;
uint32_t next_map = 1;
bool vk_finite(double x) { return std::isfinite(x); }
bool valid_model(const vk_camera_model *m) {
    if (!m || m->struct_size < sizeof(vk_camera_model) || !m->width || !m->height ||
        m->width >= 32767 || m->height >= 32767 || !vk_finite(m->fx) ||
        !vk_finite(m->fy) || !vk_finite(m->cx) || !vk_finite(m->cy) ||
        m->fx <= 0 || m->fy <= 0 ||
        (m->distortion_model != VK_DISTORTION_NONE &&
         m->distortion_model != VK_DISTORTION_PLUMB_BOB)) return false;
    for (double v : {m->k1, m->k2, m->p1, m->p2, m->k3})
        if (!vk_finite(v)) return false;
    return true;
}
cv::Mat intrinsics(const vk_camera_model &m) {
    cv::Mat k = cv::Mat::eye(3, 3, CV_64F);
    k.at<double>(0, 0) = m.fx; k.at<double>(1, 1) = m.fy;
    k.at<double>(0, 2) = m.cx; k.at<double>(1, 2) = m.cy;
    return k;
}
cv::Mat distortion(const vk_camera_model &m) {
    if (m.distortion_model == VK_DISTORTION_NONE) return cv::Mat();
    cv::Mat d(1, 5, CV_64F);
    d.at<double>(0, 0) = m.k1; d.at<double>(0, 1) = m.k2;
    d.at<double>(0, 2) = m.p1; d.at<double>(0, 3) = m.p2;
    d.at<double>(0, 4) = m.k3;
    return d;
}
void configure_threads() {
    static const bool configured = [] { cv::setNumThreads(VK_OPENCV_THREADS); return true; }();
    (void)configured;
}
int image_type(uint32_t format) {
    switch (format) {
    case VK_PIXEL_RGB8: return CV_8UC3;
    case VK_PIXEL_GRAY8: return CV_8UC1;
    case VK_PIXEL_DEPTH32F: return CV_32FC1;
    default: return -1;
    }
}
bool valid_image(const vk_image_view *v, uint32_t width, uint32_t height) {
    if (!v || v->struct_size < sizeof(vk_image_view) || !v->data ||
        v->width != width || v->height != height) return false;
    const int type = image_type(v->pixel_format);
    if (type < 0) return false;
    const uint64_t row = uint64_t(width) * CV_ELEM_SIZE(type);
    return v->stride_bytes >= row &&
        uint64_t(v->stride_bytes) * (height - 1) + row <= v->buffer_bytes;
}
}

extern "C" VK_API uint32_t vk_version(void) {
    configure_threads();
    return 1;
}

extern "C" VK_API vk_result vk_project_points(const vk_camera_model *model,
        const vk_point3 *points, uint32_t count, vk_pixel *out_pixels) {
    if (!valid_model(model) || !count || !points || !out_pixels || count > INT_MAX)
        return VK_ERROR_INVALID_ARGUMENT;
    try {
        configure_threads();
        std::vector<cv::Point3d> input;
        input.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            const auto &p = points[i];
            if (!vk_finite(p.x) || !vk_finite(p.y) || !vk_finite(p.z) || p.x <= 0)
                return VK_ERROR_INVALID_ARGUMENT;
            // Materia (forward, left, up) -> OpenCV (right, down, forward).
            input.emplace_back(-p.y, -p.z, p.x);
        }
        std::vector<cv::Point2d> pixels;
        cv::projectPoints(input, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
                          intrinsics(*model), distortion(*model), pixels);
        for (uint32_t i = 0; i < count; ++i) {
            if (!vk_finite(pixels[i].x) || !vk_finite(pixels[i].y)) return VK_ERROR_BACKEND;
            out_pixels[i] = {pixels[i].x, pixels[i].y};
        }
        return VK_OK;
    } catch (const std::bad_alloc &) { return VK_ERROR_OUT_OF_MEMORY;
    } catch (const cv::Exception &) { return VK_ERROR_BACKEND; }
}

extern "C" VK_API vk_result vk_unproject_points(const vk_camera_model *model,
        const vk_pixel *pixels, uint32_t count, vk_ray3 *out_rays) {
    if (!valid_model(model) || !count || !pixels || !out_rays || count > INT_MAX)
        return VK_ERROR_INVALID_ARGUMENT;
    try {
        configure_threads();
        std::vector<cv::Point2d> input;
        input.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            if (!vk_finite(pixels[i].x) || !vk_finite(pixels[i].y)) return VK_ERROR_INVALID_ARGUMENT;
            input.emplace_back(pixels[i].x, pixels[i].y);
        }
        std::vector<cv::Point2d> normalized;
        cv::undistortPoints(input, normalized, intrinsics(*model), distortion(*model));
        for (uint32_t i = 0; i < count; ++i) {
            const double x = normalized[i].x, y = normalized[i].y;
            const double length = std::sqrt(1 + x*x + y*y);
            if (!vk_finite(length) || length == 0) return VK_ERROR_BACKEND;
            out_rays[i] = {1/length, -x/length, -y/length};
        }
        return VK_OK;
    } catch (const std::bad_alloc &) { return VK_ERROR_OUT_OF_MEMORY;
    } catch (const cv::Exception &) { return VK_ERROR_BACKEND; }
}

extern "C" VK_API vk_result vk_undistort_map_create(const vk_camera_model *model,
        uint32_t policy, vk_undistort_map *out_map, vk_camera_model *out_rectified) {
    if (!valid_model(model) || !out_map || !out_rectified ||
        out_rectified->struct_size < sizeof(vk_camera_model) ||
        (policy != VK_KEEP_ALL_PIXELS && policy != VK_CROP_VALID))
        return VK_ERROR_INVALID_ARGUMENT;
    *out_map = VK_INVALID_UNDISTORT_MAP;
    try {
        configure_threads();
        const cv::Size size(int(model->width), int(model->height));
        const cv::Mat k = intrinsics(*model), d = distortion(*model);
        cv::Mat new_k = model->distortion_model == VK_DISTORTION_NONE ? k :
            cv::getOptimalNewCameraMatrix(k, d, size,
                policy == VK_KEEP_ALL_PIXELS ? 1.0 : 0.0, size);
        auto mapping = std::make_shared<Map>();
        mapping->width = model->width;
        mapping->height = model->height;
        cv::initUndistortRectifyMap(k, d, cv::Mat(), new_k, size, CV_32FC1,
                                    mapping->x, mapping->y);
        vk_camera_model rectified = *model;
        rectified.distortion_model = VK_DISTORTION_NONE;
        rectified.fx = new_k.at<double>(0, 0);
        rectified.fy = new_k.at<double>(1, 1);
        rectified.cx = new_k.at<double>(0, 2);
        rectified.cy = new_k.at<double>(1, 2);
        rectified.k1 = rectified.k2 = rectified.p1 = rectified.p2 = rectified.k3 = 0;
        if (!valid_model(&rectified)) return VK_ERROR_BACKEND;
        std::lock_guard<std::mutex> lock(maps_mutex);
        if (!next_map) return VK_ERROR_OUT_OF_MEMORY;
        const uint32_t id = next_map++;
        maps.emplace(id, std::move(mapping));
        *out_rectified = rectified;
        *out_map = id;
        return VK_OK;
    } catch (const std::bad_alloc &) { return VK_ERROR_OUT_OF_MEMORY;
    } catch (const cv::Exception &) { return VK_ERROR_BACKEND; }
}

extern "C" VK_API void vk_undistort_map_destroy(vk_undistort_map map) {
    std::lock_guard<std::mutex> lock(maps_mutex);
    maps.erase(map);
}

extern "C" VK_API vk_result vk_undistort_image(vk_undistort_map map,
        const vk_image_view *src, const vk_image_view *dst) {
    std::shared_ptr<Map> mapping;
    {
        std::lock_guard<std::mutex> lock(maps_mutex);
        const auto it = maps.find(map);
        if (it == maps.end()) return VK_ERROR_INVALID_HANDLE;
        mapping = it->second;
    }
    if (!valid_image(src, mapping->width, mapping->height) ||
        !valid_image(dst, mapping->width, mapping->height) ||
        src->pixel_format != dst->pixel_format) return VK_ERROR_INVALID_ARGUMENT;
    const uintptr_t sb = reinterpret_cast<uintptr_t>(src->data);
    const uintptr_t db = reinterpret_cast<uintptr_t>(dst->data);
    const uint64_t sn = src->buffer_bytes;
    const uint64_t dn = dst->buffer_bytes;
    if (sb <= db ? db - sb < sn : sb - db < dn) return VK_ERROR_INVALID_ARGUMENT;
    try {
        configure_threads();
        const int type = image_type(src->pixel_format);
        cv::Mat input(int(src->height), int(src->width), type, src->data, src->stride_bytes);
        cv::Mat output(int(dst->height), int(dst->width), type, dst->data, dst->stride_bytes);
        const int interpolation = type == CV_32FC1 ? cv::INTER_NEAREST : cv::INTER_LINEAR;
        cv::remap(input, output, mapping->x, mapping->y, interpolation,
                  cv::BORDER_CONSTANT, cv::Scalar(0));
        return VK_OK;
    } catch (const std::bad_alloc &) { return VK_ERROR_OUT_OF_MEMORY;
    } catch (const cv::Exception &) { return VK_ERROR_BACKEND; }
}

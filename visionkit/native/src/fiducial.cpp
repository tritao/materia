#include "visionkit.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <algorithm>
#include <climits>
#include <cmath>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {
bool valid_model(const vk_camera_model *m) {
    if (!m || m->struct_size < sizeof(*m) || !m->width || !m->height ||
        m->width >= 32767 || m->height >= 32767 ||
        !std::isfinite(m->fx) || !std::isfinite(m->fy) ||
        !std::isfinite(m->cx) || !std::isfinite(m->cy) ||
        m->fx <= 0 || m->fy <= 0 ||
        (m->distortion_model != VK_DISTORTION_NONE &&
         m->distortion_model != VK_DISTORTION_PLUMB_BOB)) return false;
    for (double v : {m->k1, m->k2, m->p1, m->p2, m->k3})
        if (!std::isfinite(v)) return false;
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
cv::Matx33d camera_basis() {
    // OpenCV (right, down, forward) -> Materia (forward, left, up).
    return cv::Matx33d(0, 0, 1, -1, 0, 0, 0, -1, 0);
}
cv::Matx33d marker_basis() {
    // Materia marker (+X into scene, +Y left, +Z up) -> IPPE
    // (+X right, +Y up, +Z toward viewer). This is a proper rotation.
    return cv::Matx33d(0, -1, 0, 0, 0, 1, -1, 0, 0);
}
vk_pose3 pose_from_cv(const cv::Mat &rotation_vec, const cv::Mat &translation,
                      bool square) {
    cv::Mat rmat;
    cv::Rodrigues(rotation_vec, rmat);
    cv::Matx33d r = camera_basis() * cv::Matx33d(rmat);
    if (square) r = r * marker_basis();
    const cv::Vec3d t = camera_basis() * cv::Vec3d(translation);
    vk_pose3 pose{};
    pose.struct_size = sizeof(pose);
    pose.x = t[0]; pose.y = t[1]; pose.z = t[2];
    const double trace = r(0,0) + r(1,1) + r(2,2);
    if (trace > 0) {
        const double s = std::sqrt(trace + 1.0) * 2;
        pose.qw = 0.25*s;
        pose.qx = (r(2,1)-r(1,2))/s;
        pose.qy = (r(0,2)-r(2,0))/s;
        pose.qz = (r(1,0)-r(0,1))/s;
    } else if (r(0,0) > r(1,1) && r(0,0) > r(2,2)) {
        const double s = std::sqrt(1+r(0,0)-r(1,1)-r(2,2))*2;
        pose.qw = (r(2,1)-r(1,2))/s; pose.qx = 0.25*s;
        pose.qy = (r(0,1)+r(1,0))/s; pose.qz = (r(0,2)+r(2,0))/s;
    } else if (r(1,1) > r(2,2)) {
        const double s = std::sqrt(1+r(1,1)-r(0,0)-r(2,2))*2;
        pose.qw = (r(0,2)-r(2,0))/s; pose.qx = (r(0,1)+r(1,0))/s;
        pose.qy = 0.25*s; pose.qz = (r(1,2)+r(2,1))/s;
    } else {
        const double s = std::sqrt(1+r(2,2)-r(0,0)-r(1,1))*2;
        pose.qw = (r(1,0)-r(0,1))/s; pose.qx = (r(0,2)+r(2,0))/s;
        pose.qy = (r(1,2)+r(2,1))/s; pose.qz = 0.25*s;
    }
    return pose;
}
bool valid_square(const vk_point3 *p) {
    const double s = p[0].y - p[1].y;
    const double tolerance = 1e-7 * std::max(1.0, std::abs(s));
    return s > 0 && std::abs(p[0].x) < tolerance &&
        std::abs(p[1].x) < tolerance && std::abs(p[2].x) < tolerance &&
        std::abs(p[3].x) < tolerance &&
        std::abs(p[0].y-p[3].y) < tolerance &&
        std::abs(p[1].y-p[2].y) < tolerance &&
        std::abs(p[0].z-p[1].z) < tolerance &&
        std::abs(p[2].z-p[3].z) < tolerance &&
        std::abs(p[0].z-p[3].z-s) < tolerance &&
        std::abs(p[0].y+p[1].y) < tolerance &&
        std::abs(p[0].z+p[3].z) < tolerance;
}
struct Detector { cv::aruco::ArucoDetector native; explicit Detector(
    const cv::aruco::Dictionary &d, const cv::aruco::DetectorParameters &p)
    : native(d, p) {} };
std::mutex detector_mutex;
std::unordered_map<uint32_t, std::shared_ptr<Detector>> detectors;
uint32_t next_detector = 1;
}

extern "C" VK_API vk_result vk_solve_pnp(const vk_camera_model *model,
    const vk_point3 *object_points, const vk_pixel *image_points, uint32_t count,
    uint32_t method, vk_pose3 *out_camera_T_object, double *out_reprojection_errors) {
    if (!valid_model(model) || !object_points || !image_points || !out_camera_T_object ||
        !out_reprojection_errors || out_camera_T_object->struct_size < sizeof(vk_pose3) ||
        count < 4 || count > INT_MAX ||
        (method != VK_PNP_ITERATIVE && method != VK_PNP_IPPE_SQUARE) ||
        (method == VK_PNP_IPPE_SQUARE && (count != 4 || !valid_square(object_points))))
        return VK_ERROR_INVALID_ARGUMENT;
    try {
        if (vk_version() != 1) return VK_ERROR_BACKEND;
        std::vector<cv::Point3d> object;
        std::vector<cv::Point2d> image;
        object.reserve(count); image.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            const auto &p = object_points[i];
            const auto &q = image_points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                !std::isfinite(q.x) || !std::isfinite(q.y)) return VK_ERROR_INVALID_ARGUMENT;
            if (method == VK_PNP_IPPE_SQUARE) object.emplace_back(-p.y, p.z, -p.x);
            else object.emplace_back(p.x, p.y, p.z);
            image.emplace_back(q.x, q.y);
        }
        cv::Mat rvec, tvec;
        bool solved = cv::solvePnP(object, image, intrinsics(*model),
            distortion(*model), rvec, tvec, false,
            method == VK_PNP_IPPE_SQUARE ? cv::SOLVEPNP_IPPE_SQUARE : cv::SOLVEPNP_ITERATIVE);
        if (!solved && method == VK_PNP_IPPE_SQUARE)
            solved = cv::solvePnP(object, image, intrinsics(*model),
                distortion(*model), rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        if (!solved) return VK_ERROR_BACKEND;
        std::vector<cv::Point2d> projected;
        cv::projectPoints(object, rvec, tvec, intrinsics(*model), distortion(*model), projected);
        double sum_squared = 0;
        for (uint32_t i = 0; i < count; ++i)
            sum_squared += cv::normL2Sqr<double>(projected[i] - image[i]);
        if (method == VK_PNP_IPPE_SQUARE) {
            // Near a front-facing square, IPPE's two planar solutions can have
            // nearly equal error. The iterative solution is more stable there.
            cv::Mat refined_r, refined_t;
            if (cv::solvePnP(object, image, intrinsics(*model), distortion(*model),
                             refined_r, refined_t, false, cv::SOLVEPNP_ITERATIVE)) {
                std::vector<cv::Point2d> refined_pixels;
                cv::projectPoints(object, refined_r, refined_t, intrinsics(*model),
                                  distortion(*model), refined_pixels);
                double refined_squared = 0;
                for (uint32_t i = 0; i < count; ++i)
                    refined_squared += cv::normL2Sqr<double>(refined_pixels[i] - image[i]);
                if (std::isfinite(refined_squared) &&
                    (!std::isfinite(sum_squared) || refined_squared <= sum_squared + 1e-6)) {
                    rvec = refined_r; tvec = refined_t;
                    projected = std::move(refined_pixels);
                }
            }
        }
        const vk_pose3 pose = pose_from_cv(rvec, tvec, method == VK_PNP_IPPE_SQUARE);
        if (!std::isfinite(pose.x) || !std::isfinite(pose.qw)) return VK_ERROR_BACKEND;
        for (uint32_t i = 0; i < count; ++i)
            out_reprojection_errors[i] = cv::norm(projected[i] - image[i]);
        *out_camera_T_object = pose;
        return VK_OK;
    } catch (const std::bad_alloc &) { return VK_ERROR_OUT_OF_MEMORY;
    } catch (const cv::Exception &) { return VK_ERROR_BACKEND;
    } catch (...) { return VK_ERROR_BACKEND; }
}

extern "C" VK_API vk_result vk_marker_detector_create(uint32_t dictionary,
    const vk_marker_detector_params *params, vk_marker_detector *out_detector) {
    if (!out_detector || (params && (params->struct_size < sizeof(*params) ||
        !std::isfinite(params->min_marker_perimeter_rate) ||
        params->min_marker_perimeter_rate <= 0 ||
        params->min_marker_perimeter_rate > 1 ||
        params->corner_refinement > 1))) return VK_ERROR_INVALID_ARGUMENT;
    *out_detector = VK_INVALID_MARKER_DETECTOR;
    cv::aruco::PredefinedDictionaryType code;
    switch (dictionary) {
    case VK_ARUCO_4X4_50: code = cv::aruco::DICT_4X4_50; break;
    case VK_ARUCO_5X5_100: code = cv::aruco::DICT_5X5_100; break;
    case VK_APRILTAG_36H11: code = cv::aruco::DICT_APRILTAG_36h11; break;
    default: return VK_ERROR_UNSUPPORTED;
    }
    try {
        if (vk_version() != 1) return VK_ERROR_BACKEND;
        cv::aruco::DetectorParameters native_params;
        if (params) {
            native_params.minMarkerPerimeterRate = params->min_marker_perimeter_rate;
            if (params->corner_refinement)
                native_params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        }
        auto detector = std::make_shared<Detector>(
            cv::aruco::getPredefinedDictionary(code), native_params);
        std::lock_guard<std::mutex> lock(detector_mutex);
        if (!next_detector) return VK_ERROR_OUT_OF_MEMORY;
        const uint32_t id = next_detector++;
        detectors.emplace(id, std::move(detector));
        *out_detector = id;
        return VK_OK;
    } catch (const std::bad_alloc &) { return VK_ERROR_OUT_OF_MEMORY;
    } catch (const cv::Exception &) { return VK_ERROR_BACKEND;
    } catch (...) { return VK_ERROR_BACKEND; }
}

extern "C" VK_API void vk_marker_detector_destroy(vk_marker_detector detector) {
    try {
        std::lock_guard<std::mutex> lock(detector_mutex);
        detectors.erase(detector);
    } catch (...) {}
}

extern "C" VK_API vk_result vk_marker_detect(vk_marker_detector detector,
    const vk_image_view *image, const vk_camera_model *model, double marker_size_m,
    vk_marker_observation *out_markers, uint32_t capacity, uint32_t *out_count) {
    std::shared_ptr<Detector> native;
    try {
        std::lock_guard<std::mutex> lock(detector_mutex);
        const auto it = detectors.find(detector);
        if (it == detectors.end()) return VK_ERROR_INVALID_HANDLE;
        native = it->second;
    } catch (...) { return VK_ERROR_BACKEND; }
    if (!valid_model(model) || !image || image->struct_size < sizeof(*image) ||
        !image->data || image->width != model->width || image->height != model->height ||
        (image->pixel_format != VK_PIXEL_GRAY8 && image->pixel_format != VK_PIXEL_RGB8) ||
        !std::isfinite(marker_size_m) || marker_size_m <= 0 || !out_count ||
        (capacity && !out_markers)) return VK_ERROR_INVALID_ARGUMENT;
    const uint32_t channels = image->pixel_format == VK_PIXEL_RGB8 ? 3 : 1;
    if (image->stride_bytes < uint64_t(image->width)*channels ||
        uint64_t(image->stride_bytes)*(image->height-1) +
        uint64_t(image->width)*channels > image->buffer_bytes) return VK_ERROR_INVALID_ARGUMENT;
    try {
        cv::Mat input(int(image->height), int(image->width),
            channels == 3 ? CV_8UC3 : CV_8UC1, image->data, image->stride_bytes);
        cv::Mat gray;
        if (channels == 3) cv::cvtColor(input, gray, cv::COLOR_RGB2GRAY);
        else gray = input;
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        native->native.detectMarkers(gray, corners, ids);
        std::vector<vk_marker_observation> observations;
        for (size_t i = 0; i < ids.size(); ++i) {
            if (corners[i].size() != 4 || ids[i] < 0) continue;
            const double half = marker_size_m*0.5;
            const vk_point3 object[4] = {{0,half,half}, {0,-half,half},
                {0,-half,-half}, {0,half,-half}};
            vk_pixel pixels[4]{};
            for (int j = 0; j < 4; ++j) pixels[j] = {corners[i][j].x, corners[i][j].y};
            vk_pose3 pose{}; pose.struct_size = sizeof(pose);
            double errors[4]{};
            if (vk_solve_pnp(model, object, pixels, 4, VK_PNP_IPPE_SQUARE,
                             &pose, errors) != VK_OK) continue;
            vk_marker_observation obs{};
            obs.struct_size = sizeof(obs);
            obs.id = uint32_t(ids[i]);
            for (int j = 0; j < 4; ++j) obs.corners[j] = pixels[j];
            obs.camera_T_marker = pose;
            double sum_squared = 0;
            for (double error : errors) sum_squared += error*error;
            obs.rms_reprojection_error = std::sqrt(sum_squared/4);
            if (!std::isfinite(obs.rms_reprojection_error)) continue;
            // Monotone score: 1 at zero RMS, 0.5 at 2 px RMS.
            obs.confidence = 1.0/(1.0 + obs.rms_reprojection_error/2.0);
            observations.push_back(obs);
        }
        *out_count = uint32_t(observations.size());
        if (observations.size() > capacity) return VK_ERROR_LIMIT;
        for (size_t i = 0; i < observations.size(); ++i)
            out_markers[i] = observations[i];
        return VK_OK;
    } catch (const std::bad_alloc &) { return VK_ERROR_OUT_OF_MEMORY;
    } catch (const cv::Exception &) { return VK_ERROR_BACKEND;
    } catch (...) { return VK_ERROR_BACKEND; }
}

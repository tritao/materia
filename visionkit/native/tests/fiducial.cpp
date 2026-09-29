#include "visionkit.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

static vk_camera_model camera() {
    vk_camera_model m{}; m.struct_size = sizeof(m);
    m.width = 640; m.height = 480;
    m.fx = 500; m.fy = 500; m.cx = 320; m.cy = 240;
    return m;
}
static void near(double a, double b, double tol) {
    if (std::abs(a-b) >= tol) std::fprintf(stderr, "near: %.9f vs %.9f (tol %.9f)\n", a, b, tol);
    assert(std::abs(a-b) < tol);
}

static void pnp() {
    auto m = camera();
    constexpr double half = 0.1;
    const vk_point3 object[4] = {{0,half,half},{0,-half,half},
        {0,-half,-half},{0,half,-half}};
    vk_point3 in_camera[4];
    for (int i = 0; i < 4; ++i)
        in_camera[i] = {object[i].x+1, object[i].y, object[i].z};
    vk_pixel pixels[4]{};
    assert(vk_project_points(&m, in_camera, 4, pixels) == VK_OK);
    vk_pose3 pose{}; pose.struct_size = sizeof(pose);
    double errors[4]{};
    assert(vk_solve_pnp(&m, object, pixels, 4, VK_PNP_IPPE_SQUARE,
                        &pose, errors) == VK_OK);
    near(pose.x, 1, 1e-5); near(pose.y, 0, 1e-5); near(pose.z, 0, 1e-5);
    near(std::abs(pose.qw), 1, 1e-5);
    for (double error : errors) near(error, 0, 1e-5);
    // Front-facing, laterally offset markers are a planar PnP ambiguity case.
    for (int i = 0; i < 4; ++i)
        in_camera[i] = {1, object[i].y+0.25, object[i].z-0.12};
    assert(vk_project_points(&m, in_camera, 4, pixels) == VK_OK);
    assert(vk_solve_pnp(&m, object, pixels, 4, VK_PNP_IPPE_SQUARE,
                        &pose, errors) == VK_OK);
    near(pose.x, 1, 1e-4); near(pose.y, 0.25, 1e-4); near(pose.z, -0.12, 1e-4);
    near(std::abs(pose.qw), 1, 1e-4);
    near(pose.qx, 0, 1e-4); near(pose.qy, 0, 1e-4); near(pose.qz, 0, 1e-4);
    // A 90 degree in-plane roll has an absolute orientation expectation.
    for (int i = 0; i < 4; ++i)
        in_camera[i] = {1, -object[i].z, object[i].y};
    assert(vk_project_points(&m, in_camera, 4, pixels) == VK_OK);
    assert(vk_solve_pnp(&m, object, pixels, 4, VK_PNP_IPPE_SQUARE,
                        &pose, errors) == VK_OK);
    near(std::abs(pose.qx), std::sqrt(0.5), 1e-4);
    near(std::abs(pose.qw), std::sqrt(0.5), 1e-4);
    assert(vk_solve_pnp(&m, object, pixels, 3, VK_PNP_IPPE_SQUARE,
                        &pose, errors) == VK_ERROR_INVALID_ARGUMENT);
    const vk_point3 arbitrary[6] = {{0,0,0},{0,0.2,0},{0,0,0.2},
        {0.1,0.1,0},{0.1,0,0.1},{0,0.1,0.1}};
    vk_point3 arbitrary_camera[6];
    for (int i = 0; i < 6; ++i)
        arbitrary_camera[i] = {arbitrary[i].x+1.2, arbitrary[i].y+0.05,
                               arbitrary[i].z-0.03};
    vk_pixel arbitrary_pixels[6]{};
    double arbitrary_errors[6]{};
    assert(vk_project_points(&m, arbitrary_camera, 6, arbitrary_pixels) == VK_OK);
    assert(vk_solve_pnp(&m, arbitrary, arbitrary_pixels, 6, VK_PNP_ITERATIVE,
                        &pose, arbitrary_errors) == VK_OK);
    near(pose.x, 1.2, 1e-4); near(pose.y, 0.05, 1e-4);
    near(pose.z, -0.03, 1e-4);
}

static void draw_marker(cv::Mat &canvas, const vk_camera_model &m,
                        cv::aruco::PredefinedDictionaryType dictionary,
                        int id, double x, double y, double size) {
    cv::Mat marker;
    cv::aruco::getPredefinedDictionary(dictionary).generateImageMarker(id, 200, marker);
    const double h = size/2;
    const vk_point3 corners[4] = {{x,y+h,h},{x,y-h,h},
        {x,y-h,-h},{x,y+h,-h}};
    vk_pixel pixel[4]{};
    assert(vk_project_points(&m, corners, 4, pixel) == VK_OK);
    const cv::Point2f from[4] = {{0,0},{199,0},{199,199},{0,199}};
    cv::Point2f to[4];
    for (int i = 0; i < 4; ++i)
        to[i] = cv::Point2f(float(pixel[i].x), float(pixel[i].y));
    cv::Mat warped(canvas.size(), CV_8UC1, cv::Scalar(255));
    cv::warpPerspective(marker, warped, cv::getPerspectiveTransform(from, to),
                        canvas.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT,
                        cv::Scalar(255));
    cv::min(canvas, warped, canvas);
}

static vk_image_view view(cv::Mat &image) {
    return {sizeof(vk_image_view), uint32_t(image.cols), uint32_t(image.rows),
        uint32_t(image.step), VK_PIXEL_GRAY8,
        uint32_t(image.step*image.rows), image.data};
}

static void detection() {
    auto m = camera();
    cv::Mat frame(int(m.height), int(m.width), CV_8UC1, cv::Scalar(255));
    draw_marker(frame, m, cv::aruco::DICT_4X4_50, 7, 1.0, -0.17, 0.18);
    draw_marker(frame, m, cv::aruco::DICT_4X4_50, 13, 1.1, 0.18, 0.18);
    auto image = view(frame);
    vk_marker_detector detector = 0;
    assert(vk_marker_detector_create(VK_ARUCO_4X4_50, nullptr, &detector) == VK_OK);
    assert(vk_opencv_threads() == VK_OPENCV_THREADS);
    vk_marker_observation found[4]{};
    uint32_t count = 0;
    assert(vk_marker_detect(detector, &image, &m, 0.18, found, 4, &count) == VK_OK);
    assert(count == 2);
    bool seven = false, thirteen = false;
    for (uint32_t i = 0; i < count; ++i) {
        assert(found[i].struct_size == sizeof(vk_marker_observation));
        assert(found[i].confidence > 0.5 && found[i].confidence <= 1);
        assert(found[i].rms_reprojection_error < 2);
        if (found[i].id == 7) {
            seven = true;
            near(found[i].camera_T_marker.x, 1.0, 0.05);
            near(found[i].camera_T_marker.y, -0.17, 0.05);
        }
        if (found[i].id == 13) thirteen = true;
    }
    assert(seven && thirteen);
    assert(vk_marker_detect(detector, &image, &m, 0.18, found, 1, &count) == VK_ERROR_LIMIT);
    assert(count == 2);
    vk_marker_detector_destroy(detector);
    assert(vk_marker_detect(detector, &image, &m, 0.18, found, 4, &count) == VK_ERROR_INVALID_HANDLE);

    // A wrong dictionary must not assign either marker's ID.
    assert(vk_marker_detector_create(VK_APRILTAG_36H11, nullptr, &detector) == VK_OK);
    assert(vk_marker_detect(detector, &image, &m, 0.18, found, 4, &count) == VK_OK);
    assert(count == 0);
    vk_marker_detector_destroy(detector);

    cv::Mat april(int(m.height), int(m.width), CV_8UC1, cv::Scalar(255));
    draw_marker(april, m, cv::aruco::DICT_APRILTAG_36h11, 5, 1.0, 0, 0.2);
    image = view(april);
    assert(vk_marker_detector_create(VK_APRILTAG_36H11, nullptr, &detector) == VK_OK);
    assert(vk_marker_detect(detector, &image, &m, 0.2, found, 4, &count) == VK_OK);
    assert(count == 1 && found[0].id == 5);
    near(found[0].camera_T_marker.x, 1.0, 0.05);
    // Partial occlusion should fail closed, without a spurious pose.
    cv::rectangle(april, cv::Rect(280, 200, 80, 90), cv::Scalar(255), cv::FILLED);
    assert(vk_marker_detect(detector, &image, &m, 0.2, found, 4, &count) == VK_OK);
    assert(count == 0);
    vk_marker_detector_destroy(detector);
}

int main() { pnp(); detection(); }

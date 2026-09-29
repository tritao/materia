#include "visionkit.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <vector>

static void near(double a, double b, double tolerance = 1e-4) {
    assert(std::abs(a - b) < tolerance);
}

static vk_camera_model camera(bool distorted) {
    vk_camera_model m{};
    m.struct_size = sizeof(m);
    m.width = 320; m.height = 240;
    m.fx = 240; m.fy = 245; m.cx = 160; m.cy = 120;
    m.distortion_model = distorted ? VK_DISTORTION_PLUMB_BOB : VK_DISTORTION_NONE;
    m.k1 = distorted ? -0.23 : 0;
    m.k2 = distorted ? 0.07 : 0;
    m.p1 = distorted ? 0.001 : 0;
    m.p2 = distorted ? -0.002 : 0;
    m.k3 = distorted ? -0.01 : 0;
    return m;
}

static void geometry(bool distorted) {
    auto m = camera(distorted);
    const vk_point3 points[] = {{2, 0, 0}, {2, 0.2, -0.1}, {2, -0.3, 0.15}};
    vk_pixel pixels[3]{};
    vk_ray3 rays[3]{};
    assert(vk_project_points(&m, points, 3, pixels) == VK_OK);
    near(pixels[0].x, m.cx, 1e-10);
    near(pixels[0].y, m.cy, 1e-10);
    assert(vk_unproject_points(&m, pixels, 3, rays) == VK_OK);
    near(rays[0].x, 1, 1e-10);
    near(rays[0].y, 0, 1e-10);
    near(rays[0].z, 0, 1e-10);
    for (int i = 0; i < 3; ++i) {
        const double length = std::sqrt(points[i].x*points[i].x +
            points[i].y*points[i].y + points[i].z*points[i].z);
        near(rays[i].x, points[i].x/length);
        near(rays[i].y, points[i].y/length);
        near(rays[i].z, points[i].z/length);
    }
    // A bent distorted pixel row straightens into a constant-height ray row.
    for (int i = -5; i <= 5; ++i) {
        const vk_point3 p{2, i*0.05, 0.2};
        vk_pixel pixel{};
        vk_ray3 ray{};
        assert(vk_project_points(&m, &p, 1, &pixel) == VK_OK);
        assert(vk_unproject_points(&m, &pixel, 1, &ray) == VK_OK);
        near(ray.z/ray.x, 0.1);
    }
}

static void absolute_conventions_and_edges() {
    auto m = camera(false);
    const vk_point3 points[] = {{2, 0.5, 0}, {2, -0.5, 0},
        {2, 0, 0.5}, {2, 0, -0.5}};
    vk_pixel pixels[4]{};
    assert(vk_project_points(&m, points, 4, pixels) == VK_OK);
    near(pixels[0].x, 100); near(pixels[1].x, 220);
    near(pixels[2].y, 58.75); near(pixels[3].y, 181.25);
    const vk_pixel left_top{40, 30};
    vk_ray3 ray{};
    assert(vk_unproject_points(&m, &left_top, 1, &ray) == VK_OK);
    const double length = std::sqrt(1+0.5*0.5+(90.0/245)*(90.0/245));
    near(ray.x, 1/length); near(ray.y, 0.5/length);
    near(ray.z, (90.0/245)/length);

    // A wide image exercises the inverse distortion near an actual corner.
    m.width=1920; m.height=1080; m.fx=650; m.fy=640; m.cx=960; m.cy=540;
    m.distortion_model=VK_DISTORTION_PLUMB_BOB;
    m.k1=-0.23; m.k2=0.07; m.k3=-0.01; m.p1=0.001; m.p2=-0.002;
    const vk_point3 edge{1, 1.2, 0.65};
    vk_pixel distorted{};
    assert(vk_project_points(&m, &edge, 1, &distorted)==VK_OK);
    assert(vk_unproject_points(&m, &distorted, 1, &ray)==VK_OK);
    const double norm=std::sqrt(1+1.2*1.2+0.65*0.65);
    near(ray.y, 1.2/norm, 1e-7); near(ray.z, 0.65/norm, 1e-7);

    m.k1=-0.5; m.k2=m.k3=m.p1=m.p2=0;
    const vk_point3 folded{1, -std::sqrt(2.0), 0};
    assert(vk_project_points(&m, &folded, 1, &distorted)==VK_ERROR_INVALID_ARGUMENT);
}

static void remap_and_stride() {
    auto m = camera(false);
    vk_undistort_map handle = 0;
    vk_camera_model rect{}; rect.struct_size = sizeof(rect);
    assert(vk_undistort_map_create(&m, VK_KEEP_ALL_PIXELS, &handle, &rect) == VK_OK);
    assert(handle != 0 && rect.distortion_model == VK_DISTORTION_NONE);
    near(rect.fx, m.fx); near(rect.cx, m.cx);
    const uint32_t stride = m.width + 13;
    std::vector<uint8_t> src(stride*m.height, 0x7f), dst(stride*m.height, 0xee);
    for (uint32_t y = 0; y < m.height; ++y)
        for (uint32_t x = 0; x < m.width; ++x)
            src[y*stride+x] = uint8_t((x + y) % 251);
    vk_image_view a{sizeof(vk_image_view), m.width, m.height, stride,
                    VK_PIXEL_GRAY8, uint32_t(src.size()), src.data()};
    vk_image_view b{sizeof(vk_image_view), m.width, m.height, stride,
                    VK_PIXEL_GRAY8, uint32_t(dst.size()), dst.data()};
    assert(vk_undistort_image(handle, &a, &b) == VK_OK);
    for (uint32_t y = 0; y < m.height; ++y) {
        for (uint32_t x = 0; x < m.width; ++x)
            assert(dst[y*stride+x] == src[y*stride+x]);
        for (uint32_t x = m.width; x < stride; ++x)
            assert(dst[y*stride+x] == 0xee);
    }
    assert(vk_undistort_image(handle, &a, &a) == VK_ERROR_INVALID_ARGUMENT);
    const uint32_t rgb_stride = m.width*3 + 7;
    std::vector<uint8_t> rgb_src(rgb_stride*m.height, 41), rgb_dst(rgb_stride*m.height, 0);
    vk_image_view rgb_a{sizeof(vk_image_view), m.width, m.height, rgb_stride,
                        VK_PIXEL_RGB8, uint32_t(rgb_src.size()), rgb_src.data()};
    vk_image_view rgb_b{sizeof(vk_image_view), m.width, m.height, rgb_stride,
                        VK_PIXEL_RGB8, uint32_t(rgb_dst.size()), rgb_dst.data()};
    assert(vk_undistort_image(handle, &rgb_a, &rgb_b) == VK_OK);
    assert(rgb_dst[(m.height/2)*rgb_stride+(m.width/2)*3] == 41);
    rgb_b.pixel_format = VK_PIXEL_GRAY8;
    assert(vk_undistort_image(handle, &rgb_a, &rgb_b) == VK_ERROR_INVALID_ARGUMENT);
    b.buffer_bytes = 2;
    assert(vk_undistort_image(handle, &a, &b) == VK_ERROR_INVALID_ARGUMENT);
    vk_undistort_map_destroy(handle);
    assert(vk_undistort_image(handle, &a, &b) == VK_ERROR_INVALID_HANDLE);
}

static void distorted_maps() {
    auto m = camera(true);
    for (uint32_t policy : {VK_KEEP_ALL_PIXELS, VK_CROP_VALID}) {
        vk_undistort_map handle = 0;
        vk_camera_model rect{}; rect.struct_size = sizeof(rect);
        assert(vk_undistort_map_create(&m, policy, &handle, &rect) == VK_OK);
        assert(rect.distortion_model == VK_DISTORTION_NONE);
        assert(rect.k1 == 0 && rect.k2 == 0);
        const uint32_t stride = m.width * 4 + 16;
        std::vector<uint8_t> src(stride*m.height), dst(stride*m.height);
        for (uint32_t y = 0; y < m.height; ++y)
            for (uint32_t x = 0; x < m.width; ++x) {
                const float value = float((x + y) % 3 + 1);
                *reinterpret_cast<float *>(src.data() + y*stride + x*4) = value;
            }
        vk_image_view a{sizeof(vk_image_view), m.width, m.height, stride,
                        VK_PIXEL_DEPTH32F, uint32_t(src.size()), src.data()};
        vk_image_view b{sizeof(vk_image_view), m.width, m.height, stride,
                        VK_PIXEL_DEPTH32F, uint32_t(dst.size()), dst.data()};
        b.stride_bytes = stride+1;
        assert(vk_undistort_image(handle, &a, &b) == VK_ERROR_INVALID_ARGUMENT);
        b.stride_bytes = stride;
        assert(vk_undistort_image(handle, &a, &b) == VK_OK);
        for (uint32_t y = 0; y < m.height; ++y)
            for (uint32_t x = 0; x < m.width; ++x) {
                const float value = *reinterpret_cast<float *>(dst.data() + y*stride + x*4);
                assert(value == 0 || value == 1 || value == 2 || value == 3);
            }
        vk_undistort_map_destroy(handle);
    }
}

static void undistort_grid() {
    auto m = camera(true);
    vk_undistort_map handle = 0;
    vk_camera_model rect{}; rect.struct_size = sizeof(rect);
    assert(vk_undistort_map_create(&m, VK_KEEP_ALL_PIXELS, &handle, &rect) == VK_OK);
    std::vector<uint8_t> src(m.width*m.height), dst(src.size());
    for (int row = -2; row <= 2; ++row)
        for (int col = -30; col <= 30; ++col) {
            vk_point3 point{2, col*0.015, row*0.12};
            vk_pixel pixel{};
            assert(vk_project_points(&m, &point, 1, &pixel) == VK_OK);
            const int x = int(std::round(pixel.x)), y = int(std::round(pixel.y));
            for (int dy = -2; dy <= 2; ++dy)
                for (int dx = -2; dx <= 2; ++dx)
                    if (x+dx >= 0 && y+dy >= 0 && x+dx < int(m.width) && y+dy < int(m.height))
                        src[(y+dy)*m.width+x+dx] = 255;
        }
    vk_image_view a{sizeof(vk_image_view), m.width, m.height, m.width,
                    VK_PIXEL_GRAY8, uint32_t(src.size()), src.data()};
    vk_image_view b{sizeof(vk_image_view), m.width, m.height, m.width,
                    VK_PIXEL_GRAY8, uint32_t(dst.size()), dst.data()};
    assert(vk_undistort_image(handle, &a, &b) == VK_OK);
    for (int row = -2; row <= 2; ++row)
        for (int col = -30; col <= 30; col += 5) {
            vk_point3 point{2, col*0.015, row*0.12};
            vk_pixel pixel{};
            assert(vk_project_points(&rect, &point, 1, &pixel) == VK_OK);
            const int x = int(std::round(pixel.x)), y = int(std::round(pixel.y));
            assert(x >= 0 && y >= 0 && x < int(m.width) && y < int(m.height));
            assert(dst[y*m.width+x] > 100);
            if (row == 0 && y >= 6 && y+6 < int(m.height)) {
                assert(dst[(y-6)*m.width+x] < 100);
                assert(dst[(y+6)*m.width+x] < 100);
            }
        }
    vk_undistort_map_destroy(handle);
}

static void invalid_arguments() {
    auto m = camera(false);
    vk_pixel p{}; vk_ray3 r{}; vk_point3 point{1, 0, 0};
    assert(vk_project_points(nullptr, &point, 1, &p) == VK_ERROR_INVALID_ARGUMENT);
    assert(vk_project_points(&m, &point, 0, &p) == VK_ERROR_INVALID_ARGUMENT);
    point.x = -1;
    assert(vk_project_points(&m, &point, 1, &p) == VK_ERROR_INVALID_ARGUMENT);
    assert(vk_unproject_points(&m, nullptr, 1, &r) == VK_ERROR_INVALID_ARGUMENT);
    m.fx = 0;
    assert(vk_unproject_points(&m, &p, 1, &r) == VK_ERROR_INVALID_ARGUMENT);
    m = camera(false);
    m.struct_size = 0;
    assert(vk_project_points(&m, &point, 1, &p) == VK_ERROR_INVALID_ARGUMENT);
}

int main() {
    geometry(false);
    geometry(true);
    absolute_conventions_and_edges();
    remap_and_stride();
    distorted_maps();
    undistort_grid();
    invalid_arguments();
}

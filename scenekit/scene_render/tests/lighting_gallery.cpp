#include "nativekit.h"
#include "nativekit_scene_render.hpp"
#include "nativekit_window.h"

#include "scene_internal.hpp"

// Diagnostic capture: run under xvfb-run with an existing output directory.
// Produces eight orbit-*.ppm images; pass "dark" for the dark material variant.

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace {

using Vec = std::array<float, 3>;
constexpr float pi = 3.14159265358979323846f;

Vec add(Vec a, Vec b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
Vec scale(Vec a, float s) { return {a[0] * s, a[1] * s, a[2] * s}; }
float dot(Vec a, Vec b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
Vec cross(Vec a, Vec b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}
Vec unit(Vec a) { return scale(a, 1.0f / std::sqrt(dot(a, a))); }

struct Mesh {
    std::vector<nkscene::GeometryVertex> vertices;
    std::vector<Vec> normals;

    void triangle(Vec a, Vec b, Vec c, Vec na, Vec nb, Vec nc) {
        vertices.push_back({a}); vertices.push_back({b}); vertices.push_back({c});
        normals.push_back(na); normals.push_back(nb); normals.push_back(nc);
    }
    void flat(Vec a, Vec b, Vec c) {
        const auto n = unit(cross(add(b, scale(a, -1)), add(c, scale(a, -1))));
        triangle(a, b, c, n, n, n);
    }
    void box(Vec lo, Vec hi) {
        const Vec p[8] = {{lo[0],lo[1],lo[2]}, {hi[0],lo[1],lo[2]},
                          {hi[0],hi[1],lo[2]}, {lo[0],hi[1],lo[2]},
                          {lo[0],lo[1],hi[2]}, {hi[0],lo[1],hi[2]},
                          {hi[0],hi[1],hi[2]}, {lo[0],hi[1],hi[2]}};
        const int f[6][4] = {{0,3,2,1},{4,5,6,7},{0,1,5,4},
                             {1,2,6,5},{2,3,7,6},{3,0,4,7}};
        for (const auto &q : f) {
            flat(p[q[0]], p[q[1]], p[q[2]]);
            flat(p[q[0]], p[q[2]], p[q[3]]);
        }
    }
    void sphere(Vec center, float radius) {
        constexpr int latitude = 20, longitude = 32;
        const auto point = [&](int t, int p) {
            const float theta = pi * t / latitude, phi = 2 * pi * p / longitude;
            const Vec n{std::sin(theta)*std::cos(phi), std::sin(theta)*std::sin(phi),
                        std::cos(theta)};
            return std::pair{add(center, scale(n, radius)), n};
        };
        for (int t = 0; t < latitude; ++t)
            for (int p = 0; p < longitude; ++p) {
                const auto [a, na] = point(t,p);
                const auto [b, nb] = point(t+1,p);
                const auto [c, nc] = point(t+1,p+1);
                const auto [d, nd] = point(t,p+1);
                triangle(a,b,c,na,nb,nc); triangle(a,c,d,na,nc,nd);
            }
    }
    void cylinder(Vec center, float radius, float half_height) {
        constexpr int segments = 40;
        for (int i = 0; i < segments; ++i) {
            const float a = 2*pi*i/segments, b = 2*pi*(i+1)/segments;
            const Vec na{std::cos(a),std::sin(a),0}, nb{std::cos(b),std::sin(b),0};
            const Vec al = add(center,{radius*na[0],radius*na[1],-half_height});
            const Vec ah = add(center,{radius*na[0],radius*na[1],half_height});
            const Vec bl = add(center,{radius*nb[0],radius*nb[1],-half_height});
            const Vec bh = add(center,{radius*nb[0],radius*nb[1],half_height});
            triangle(al,bl,bh,na,nb,nb); triangle(al,bh,ah,na,nb,na);
            flat(add(center,{0,0,half_height}),ah,bh);
            flat(add(center,{0,0,-half_height}),bl,al);
        }
    }
};

std::array<float,16> view_projection(float yaw, float pitch, float aspect) {
    const Vec eye{10*std::cos(pitch)*std::cos(yaw),
                  10*std::cos(pitch)*std::sin(yaw), 10*std::sin(pitch)};
    const auto forward = unit(scale(eye,-1));
    const auto right = unit(cross(forward,{0,0,1}));
    const auto up = cross(right,forward);
    const std::array<float,16> view{right[0],up[0],-forward[0],0,
                                     right[1],up[1],-forward[1],0,
                                     right[2],up[2],-forward[2],0,
                                     -dot(right,eye),-dot(up,eye),dot(forward,eye),1};
    const float s = 1/std::tan(50*pi/360), near = 0.01f, far = 100.0f;
    const std::array<float,16> projection{s/aspect,0,0,0, 0,s,0,0,
        0,0,(far+near)/(near-far),-1, 0,0,2*far*near/(near-far),0};
    std::array<float,16> result{};
    for (int col=0; col<4; ++col) for (int row=0; row<4; ++row)
        for (int k=0; k<4; ++k)
            result[col*4+row] += projection[k*4+row]*view[col*4+k];
    return result;
}

void configure_studio(nkscene::SceneView &view, float yaw, float pitch) {
    const Vec eye{std::cos(pitch)*std::cos(yaw),std::cos(pitch)*std::sin(yaw),
                  std::sin(pitch)};
    const auto forward=scale(eye,-1), right=unit(cross(forward,{0,0,1}));
    const auto up=cross(right,forward);
    const std::array<std::array<float,4>,3> coin{{
        {0.6841049f,-0.12062616f,-0.7193398f,0.76f},
        {-0.6403416f,0.7631294f,0.087155744f,0.34f},
        {-0.7544065f,-0.63302225f,-0.17364818f,0.50f}}};
    view.studio_lighting.enabled=true;
    view.studio_lighting.ambient_sky={0.22f,0.23f,0.24f,0};
    view.studio_lighting.ambient_ground={0.16f,0.16f,0.17f,0};
    for (int i=0;i<3;++i) {
        const auto &l=coin[i];
        const auto world=add(add(scale(right,-l[0]),scale(up,-l[1])),scale(forward,l[2]));
        view.studio_lighting.directions[i]={world[0],world[1],world[2],l[3]};
    }
}

bool wait_for_surface(nk_surface surface) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while (std::chrono::steady_clock::now()<deadline) {
        nk_event event{}; event.struct_size=sizeof(event);
        if (nk_poll_event(&event)!=NK_OK) return false;
        const bool ready=event.kind==NK_EVENT_SURFACE_READY && event.source==surface;
        nk_event_release(&event);
        if (ready) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}
}

int main(int argc, char **argv) {
    if (argc < 2 || argc > 3 || (argc == 3 && std::string(argv[2]) != "dark")) return 2;
    nk_init_options init{}; init.struct_size=sizeof(init); init.api_version=NK_API_VERSION;
    if (nk_init(&init)!=NK_OK) return 1;
    nk_window window{}; nk_surface surface{}; nkgpu_renderer renderer{};
    nk_window_options options{}; options.struct_size=sizeof(options);
    options.width=768; options.height=400; options.title="SceneKit lighting gallery";
    int status=1;
    do {
        if (nk_window_create(&options,&window)!=NK_OK ||
            nkgpu_surface_create(window,options.width,options.height,&surface)!=NKGPU_OK ||
            !wait_for_surface(surface) || nkgpu_renderer_create(surface,&renderer)!=NKGPU_OK)
            break;
        auto scene=std::make_shared<nkscene::Scene>();
        Mesh mesh;
        mesh.sphere({-2.4f,0,0},0.85f);
        mesh.cylinder({0,0,0},0.78f,0.85f);
        mesh.box({1.65f,-0.9f,-0.85f},{3.25f,0.9f,-0.55f});
        mesh.box({1.65f,-0.9f,-0.55f},{1.95f,0.9f,0.85f});
        mesh.box({2.95f,-0.9f,-0.55f},{3.25f,0.9f,0.85f});
        mesh.box({1.95f,0.6f,-0.55f},{2.95f,0.9f,0.85f});
        const auto geometry=scene->reserve_geometry_id();
        auto &resource=scene->geometry_store().create(geometry);
        resource.edit_payload().vertices=mesh.vertices;
        nkscene::GeometryVertexStream normals;
        normals.semantic=nkscene::VertexSemantic::Normal;
        normals.format=nkscene::VertexFormat::Float32x3;
        normals.stride=sizeof(Vec); normals.count=mesh.normals.size();
        normals.data.resize(sizeof(Vec)*mesh.normals.size());
        std::memcpy(normals.data.data(),mesh.normals.data(),normals.data.size());
        resource.edit_payload().streams.push_back(std::move(normals));
        resource.bounds.valid=true;
        resource.bounds.minimum={-3.3f,-1.0f,-1.0f};
        resource.bounds.maximum={3.3f,1.0f,1.0f};
        const auto material=scene->reserve_material_id();
        auto &mat=scene->material_store().create(material);
        auto &material_state = mat.edit_state();
        material_state.base_color = argc == 3 ? std::array<float,4>{0.24f,0.29f,0.35f,1.0f}
                                               : std::array<float,4>{0.74f,0.77f,0.81f,1.0f};
        // Match the editor's default broad plastic highlight.
        material_state.roughness = 0.65f;
        const auto node=scene->reserve_node_id();
        nkscene::Transaction create(scene); create.add_create(node);
        nkscene::ChangeSet changes;
        if (scene->commit(create,changes)!=NKS_OK) break;
        create.close();
        nkscene::Transaction configure(scene);
        configure.add_geometry(node,geometry); configure.add_material(node,material);
        if (scene->commit(configure,changes)!=NKS_OK) break;
        configure.close();
        nkscene::NativeKitGpuExecutor executor(renderer);
        status=0;
        for (int frame=0;frame<8;++frame) {
            const float yaw=(-50.0f+frame*45.0f)*pi/180.0f;
            const float pitch=32*pi/180.0f;
            nkscene::SceneView view; view.camera.enabled=true;
            view.camera.view_projection=view_projection(yaw,pitch,
                float(options.width)/options.height);
            configure_studio(view,yaw,pitch);
            auto plan=nkscene::compile(scene->snapshot(),view);
            std::vector<std::uint8_t> pixels;
            if (executor.capture_rgba8(plan,scene->snapshot(),options.width,options.height,
                    {0.88f,0.9f,0.92f,1.0f},pixels)!=NKGPU_OK) { status=1; break; }
            const auto path=std::string(argv[1])+"/orbit-"+std::to_string(frame)+".ppm";
            std::ofstream file(path,std::ios::binary);
            file << "P6\n" << options.width << ' ' << options.height << "\n255\n";
            for (std::size_t index=0;index<pixels.size();index+=4)
                file.write(reinterpret_cast<const char *>(pixels.data()+index),3);
            if (!file) { status=1; break; }
        }
    } while (false);
    if (renderer.id) nkgpu_renderer_destroy(renderer);
    if (surface) nkgpu_surface_destroy(surface);
    if (window) nk_window_destroy(window);
    nk_shutdown();
    return status;
}

#include "nativekit.h"
#include "nativekit_scene_render.hpp"
#include "nativekit_window.h"
#include "scene_internal.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

// Constant normal texel, directional light and orthographic view isolate the
// cotangent frame from material, filtering, AO and view-dependent highlights.
int main() {
    nk_init_options init{}; init.struct_size=sizeof(init);init.api_version=NK_API_VERSION;
    assert(nk_init(&init)==NK_OK);
    nk_window_options options{};options.struct_size=sizeof(options);
    options.width=128;options.height=128;options.title="SceneKit normal-map scale regression";
    nk_window window{};nk_surface surface{};nkgpu_renderer renderer{};
    assert(nk_window_create(&options,&window)==NK_OK);
    assert(nkgpu_surface_create(window,128,128,&surface)==NKGPU_OK);
    bool ready=false;
    for(int attempt=0;attempt<500 && !ready;++attempt) {
        nk_event event{};event.struct_size=sizeof(event);assert(nk_poll_event(&event)==NK_OK);
        ready=event.kind==NK_EVENT_SURFACE_READY && event.source==surface;
        nk_event_release(&event);std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(ready && nkgpu_renderer_create(surface,&renderer)==NKGPU_OK);
    {
        auto scene=std::make_shared<nkscene::Scene>();
        const auto geometry=scene->reserve_geometry_id();auto &resource=scene->geometry_store().create(geometry);
        resource.edit_payload().vertices={{{{-.5f,-.5f,0}}},{{{.5f,-.5f,0}}},{{{.5f,.5f,0}}},{{{-.5f,.5f,0}}}};
        resource.edit_payload().indices={0,1,2,0,2,3};
        const std::array<float,12> normals={0,0,1,0,0,1,0,0,1,0,0,1};
        const std::array<float,8> uvs={0,0,1,0,1,1,0,1};
        nkscene::GeometryVertexStream normal,uv;
        normal.semantic=nkscene::VertexSemantic::Normal;normal.format=nkscene::VertexFormat::Float32x3;
        normal.stride=12;normal.count=4;normal.data.resize(sizeof(normals));std::memcpy(normal.data.data(),normals.data(),sizeof(normals));
        uv.semantic=nkscene::VertexSemantic::Texcoord0;uv.format=nkscene::VertexFormat::Float32x2;
        uv.stride=8;uv.count=4;uv.data.resize(sizeof(uvs));std::memcpy(uv.data.data(),uvs.data(),sizeof(uvs));
        resource.edit_payload().streams={normal,uv};resource.bounds.valid=true;
        resource.bounds.minimum={-.5f,-.5f,0};resource.bounds.maximum={.5f,.5f,0};
        const auto image=scene->reserve_image_id();auto &pixels=scene->image_store().create(image);
        pixels.width=pixels.height=1;pixels.format=nkscene::ImageFormat::RGBA8;
        pixels.data={std::byte{128},std::byte{218},std::byte{218},std::byte{255}};
        const auto texture=scene->reserve_texture_id();scene->texture_store().create(texture).image=image;
        const auto material=scene->reserve_material_id();auto &mat=scene->material_store().create(material).edit_state();
        mat.base_color={.5f,.5f,.5f,1};mat.roughness=1;mat.normal_texture=texture;
        const auto node=scene->reserve_node_id();nkscene::ChangeSet changes;
        nkscene::Transaction create(scene);create.add_create(node);assert(scene->commit(create,changes)==NKS_OK);create.close();
        nkscene::Transaction configure(scene);configure.add_geometry(node,geometry);configure.add_material(node,material);
        assert(scene->commit(configure,changes)==NKS_OK);configure.close();
        nkscene::SceneView view;view.camera.enabled=true;view.camera.has_view_pose=true;
        view.camera.orthographic=true;view.camera.view_direction={0,0,1};
        view.studio_lighting.enabled=true;view.studio_lighting.directions[0]={0,0,1,.7f};
        view.studio_lighting.ambient_sky={0,0,0,0};view.studio_lighting.ambient_ground={0,0,0,0};
        nkscene::NativeKitGpuExecutor executor(renderer);
        auto capture=[&](float world_scale,float zoom) {
            nkscene::LocalTransform transform;
            transform.matrix[0]=transform.matrix[5]=transform.matrix[10]=world_scale;
            nkscene::Transaction move(scene);move.add_transform(node,transform);
            assert(scene->commit(move,changes)==NKS_OK);move.close();
            view.camera.view_projection={zoom/world_scale,0,0,0,0,zoom/world_scale,0,0,0,0,-1/world_scale,0,0,0,0,1};
            auto plan=nkscene::compile(scene->snapshot(),view);std::vector<std::uint8_t> result;
            assert(executor.capture_rgba8(plan,scene->snapshot(),128,128,{0,0,0,0},result)==NKGPU_OK);
            return result[(64*128+64)*4];
        };
        const int reference=capture(1,1);assert(reference>20);
        int largest_difference=0;
        for(float scale:{1.f,.1f,.01f,.001f})for(float zoom:{.5f,1.f,1.5f}) {
            int brightness=capture(scale,zoom),difference=std::abs(brightness-reference);
            largest_difference=std::max(largest_difference,difference);
            std::printf("normal-map scale=%g zoom=%g brightness=%d reference=%d\n",scale,zoom,brightness,reference);
        }
        // Perspective depth changes the pixel footprint along a long screw too.
        // Keep V fixed deliberately so this measures the normal frame rather
        // than the legitimate change in specular reflection with viewing angle.
        for(float depth:{.1f,.5f,1.f,4.f,8.f}) {
            nkscene::LocalTransform transform;transform.matrix[14]=-depth;
            nkscene::Transaction move(scene);move.add_transform(node,transform);
            assert(scene->commit(move,changes)==NKS_OK);move.close();
            constexpr float near=.01f,far=100.f;
            view.camera.view_projection={1,0,0,0,0,1,0,0,0,0,-(far+near)/(far-near),-1,0,0,-2*far*near/(far-near),0};
            auto plan=nkscene::compile(scene->snapshot(),view);std::vector<std::uint8_t> result;
            assert(executor.capture_rgba8(plan,scene->snapshot(),128,128,{0,0,0,0},result)==NKGPU_OK);
            int brightness=result[(64*128+64)*4];
            largest_difference=std::max(largest_difference,std::abs(brightness-reference));
            std::printf("normal-map perspective depth=%g brightness=%d reference=%d\n",depth,brightness,reference);
        }
        std::fflush(stdout);assert(largest_difference<=2);
        // UV-degenerate triangles must retain a finite, lit geometric normal.
        auto &degenerate_uv=resource.edit_payload().streams[1].data;
        std::fill(degenerate_uv.begin(),degenerate_uv.end(),std::byte{0});
        scene->publish();assert(capture(.001f,1)>20);
    }
    nkgpu_renderer_destroy(renderer);nk_surface_destroy(surface);nk_window_destroy(window);nk_shutdown();
}

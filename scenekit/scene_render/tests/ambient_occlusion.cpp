#include "nativekit.h"
#include "nativekit_scene_render.hpp"
#include "nativekit_window.h"
#include "scene_internal.hpp"
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>
#include <vector>

// A flat floor and a raised ledge: only the floor near the ledge should lose indirect light.
int main(int argc, char **argv) {
    nk_init_options init{}; init.struct_size=sizeof(init); init.api_version=NK_API_VERSION;
    assert(nk_init(&init)==NK_OK);
    nk_window_options options{}; options.struct_size=sizeof(options);
    options.width=128;options.height=128;options.title="SceneKit ambient occlusion regression";
    nk_window window{};nk_surface surface{};nkgpu_renderer renderer{};
    assert(nk_window_create(&options,&window)==NK_OK);
    assert(nkgpu_surface_create(window,128,128,&surface)==NKGPU_OK);
    bool ready=false;
    for(int attempt=0;attempt<500 && !ready;++attempt) {
        nk_event event{};event.struct_size=sizeof(event);
        assert(nk_poll_event(&event)==NK_OK);
        ready=event.kind==NK_EVENT_SURFACE_READY && event.source==surface;
        nk_event_release(&event);std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(ready && nkgpu_renderer_create(surface,&renderer)==NKGPU_OK);
    {
        auto scene=std::make_shared<nkscene::Scene>();
        const auto geometry=scene->reserve_geometry_id();
        auto &resource=scene->geometry_store().create(geometry);
        using Vec=std::array<float,3>;
        std::vector<Vec> normals;
        auto quad=[&](float x0,float x1,float y0,float y1,float z) {
            for(auto p:std::array<Vec,6>{{{x0,y0,z},{x1,y0,z},{x1,y1,z},{x0,y0,z},{x1,y1,z},{x0,y1,z}}}) {
                resource.edit_payload().vertices.push_back({p});normals.push_back({0,0,1});
            }
        };
        quad(-0.8f,0.8f,-0.8f,0.8f,0);
        quad(0.1f,0.3f,-0.6f,0.6f,0.2f);
        nkscene::GeometryVertexStream stream;
        stream.semantic=nkscene::VertexSemantic::Normal;stream.format=nkscene::VertexFormat::Float32x3;
        stream.stride=sizeof(Vec);stream.count=normals.size();stream.data.resize(normals.size()*sizeof(Vec));
        std::memcpy(stream.data.data(),normals.data(),stream.data.size());
        resource.edit_payload().streams.push_back(stream);
        resource.bounds.valid=true;resource.bounds.minimum={-.8f,-.8f,0};resource.bounds.maximum={.8f,.8f,.2f};
        const auto material=scene->reserve_material_id();
        auto &mat=scene->material_store().create(material).edit_state();
        mat.base_color={.6f,.6f,.6f,1};mat.roughness=.65f;
        const auto node=scene->reserve_node_id();
        nkscene::Transaction create(scene);create.add_create(node);nkscene::ChangeSet changes;
        assert(scene->commit(create,changes)==NKS_OK);create.close();
        nkscene::Transaction configure(scene);configure.add_geometry(node,geometry);configure.add_material(node,material);
        assert(scene->commit(configure,changes)==NKS_OK);configure.close();
        nkscene::SceneView view;view.camera.enabled=true;
        view.camera.view_projection={1,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,1};
        view.camera.has_view_pose=true;view.camera.orthographic=true;view.camera.view_direction={0,0,1};
        view.studio_lighting.enabled=true;
        view.studio_lighting.ambient_sky={.5f,.5f,.5f,0};view.studio_lighting.ambient_ground={.5f,.5f,.5f,0};
        nkscene::NativeKitGpuExecutor executor(renderer);
        for(auto samples:{1u,4u}) {
            executor.set_sample_count(samples);
            std::vector<std::uint8_t> plain,ao,restored;
            auto capture=[&](std::vector<std::uint8_t> &pixels) {
                auto plan=nkscene::compile(scene->snapshot(),view);
                assert(executor.capture_rgba8(plan,scene->snapshot(),128,128,{0,0,0,0},pixels)==NKGPU_OK);
            };
            view.studio_lighting.surface_detail={};capture(plain);
            view.studio_lighting.surface_detail={.4f,1.f,.08f,0};capture(ao);
            double darkening=0;int count=0;
            for(int y=45;y<83;++y)for(int x=65;x<70;++x) {
                const auto i=(y*128+x)*4;darkening+=int(plain[i])-int(ao[i]);++count;
                assert(plain[i+3]==ao[i+3]);
            }
            assert(darkening/count>2.0); // ledge-to-floor occlusion
            for(int y=45;y<83;++y)for(int x=20;x<30;++x) {
                const auto i=(y*128+x)*4;assert(std::abs(int(plain[i])-int(ao[i]))<=1);
            }
            assert(ao[3]==0); // no background halo or opacity change
            view.studio_lighting.surface_detail={};capture(restored);
            assert(restored==plain); // disabling AO leaves the original renderer exactly intact
            if(argc>1) {
                std::filesystem::create_directories(argv[1]);
                for(auto entry:{std::pair{"plain",&plain},std::pair{"ao",&ao}}) {
                    std::ofstream file(std::string(argv[1])+"/"+entry.first+"-"+std::to_string(samples)+".ppm",std::ios::binary);
                    file<<"P6\n128 128\n255\n";
                    for(std::size_t i=0;i<entry.second->size();i+=4)file.write(reinterpret_cast<const char*>(entry.second->data()+i),3);
                }
            }
        }
        // An isolated sloping plane cannot occlude itself, even when zooming
        // forces a tiny near plane. Device-depth reconstruction used to stripe it.
        resource.edit_payload().vertices.clear(); normals.clear();
        quad(-4,4,-4,4,0);
        const auto normal_length=std::sqrt(1.1f);
        for(auto &vertex:resource.edit_payload().vertices)
            vertex.position[2]=0.1f*vertex.position[0]+0.3f*vertex.position[1];
        for(auto &normal:normals)normal={-0.1f/normal_length,-0.3f/normal_length,1.f/normal_length};
        auto &normal_stream=resource.edit_payload().streams[0];
        normal_stream.count=normals.size();normal_stream.data.resize(normals.size()*sizeof(Vec));
        std::memcpy(normal_stream.data.data(),normals.data(),normal_stream.data.size());
        resource.bounds.minimum={-4,-4,-1.6f};resource.bounds.maximum={4,4,1.6f};
        scene->publish();
        view.camera.orthographic=false;
        for(auto size:{96u,128u})for(auto samples:{1u,4u})for(auto near:{0.000001f,0.00001f,0.0001f,0.01f})for(auto distance:{0.5f,1.f,3.f}) {
            executor.set_sample_count(samples);
            const float far=100, c=(far+near)/(far-near), d=-2*far*near/(far-near);
            view.camera.position={0,0,distance};
            view.camera.view_projection={2,0,0,0,0,2,0,0,0,0,-c,-1,0,0,c*distance+d,distance};
            std::vector<std::uint8_t> plain,ao;
            auto capture=[&](std::vector<std::uint8_t> &pixels) {
                auto plan=nkscene::compile(scene->snapshot(),view);
                assert(executor.capture_rgba8(plan,scene->snapshot(),size,size,{0,0,0,0},pixels)==NKGPU_OK);
            };
            view.studio_lighting.surface_detail={};capture(plain);
            view.studio_lighting.surface_detail={.025f,.8f,.08f,0};capture(ao);
            int maximum=0;
            for(unsigned y=size/4;y<3*size/4;++y)for(unsigned x=size/4;x<3*size/4;++x) {
                const auto i=(y*size+x)*4;
                maximum=std::max(maximum,std::abs(int(plain[i])-int(ao[i])));
                assert(plain[i+3]==ao[i+3]);
            }
            if(maximum>1)std::fprintf(stderr,"Flat-plane AO stripes: size=%u samples=%u near=%g distance=%g maximum=%d\n",size,samples,near,distance,maximum);
            assert(maximum<=1);
        }

        // AO and picking share geometry but not their output representation.
        auto pick_plan=nkscene::compile(scene->snapshot(),view);
        nkscene::PickResult picked;
        assert(executor.pick_pixel(pick_plan,scene->snapshot(),128,128,64,64,&picked)==NKGPU_OK);
        assert(picked.node==node);
        assert(std::abs(picked.worldPosition.z-0.1f*picked.worldPosition.x-0.3f*picked.worldPosition.y)<0.001f);

    }
    nkgpu_renderer_destroy(renderer);nk_surface_destroy(surface);nk_window_destroy(window);nk_shutdown();
}

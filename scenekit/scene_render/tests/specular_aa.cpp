#include "nativekit.h"
#include "nativekit_scene_render.hpp"
#include "nativekit_window.h"
#include "scene_internal.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>
#include <vector>
using namespace nkscene;
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

std::array<float,16> view_projection(float yaw, float pitch, float aspect, float distance) {
    const Vec eye{distance*std::cos(pitch)*std::cos(yaw),
                  distance*std::cos(pitch)*std::sin(yaw), distance*std::sin(pitch)};
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

void configure_studio(nkscene::SceneView &view, float yaw, float pitch, bool panels) {
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
    if (panels) {
        // haxeon.editor.ViewportLook's Studio preset: its rig, room, panels and exposure.
        const std::array<std::array<float,4>,3> studio{{
            {0.60f,-0.60f,-0.53f,0.95f},{-0.65f,-0.15f,-0.74f,0.16f},{-0.45f,0.75f,0.48f,0.30f}}};
        for (int i=0;i<3;++i) {
            const auto &l=studio[i];
            const auto world=add(add(scale(right,-l[0]),scale(up,-l[1])),scale(forward,l[2]));
            view.studio_lighting.directions[i]={world[0],world[1],world[2],l[3]};
        }
        view.studio_lighting.ambient_sky={0.50f,0.53f,0.58f,0};
        view.studio_lighting.ambient_ground={0.22f,0.23f,0.26f,0};
        view.studio_lighting.light_radii={0.45f,0.7f,0.35f};
        view.studio_lighting.environment=true;
        view.studio_lighting.reflection=1.0f;
        view.studio_lighting.exposure=1.0f;
        view.studio_lighting.surface_detail={0.0f,0.0f,0.0f,0.40f};
    }
}


struct Mesh {
    std::vector<GeometryVertex> vertices;
    std::vector<Vec> normals;
    void tri(Vec a,Vec b,Vec c,Vec na,Vec nb,Vec nc){for(auto v:{a,b,c})vertices.push_back({v});for(auto n:{na,nb,nc})normals.push_back(n);}
    void quad(Vec a,Vec b,Vec c,Vec d,Vec na,Vec nb,Vec nc,Vec nd){tri(a,b,c,na,nb,nc);tri(a,c,d,na,nc,nd);}
};
// Rounded helical flanks on a nut bore: real geometric normals, no normal map.
Mesh nut() {
    Mesh m;constexpr int rings=240,sides=192;constexpr float pitch=.045f,depth=.012f;
    auto point=[](float a,float z){float phase=2*pi*z/pitch-a,r=.36f+depth*std::cos(phase);return Vec{r*std::cos(a),r*std::sin(a),z};};
    auto normal=[](float a,float z){float phase=2*pi*z/pitch-a,r=.36f+depth*std::cos(phase),t=depth*std::sin(phase)/r;
        return unit(Vec{-std::cos(a)-t*std::sin(a),-std::sin(a)+t*std::cos(a),-depth*2*pi/pitch*std::sin(phase)});};
    auto outer=[](float a,float z){float r=.63f/std::cos(std::remainder(a,pi/3));return Vec{r*std::cos(a),r*std::sin(a),z};};
    for(int i=0;i<sides;++i){float a=2*pi*i/sides,b=2*pi*(i+1)/sides;
        for(int j=0;j<rings;++j){float z=-.24f+.48f*j/rings,w=-.24f+.48f*(j+1)/rings;
            m.quad(point(b,z),point(a,z),point(a,w),point(b,w),normal(b,z),normal(a,z),normal(a,w),normal(b,w));}
        m.quad(point(a,.24f),outer(a,.24f),outer(b,.24f),point(b,.24f),{0,0,1},{0,0,1},{0,0,1},{0,0,1});
        float normal_angle=std::round((a+b)*.5f/(pi/3))*(pi/3);
        Vec n{std::cos(normal_angle),std::sin(normal_angle),0};
        m.quad(outer(a,-.24f),outer(b,-.24f),outer(b,.24f),outer(a,.24f),n,n,n,n);
    }return m;
}
Mesh fillet() {
    Mesh m;constexpr int steps=512;
    for(int i=0;i<steps;++i){float a=pi*.5f*i/steps,b=pi*.5f*(i+1)/steps;
        Vec na{std::cos(a),0,std::sin(a)},nb{std::cos(b),0,std::sin(b)};
        auto p=[](Vec n,float y){return Vec{.09f*n[0],y,.09f*n[2]};};
        m.quad(p(na,-.6f),p(na,.6f),p(nb,.6f),p(nb,-.6f),na,na,nb,nb);
    }return m;
}
Mesh flat() {Mesh m;m.quad({-.7f,-.7f,0},{.7f,-.7f,0},{.7f,.7f,0},{-.7f,.7f,0},{0,0,1},{0,0,1},{0,0,1},{0,0,1});return m;}
void save(const std::filesystem::path &path,const std::vector<uint8_t>&pixels,int side){std::ofstream f(path,std::ios::binary);f<<"P6\n"<<side<<" "<<side<<"\n255\n";for(size_t i=0;i<pixels.size();i+=4)f.write((const char*)pixels.data()+i,3);}
void check_mesh(nkgpu_renderer renderer,Mesh mesh,const char *name,const std::filesystem::path &output) {
    auto scene=std::make_shared<Scene>();auto id=scene->reserve_geometry_id();auto &g=scene->geometry_store().create(id);g.edit_payload().vertices=mesh.vertices;
    GeometryVertexStream n;n.semantic=VertexSemantic::Normal;n.format=VertexFormat::Float32x3;n.stride=12;n.count=mesh.normals.size();n.data.resize(n.count*12);std::memcpy(n.data.data(),mesh.normals.data(),n.data.size());g.edit_payload().streams={n};g.bounds={{{-1,-1,-1}},{{1,1,1}},true};
    auto material=scene->reserve_material_id();auto &mat=scene->material_store().create(material).edit_state();mat.base_color={.74f,.77f,.81f,1};mat.metallic=.95f;mat.roughness=.06f;
    auto node=scene->reserve_node_id();ChangeSet changes;Transaction create(scene);create.add_create(node);assert(scene->commit(create,changes)==NKS_OK);create.close();Transaction config(scene);config.add_geometry(node,id);config.add_material(node,material);assert(scene->commit(config,changes)==NKS_OK);config.close();scene->publish();
    NativeKitGpuExecutor gpu(renderer);constexpr int side=256;
    for(unsigned samples:{1u,4u})for(float distance:{1.5f,3.f,6.f}) {
        gpu.set_sample_count(samples);std::vector<uint8_t> previous;double maximum_mean=0;int maximum_jump=0,large=0,valid=0;
        for(int frame=0;frame<7;++frame){float yaw=-.8f+frame*.0007f,pitch=.9f;
            SceneView view;view.camera.enabled=true;view.camera.has_view_pose=true;
            view.camera.position={distance*std::cos(pitch)*std::cos(yaw),distance*std::cos(pitch)*std::sin(yaw),distance*std::sin(pitch)};
            view.camera.view_direction=unit(view.camera.position);view.camera.view_projection=view_projection(yaw,pitch,1,distance);configure_studio(view,yaw,pitch,true);
            view.studio_lighting.light_radii={.12f,.2f,.12f};
            view.studio_lighting.surface_detail={0,0,0,0}; // Test filtering, without the studio roughness floor.
            auto plan=compile(scene->snapshot(),view);std::vector<uint8_t> pixels;assert(gpu.capture_rgba8(plan,scene->snapshot(),side,side,{0,0,0,0},pixels)==NKGPU_OK);
            if(!output.empty()&&samples==4&&frame==0) {
                save(output/(std::string(name)+"-"+std::to_string(distance)+".ppm"),pixels,side);
                if(std::string(name)=="threaded-nut"&&distance==1.5f) {
                    auto closeup=view;closeup.camera.view_projection=view_projection(yaw,pitch,1,2.1f);
                    closeup.camera.position=scale(unit(view.camera.position),2.1f);
                    // Review capture uses the editor's steel / Studio appearance;
                    // the temporal tests above deliberately use a sharper stress material.
                    configure_studio(closeup,yaw,pitch,true);
                    scene->material_store().find(material)->edit_state().roughness=.34f;scene->publish();
                    auto closeup_plan=compile(scene->snapshot(),closeup);std::vector<uint8_t> detail;
                    assert(gpu.capture_rgba8(closeup_plan,scene->snapshot(),768,768,{.88f,.9f,.92f,1},detail)==NKGPU_OK);
                    save(output/"threaded-nut-closeup.ppm",detail,768);
                    scene->material_store().find(material)->edit_state().roughness=.06f;scene->publish();
                }
            }
            if(!previous.empty()) {double sum=0;int count=0;
                for(int y=2;y<side-2;++y)for(int x=2;x<side-2;++x){bool interior=true;for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx){int i=((y+dy)*side+x+dx)*4;if(pixels[i+3]!=255||previous[i+3]!=255)interior=false;}if(!interior)continue;
                    for(int c=0;c<3;++c){int d=std::abs(int(pixels[(y*side+x)*4+c])-int(previous[(y*side+x)*4+c]));sum+=d;++count;maximum_jump=std::max(maximum_jump,d);large+=d>20;++valid;}}
                if(count)maximum_mean=std::max(maximum_mean,sum/count);
            }previous=std::move(pixels);
        }
        std::printf("SpecularAA %s samples=%u distance=%g mean=%g max=%d large=%d/%d\n",name,samples,distance,maximum_mean,maximum_jump,large,valid);std::fflush(stdout);
        assert(valid>0);
        if(std::string(name)=="flat") {assert(maximum_mean<.001);assert(maximum_jump<=1);}
        else if(std::string(name)=="fillet") {assert(maximum_mean<.8);if(distance==6)assert(maximum_jump<=3);}
        else {assert(maximum_mean<.16);assert(double(large)/valid<.0011);}
        // These bounds exclude silhouette motion above, but retain changing
        // thread crests. A few visibility discontinuities are allowed; a run
        // of flashing highlight pixels is not. Far fillets bound individual jumps.
    }
}
void normal_map_variance(nkgpu_renderer renderer) {
    auto scene=std::make_shared<Scene>();auto geometry=scene->reserve_geometry_id();auto &g=scene->geometry_store().create(geometry);
    auto mesh=flat();g.edit_payload().vertices=mesh.vertices;g.bounds={{{-.7f,-.7f,0}},{{.7f,.7f,0}},true};
    GeometryVertexStream n;n.semantic=VertexSemantic::Normal;n.format=VertexFormat::Float32x3;n.stride=12;n.count=mesh.normals.size();n.data.resize(n.count*12);std::memcpy(n.data.data(),mesh.normals.data(),n.data.size());
    GeometryVertexStream uv;uv.semantic=VertexSemantic::Texcoord0;uv.format=VertexFormat::Float32x2;uv.stride=8;uv.count=n.count;uv.data.resize(uv.count*8);
    // Nondegenerate UVs are required for the normal map's cotangent frame.
    float coords[12]={0,0,1,0,1,1,0,0,1,1,0,1};std::memcpy(uv.data.data(),coords,sizeof(coords));g.edit_payload().streams={n,uv};
    auto image=scene->reserve_image_id();auto &im=scene->image_store().create(image);im.width=im.height=1;im.format=ImageFormat::RGBA8;im.data={std::byte{128},std::byte{128},std::byte{255},std::byte{255}};
    auto texture=scene->reserve_texture_id();scene->texture_store().create(texture).image=image;
    auto material=scene->reserve_material_id();auto &mat=scene->material_store().create(material).edit_state();mat.base_color={.8f,.8f,.8f,1};mat.metallic=1;mat.roughness=.06f;mat.normal_texture=texture;
    auto node=scene->reserve_node_id();ChangeSet changes;Transaction create(scene);create.add_create(node);assert(scene->commit(create,changes)==NKS_OK);create.close();Transaction config(scene);config.add_geometry(node,geometry);config.add_material(node,material);assert(scene->commit(config,changes)==NKS_OK);config.close();
    SceneView view;view.camera.enabled=true;view.camera.has_view_pose=true;view.camera.orthographic=true;view.camera.view_direction={0,0,1};view.camera.view_projection={1,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,1};
    view.studio_lighting.enabled=true;view.studio_lighting.directions={{{0,0,1,1},{0,0,1,0},{0,0,1,0}}};view.studio_lighting.light_radii={.1f,0,0};view.studio_lighting.environment=true;view.studio_lighting.exposure=.01f;view.studio_lighting.reflection=1;view.studio_lighting.surface_detail={0,0,0,0};view.studio_lighting.ambient_sky={0,0,0,0};view.studio_lighting.ambient_ground={0,0,0,0};
    NativeKitGpuExecutor gpu(renderer);auto capture=[&](){scene->publish();auto plan=compile(scene->snapshot(),view);std::vector<uint8_t> pixels;assert(gpu.capture_rgba8(plan,scene->snapshot(),64,64,{0,0,0,0},pixels)==NKGPU_OK);return pixels[(32*64+32)*4];};
    int unit=capture();
    auto blurred_image=scene->reserve_image_id();auto &blurred=scene->image_store().create(blurred_image);blurred.width=blurred.height=1;blurred.format=ImageFormat::RGBA8;blurred.data={std::byte{128},std::byte{128},std::byte{191},std::byte{255}};
    auto blurred_texture=scene->reserve_texture_id();scene->texture_store().create(blurred_texture).image=blurred_image;
    scene->material_store().find(material)->edit_state().normal_texture=blurred_texture;
    int unresolved=capture();std::printf("NormalMapVariance unit=%d unresolved=%d\n",unit,unresolved);std::fflush(stdout);assert(unit>180&&unresolved<100);
}

}
int main(int argc,char **argv) {
    std::filesystem::path output=argc>1?argv[1]:"";if(!output.empty())std::filesystem::create_directories(output);
    nk_init_options init{};init.struct_size=sizeof(init);init.api_version=NK_API_VERSION;assert(nk_init(&init)==NK_OK);
    nk_window_options options{};options.struct_size=sizeof(options);options.width=options.height=256;options.title="Specular antialiasing";
    nk_window window{};nk_surface surface{};nkgpu_renderer renderer{};assert(nk_window_create(&options,&window)==NK_OK);assert(nkgpu_surface_create(window,256,256,&surface)==NKGPU_OK);
    bool ready=false;for(int i=0;i<500&&!ready;++i){nk_event e{};e.struct_size=sizeof(e);assert(nk_poll_event(&e)==NK_OK);ready=e.kind==NK_EVENT_SURFACE_READY&&e.source==surface;nk_event_release(&e);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    assert(ready&&nkgpu_renderer_create(surface,&renderer)==NKGPU_OK);
    normal_map_variance(renderer);check_mesh(renderer,nut(),"threaded-nut",output);check_mesh(renderer,fillet(),"fillet",output);check_mesh(renderer,flat(),"flat",output);
    assert(nkgpu_renderer_destroy(renderer)==NKGPU_OK);assert(nkgpu_surface_destroy(surface)==NKGPU_OK);assert(nk_window_destroy(window)==NK_OK);nk_shutdown();
}

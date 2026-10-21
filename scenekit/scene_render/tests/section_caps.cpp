#include "nativekit.h"
#include "nativekit_scene_render.hpp"
#include "nativekit_window.h"
#include "scene_internal.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <thread>
#include <chrono>
using namespace nkscene;
using P=std::array<float,3>;

struct Fixture {
    std::shared_ptr<Scene> scene=std::make_shared<Scene>();
    GeometryId geometry;MaterialId material;NodeId node;SceneView view;
    Fixture() {
        geometry=scene->reserve_geometry_id();material=scene->reserve_material_id();node=scene->reserve_node_id();
        auto &mesh=scene->geometry_store().create(geometry);mesh.edit_payload().closed=true;
        MaterialState m;m.base_color={.8f,.6f,.3f,1};scene->material_store().create(material).edit_state()=m;
        Transaction create(scene);create.add_create(node);ChangeSet changes;assert(scene->commit(create,changes)==NKS_OK);create.close();
        Transaction configure(scene);configure.add_geometry(node,geometry);configure.add_material(node,material);assert(scene->commit(configure,changes)==NKS_OK);configure.close();scene->publish();
        view.camera.enabled=true;view.camera.view_projection={1,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,1};
    }
    void quad(P a,P b,P c,P d) {
        auto &payload=scene->geometry_store().find(geometry)->edit_payload();
        for(auto v:{a,b,c,a,c,d})payload.vertices.push_back({v});
    }
    void box(float r,float z0,float z1,bool reverse=false) {
        P v[8]={{-r,-r,z0},{r,-r,z0},{r,r,z0},{-r,r,z0},{-r,-r,z1},{r,-r,z1},{r,r,z1},{-r,r,z1}};
        int faces[6][4]={{0,3,2,1},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
        for(auto &f:faces)if(reverse)quad(v[f[3]],v[f[2]],v[f[1]],v[f[0]]);else quad(v[f[0]],v[f[1]],v[f[2]],v[f[3]]);
        scene->geometry_store().find(geometry)->bounds={{{-r,-r,z0}},{{r,r,z1}},true};
    }
    RenderPlan plan(){scene->publish();return compile(scene->snapshot(),view);}
};

void one_plane_and_picking(nkgpu_renderer renderer) {
    Fixture f;f.box(.8f,-.8f,.8f);f.view.clip_planes.push_back({{{0,0,-1}},0,true});
    NativeKitGpuExecutor gpu(renderer);auto plan=f.plan();std::vector<std::uint8_t> pixels;
    assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},pixels)==NKGPU_OK);
    assert(pixels[(48*96+48)*4+3]==255);
    PickResult pick;assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&pick)==NKGPU_OK);
    assert(pick.node==f.node);assert(pick.subelement.value==cap_subelement_tag);assert(std::abs(pick.worldPosition.z)<1e-4f);
    SceneSpatialIndex spatial(f.scene->snapshot(),&f.view);auto cpu=spatial.pick_ray({{0,0,2},{0,0,-1}});
    assert(cpu.node==f.node&&cpu.subelement.value==cap_subelement_tag&&std::abs(cpu.worldPosition.z)<1e-5f);
    // Cap depth must occlude a later solid whose near surface is behind the cut.
    for(unsigned samples:{1u,4u}) {
        gpu.set_sample_count(samples);assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},pixels)==NKGPU_OK);
        assert(pixels[(48*96+48)*4+3]==255);
        // No cap or stencil leakage beyond the geometric silhouette, including MSAA samples.
        for(int y=0;y<96;++y)for(int x=0;x<96;++x)if(x<8||x>87||y<8||y>87)assert(pixels[(y*96+x)*4+3]==0);
    }
    // Mirroring reverses winding, not solid membership.
    LocalTransform mirror;mirror.matrix[0]=-1;Transaction edit(f.scene);edit.add_transform(f.node,mirror);ChangeSet changes;assert(f.scene->commit(edit,changes)==NKS_OK);edit.close();plan=f.plan();
    assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},pixels)==NKGPU_OK);
    assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&pick)==NKGPU_OK&&pick.subelement.value==cap_subelement_tag);
    std::puts("section caps: OnePlane, CapPicking, CapDepth, MSAA and MirroredInstance passed");
}

void two_plane_corner_both_surface_hits_clipped(nkgpu_renderer renderer) {
    Fixture f;f.box(.8f,-.8f,.8f);
    // Central ray: x+2z=.4, direction (-1,0,.5). Both original surface hits are clipped.
    float r=1/std::sqrt(5.f);f.view.camera.view_projection={r,0,-2*r*.5f,0,0,1,0,0,2*r,0,r*.5f,0,-.4f*r,0,0,1};
    f.view.clip_planes={{{{1,0,0}},0,true},{{{0,0,1}},0,true}};
    NativeKitGpuExecutor gpu(renderer);auto plan=f.plan();std::vector<std::uint8_t> pixels;
    assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},pixels)==NKGPU_OK);
    assert(pixels[(48*96+48)*4+3]==255);
    PickResult hit;assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&hit)==NKGPU_OK);
    assert(hit.node==f.node&&(hit.subelement.value&cap_subelement_tag));
    assert(hit.cap_plane()==1&&std::abs(hit.worldPosition.z)<1e-4f);
    SceneSpatialIndex spatial(f.scene->snapshot(),&f.view);auto cpu=spatial.pick_ray({{3,0,-1.3f},{-1,0,.5f}});
    assert(cpu.node==f.node&&cpu.cap_plane()==1);
    std::puts("section caps: TwoPlaneCornerBothSurfaceHitsClipped passed");
}

void cavity_in_front_of_back_surface(nkgpu_renderer renderer) {
    Fixture f;f.box(.8f,-.8f,.8f);f.box(.3f,-.6f,.6f,true);
    // Restore encompassing bounds after adding the inward cavity boundary.
    f.scene->geometry_store().find(f.geometry)->bounds={{{-.8f,-.8f,-.8f}},{{.8f,.8f,.8f}},true};
    f.view.clip_planes={{{{0,0,-1}},.2f,true}};
    NativeKitGpuExecutor gpu(renderer);auto plan=f.plan();PickResult hit;
    std::vector<std::uint8_t> pixels;assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},pixels)==NKGPU_OK);
    assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&hit)==NKGPU_OK);
    assert(hit.node==f.node&&!(hit.subelement.value&cap_subelement_tag)&&std::abs(hit.worldPosition.z+.6f)<1e-4f);
    assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,72,48,&hit)==NKGPU_OK);
    assert(hit.node==f.node&&hit.subelement.value==cap_subelement_tag&&std::abs(hit.worldPosition.z-.2f)<1e-4f);
    std::puts("section caps: CavityInFrontOfBackSurface passed");
}

void hollow_core(nkgpu_renderer renderer) {
    Fixture f;
    P outer[4]={{-.8f,-.8f,0},{.8f,-.8f,0},{.8f,.8f,0},{-.8f,.8f,0}};
    P inner[4]={{-.3f,-.3f,0},{.3f,-.3f,0},{.3f,.3f,0},{-.3f,.3f,0}};
    auto at=[](P p,float z){p[2]=z;return p;};
    for(int j=0;j<4;++j){int k=(j+1)%4;
        f.quad(at(outer[j],-.8f),at(outer[k],-.8f),at(outer[k],.8f),at(outer[j],.8f));
        f.quad(at(inner[k],-.8f),at(inner[j],-.8f),at(inner[j],.8f),at(inner[k],.8f));
        f.quad(at(outer[j],.8f),at(outer[k],.8f),at(inner[k],.8f),at(inner[j],.8f));
        f.quad(at(outer[k],-.8f),at(outer[j],-.8f),at(inner[j],-.8f),at(inner[k],-.8f));
    }
    f.scene->geometry_store().find(f.geometry)->bounds={{{-.8f,-.8f,-.8f}},{{.8f,.8f,.8f}},true};
    f.view.clip_planes={{{{0,0,-1}},.2f,true}};NativeKitGpuExecutor gpu(renderer);auto plan=f.plan();
    for(unsigned samples:{1u,4u}) {
        gpu.set_sample_count(samples);std::vector<std::uint8_t> pixels;
        assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},pixels)==NKGPU_OK);
        for(int y=38;y<58;++y)for(int x=38;x<58;++x)assert(pixels[(y*96+x)*4+3]==0);
        assert(pixels[(48*96+72)*4+3]==255);
    }
    PickResult hit;assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&hit)==NKGPU_OK&&!hit.node.valid());
    // AO position prepass must retain the same hole and cap depth.
    f.view.studio_lighting.enabled=true;f.view.studio_lighting.surface_detail={.1f,.5f,.05f,0};plan=f.plan();
    std::vector<std::uint8_t> ao;assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},ao)==NKGPU_OK);
    assert(ao[(48*96+48)*4+3]==0&&ao[(48*96+72)*4+3]==255);
    std::puts("section caps: HollowCore and AO passed");
}

void occluded_crossings_and_material_exclusions(nkgpu_renderer renderer) {
    Fixture f;f.box(.8f,-.8f,.8f);f.view.clip_planes={{{{0,0,-1}},0,true}};
    auto g=f.scene->reserve_geometry_id();auto &r=f.scene->geometry_store().create(g);
    r.edit_payload().vertices={{{-.8f,-.8f,-.3f}},{{.8f,-.8f,-.3f}},{{.8f,.8f,-.3f}},{{-.8f,-.8f,-.3f}},{{.8f,.8f,-.3f}},{{-.8f,.8f,-.3f}}};
    r.bounds={{{-.8f,-.8f,-.3f}},{{.8f,.8f,-.3f}},true};
    auto node=f.scene->reserve_node_id();Transaction create(f.scene);create.add_create(node);ChangeSet changes;assert(f.scene->commit(create,changes)==NKS_OK);create.close();
    Transaction configure(f.scene);configure.add_geometry(node,g);configure.add_material(node,f.material);assert(f.scene->commit(configure,changes)==NKS_OK);configure.close();
    NativeKitGpuExecutor gpu(renderer);auto plan=f.plan();PickResult hit;
    assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&hit)==NKGPU_OK&&hit.node==f.node&&hit.subelement.value==cap_subelement_tag);
    assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,85,48,&hit)==NKGPU_OK&&hit.subelement.value==cap_subelement_tag&&std::abs(hit.worldPosition.z)<1e-5f);
    // Feature edges behind a cap must be occluded by its real plane depth, including at MSAA samples.
    for(unsigned samples:{1u,4u}) {
        gpu.set_sample_count(samples);std::vector<std::uint8_t> plain,edged;
        r.edit_payload().stroke_segments.clear();plan=f.plan();
        assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},plain)==NKGPU_OK);
        r.edit_payload().stroke_segments.push_back({{-.7f,0,-.3f},{.7f,0,-.3f},0});plan=f.plan();
        assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},edged)==NKGPU_OK);
        assert(plain==edged);
    }
    // Both-sided and transparent materials show their original surfaces, never a cap.
    f.view.set_visibility_override(node,false);
    auto &material=f.scene->material_store().find(f.material)->edit_state();material.flags|=static_cast<unsigned>(MaterialFlags::DoubleSided);
    plan=f.plan();assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&hit)==NKGPU_OK&&!(hit.subelement.value&cap_subelement_tag));
    material.flags=static_cast<unsigned>(MaterialFlags::Opaque);material.alpha_mode=AlphaMode::Blend;material.opacity=.5f;
    f.scene->material_store().find(f.material)->edit_state()=material;plan=f.plan();
    assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&hit)==NKGPU_OK&&!(hit.subelement.value&cap_subelement_tag));
    std::puts("section caps: OccludedCrossings, CapBoundaryDepth, DoubleSided and TransparentExclusions passed");
}

void overlapping_instances_keep_independent_parity(nkgpu_renderer renderer) {
    Fixture f;f.box(.8f,-.8f,.8f);f.view.clip_planes={{{{0,0,-1}},0,true}};
    auto node=f.scene->reserve_node_id();Transaction create(f.scene);create.add_create(node);ChangeSet changes;assert(f.scene->commit(create,changes)==NKS_OK);create.close();
    Transaction configure(f.scene);configure.add_geometry(node,f.geometry);configure.add_material(node,f.material);
    LocalTransform moved;moved.matrix[12]=.15f;configure.add_transform(node,moved);assert(f.scene->commit(configure,changes)==NKS_OK);configure.close();
    auto plan=f.plan();NativeKitGpuExecutor gpu(renderer);std::vector<std::uint8_t> pixels;
    assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},pixels)==NKGPU_OK);
    assert(pixels[(48*96+48)*4+3]==255); // XOR across the batch would incorrectly leave a hole.
    PickResult hit;assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&hit)==NKGPU_OK&&hit.cap_plane()==0);
    std::puts("section caps: OverlappingInstancesKeepIndependentParity passed");
}

void open_mesh_is_uncapped(nkgpu_renderer renderer) {
    Fixture f;f.box(.8f,-.8f,.8f);auto &open=f.scene->geometry_store().find(f.geometry)->edit_payload();open.closed=false;open.vertices.resize(open.vertices.size()-6);
    f.view.clip_planes={{{{0,0,-1}},0,true}};NativeKitGpuExecutor gpu(renderer);std::vector<std::uint8_t> pixels;auto plan=f.plan();
    assert(gpu.capture_rgba8(plan,f.scene->snapshot(),96,96,{0,0,0,0},pixels)==NKGPU_OK);
    assert(pixels[(48*96+48)*4+3]==0);
    PickResult hit;assert(gpu.pick_pixel(plan,f.scene->snapshot(),96,96,48,48,&hit)==NKGPU_OK);assert(!hit.node.valid());
    std::puts("section caps: UndeclaredOpenMeshIsUncapped passed");
}

// Touching orange shell / faceted blue insert, cut across longitudinal B-rep edges.
// Switching only stroke_segments is the renderer equivalent of Feature edges off/on.
void touching_facets_do_not_stroke_caps(nkgpu_renderer renderer) {
    Fixture f;f.box(.85f,-.8f,.8f);
    auto prism=[&](bool inward) {
        for(int i=0;i<12;++i) {
            float a=i*6.2831853f/12,b=(i+1)*6.2831853f/12;
            P p{.45f*std::cos(a),.45f*std::sin(a),-.8f},q{.45f*std::cos(b),.45f*std::sin(b),-.8f};
            P r=q,t=p;r[2]=t[2]=.8f;
            if(inward)f.quad(t,r,q,p);else f.quad(p,q,r,t);
            P low{0,0,-.8f},high{0,0,.8f};
            if(inward){f.quad(low,p,q,low);f.quad(high,r,t,high);}
            else {f.quad(low,q,p,low);f.quad(high,t,r,high);}
        }
    };
    prism(true);
    auto insert=f.scene->reserve_geometry_id();f.scene->geometry_store().create(insert).edit_payload().closed=true;
    f.geometry=insert;prism(false);auto &mesh=*f.scene->geometry_store().find(insert);
    mesh.bounds={{{-.45f,-.45f,-.8f}},{{.45f,.45f,.8f}},true};
    auto blue=f.scene->reserve_material_id();MaterialState color;color.base_color={.2f,.4f,.8f,1};f.scene->material_store().create(blue).edit_state()=color;
    auto node=f.scene->reserve_node_id();ChangeSet changes;Transaction create(f.scene);create.add_create(node);assert(f.scene->commit(create,changes)==NKS_OK);create.close();
    Transaction configure(f.scene);configure.add_geometry(node,insert);configure.add_material(node,blue);assert(f.scene->commit(configure,changes)==NKS_OK);configure.close();
    f.view.camera.view_projection={1,0,-.25f,0,0,1,0,0,.5f,0,-.5f,0,0,0,0,1};
    f.view.clip_planes={{{{0,0,-1}},0,true}};NativeKitGpuExecutor gpu(renderer);
    for(bool perspective:{false,true})for(unsigned samples:{1u,4u}) {
        f.view.camera.view_projection[11]=perspective?-.25f:0;
        gpu.set_sample_count(samples);std::vector<std::uint8_t> plain,edged;
        mesh.edit_payload().stroke_segments.clear();auto plan=f.plan();
        assert(gpu.capture_rgba8(plan,f.scene->snapshot(),192,192,{0,0,0,0},plain)==NKGPU_OK);
        // A retained edge parallel and almost coplanar with the cut exercises
        // constant depth bias even when the plane-distance derivatives are zero.
        mesh.edit_payload().stroke_segments.push_back({{-.2f,0,-1e-8f},{.2f,0,-1e-8f},0});
        for(int i=0;i<12;++i){float a=i*6.2831853f/12;mesh.edit_payload().stroke_segments.push_back({{.45f*std::cos(a),.45f*std::sin(a),-.8f},{.45f*std::cos(a),.45f*std::sin(a),.8f},0});}
        plan=f.plan();assert(gpu.capture_rgba8(plan,f.scene->snapshot(),192,192,{0,0,0,0},edged)==NKGPU_OK);
        int changed=0;for(size_t i=0;i<plain.size();++i)changed+=plain[i]!=edged[i];
        std::printf("TouchingFacetsCapStrokeTie perspective=%d samples=%u changed channels=%d\n",perspective,samples,changed);std::fflush(stdout);
        for(size_t i=0;i<plain.size();i+=4)if(!std::equal(plain.begin()+i,plain.begin()+i+4,edged.begin()+i))
            std::printf("changed (%zu,%zu) plain=%d,%d,%d,%d edged=%d,%d,%d,%d\n",i/4%192,i/4/192,plain[i],plain[i+1],plain[i+2],plain[i+3],edged[i],edged[i+1],edged[i+2],edged[i+3]);
        std::fflush(stdout);assert(plain==edged);
    }
    // Outside caps, an ordinary coplanar surface edge must retain its tie-break.
    f.view.clip_planes.clear();mesh.edit_payload().stroke_segments.clear();auto plan=f.plan();std::vector<std::uint8_t> plain,edged;
    assert(gpu.capture_rgba8(plan,f.scene->snapshot(),192,192,{0,0,0,0},plain)==NKGPU_OK);
    mesh.edit_payload().stroke_segments.push_back({{-.2f,0,.8f},{.2f,0,.8f},0});plan=f.plan();
    assert(gpu.capture_rgba8(plan,f.scene->snapshot(),192,192,{0,0,0,0},edged)==NKGPU_OK);assert(plain!=edged);
    std::puts("section caps: TouchingFacetsCapStrokeTie and OrdinaryCoplanarEdgeTie passed");
}

int main() {
    nk_init_options init{};init.struct_size=sizeof(init);init.api_version=NK_API_VERSION;assert(nk_init(&init)==NK_OK);
    nk_window_options options{};options.struct_size=sizeof(options);options.width=96;options.height=96;options.title="Section caps";
    nk_window window{};nk_surface surface{};nkgpu_renderer renderer{};assert(nk_window_create(&options,&window)==NK_OK);
    assert(nkgpu_surface_create(window,96,96,&surface)==NKGPU_OK);bool ready=false;
    for(int i=0;i<500&&!ready;++i){nk_event event{};event.struct_size=sizeof(event);assert(nk_poll_event(&event)==NK_OK);ready=event.kind==NK_EVENT_SURFACE_READY&&event.source==surface;nk_event_release(&event);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    assert(ready&&nkgpu_renderer_create(surface,&renderer)==NKGPU_OK);
    touching_facets_do_not_stroke_caps(renderer);one_plane_and_picking(renderer);two_plane_corner_both_surface_hits_clipped(renderer);cavity_in_front_of_back_surface(renderer);hollow_core(renderer);occluded_crossings_and_material_exclusions(renderer);overlapping_instances_keep_independent_parity(renderer);open_mesh_is_uncapped(renderer);
    assert(nkgpu_renderer_destroy(renderer)==NKGPU_OK);assert(nkgpu_surface_destroy(surface)==NKGPU_OK);assert(nk_window_destroy(window)==NK_OK);nk_shutdown();
}

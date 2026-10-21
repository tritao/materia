#include "nativekit.h"
#include "nativekit_scene_render.hpp"
#include "nativekit_window.h"
#include "scene_internal.hpp"
#include "section_contours.hpp"
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

using namespace nkscene;
using Point=std::array<float,3>;
using Pixels=std::vector<std::uint8_t>;
constexpr unsigned side=192;

GeometryPayload box() {
    GeometryPayload mesh;mesh.closed=true;
    Point p[8]={{-.4f,-.8f,-.8f},{.4f,-.8f,-.8f},{.4f,.8f,-.8f},{-.4f,.8f,-.8f},
                {-.4f,-.8f,.8f},{.4f,-.8f,.8f},{.4f,.8f,.8f},{-.4f,.8f,.8f}};
    for(auto v:p)mesh.vertices.push_back({v});
    int faces[6][4]={{0,3,2,1},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
    for(auto &f:faces)for(int i:{f[0],f[1],f[2],f[0],f[2],f[3]})mesh.indices.push_back(i);
    return mesh;
}

void mesh_intersection_and_cache() {
    using namespace render_internal;
    auto mesh=std::make_shared<GeometryPayload>(box());
    auto cut=section_contour_segments(*mesh,{0,0,1,0});
    assert(cut.size()==8);double length=0;
    for(auto segment:cut){assert(segment.start[2]==0&&segment.end[2]==0);length+=std::hypot(segment.end[0]-segment.start[0],segment.end[1]-segment.start[1]);}
    assert(std::abs(length-4.8)<1e-5);
    auto expanded=std::make_shared<GeometryPayload>();
    for(auto i:mesh->indices)expanded->vertices.push_back(mesh->vertices[i]);
    auto sequential=section_contour_segments(*expanded,{0,0,1,0});
    assert(cut.size()==sequential.size());
    for(std::size_t i=0;i<cut.size();++i)assert(cut[i].start==sequential[i].start&&cut[i].end==sequential[i].end);
    // Coincident triangles produce no duplicate segments or coplanar diagonals.
    auto doubled=*mesh;doubled.indices.insert(doubled.indices.end(),mesh->indices.begin(),mesh->indices.end());
    assert(section_contour_segments(doubled,{0,0,1,0}).size()==cut.size());
    assert(section_contour_segments(*mesh,{1,0,0,-.4f}).empty());
    assert(section_contour_segments(*mesh,{0,0,1,-.8f}).empty());
    assert(section_contour_segments(*mesh,{0,0,1,-2}).empty());
    // Through four existing vertices and two vertical mesh edges: each once.
    auto diagonal=section_contour_segments(*mesh,{2,1,0,0});
    length=0;for(auto segment:diagonal) {
        for(auto p:{segment.start,segment.end})assert(std::abs(2*p[0]+p[1])<1e-6);
        length+=std::hypot(segment.end[0]-segment.start[0],segment.end[1]-segment.start[1],segment.end[2]-segment.start[2]);
    }
    assert(std::abs(length-(3.2+2*std::hypot(.8,1.6)))<1e-5);
    LocalTransform pose;pose.matrix[0]=-2;pose.matrix[5]=3;pose.matrix[10]=.5f;pose.matrix[14]=.1f;
    auto local=section_plane_local({0,0,-1,.2f},pose.matrix);
    assert(std::abs(local[2]+1)<1e-6&&std::abs(local[3]-.2)<1e-6);
    SectionContour cache;assert(cache.update(mesh,local));assert(!cache.update(mesh,local));
    pose.matrix[12]=.3f;assert(!cache.update(mesh,section_plane_local({0,0,-1,.2f},pose.matrix)));
    pose.matrix[14]=.2f;assert(cache.update(mesh,section_plane_local({0,0,-1,.2f},pose.matrix)));
    assert(cache.update(std::make_shared<GeometryPayload>(*mesh),local));
    assert(cache.update(mesh,local,1));mesh->vertices[0].position[0]-=.01f;
    assert(cache.update(mesh,local,2)); // A producer may edit an unshared payload in place.
    std::puts("section contours: IndexedAndSequential, CoplanarAndVertexDegeneracies, LocalPlaneAndCache passed");
}

struct Fixture {
    std::shared_ptr<Scene> scene=std::make_shared<Scene>();
    GeometryId geometry;MaterialId material;NodeId left,right;SceneView view;
    Fixture() {
        geometry=scene->reserve_geometry_id();auto &g=scene->geometry_store().create(geometry);g.edit_payload()=box();g.bounds={{{-.4f,-.8f,-.8f}},{{.4f,.8f,.8f}},true};
        material=scene->reserve_material_id();MaterialState m;m.base_color={.8f,.6f,.3f,1};scene->material_store().create(material).edit_state()=m;
        left=scene->reserve_node_id();right=scene->reserve_node_id();
        Transaction create(scene);create.add_create(left);create.add_create(right);commit(create);
        Transaction configure(scene);
        for(auto node:{left,right}){configure.add_geometry(node,geometry);configure.add_material(node,material);LocalTransform t;t.matrix[12]=node==left?-.4f:.4f;configure.add_transform(node,t);}
        commit(configure);
        view.camera.enabled=true;view.camera.view_projection={1,0,0,0,0,1,0,0,0,0,-1,0,0,0,0,1};
        view.clip_planes={{{{0,0,-1}},0,true}};
    }
    void commit(Transaction &t){ChangeSet changes;assert(scene->commit(t,changes)==NKS_OK);t.close();scene->publish();}
    RenderPlan plan(){scene->publish();return compile(scene->snapshot(),view);}
    Pixels capture(NativeKitGpuExecutor &gpu) {
        Pixels result;auto p=plan();assert(gpu.capture_rgba8(p,scene->snapshot(),side,side,{0,0,0,0},result)==NKGPU_OK);return result;
    }
};
bool dark(const Pixels &p,int x,int y){auto i=(y*side+x)*4;return p[i]<40&&p[i+1]<40&&p[i+2]<40&&p[i+3]>200;}

void touching_parts_and_clipping(nkgpu_renderer renderer) {
    Fixture f;NativeKitGpuExecutor gpu(renderer);
    for(unsigned samples:{1u,4u}) {
        gpu.set_sample_count(samples);auto pixels=f.capture(gpu);
        int boundary=0;for(int y=40;y<152;++y)boundary+=dark(pixels,95,y)||dark(pixels,96,y);
        assert(boundary>105);assert(!dark(pixels,80,80));
        PickResult hit;auto plan=f.plan();assert(gpu.pick_pixel(plan,f.scene->snapshot(),side,side,96,96,&hit)==NKGPU_OK);
        assert((hit.node==f.left||hit.node==f.right)&&hit.cap_plane()==0);
        // Looking obliquely must retain a full ribbon on the plane, not half a
        // centerline-depth stroke sinking into its own cap. Also mirrored parts.
        LocalTransform mirrored;mirrored.matrix[0]=-1;mirrored.matrix[12]=-.4f;
        Transaction move(f.scene);move.add_transform(f.left,mirrored);f.commit(move);
        f.view.camera.view_projection[2]=-.25f;f.view.camera.view_projection[8]=.5f;f.view.camera.view_projection[10]=-.5f;
        f.view.camera.view_projection[11]=-.25f;f.view.camera.view_projection[3]=.15f;
        pixels=f.capture(gpu);boundary=0;for(int y=40;y<152;++y)boundary+=dark(pixels,95,y)||dark(pixels,96,y);assert(boundary>105);
        // Keep Y<=0 in addition to Z<=0. No contour pixels may survive in the
        // removed upper half; no shortened remnants at the retained corner.
        f.view.clip_planes.push_back({{{0,-1,0}},0,true});pixels=f.capture(gpu);
        for(int y=0;y<94;++y)for(int x=0;x<int(side);++x)assert(pixels[(y*side+x)*4+3]==0);
        boundary=0;for(int y=99;y<152;++y)boundary+=dark(pixels,95,y)||dark(pixels,96,y);assert(boundary>48);
        f.view.clip_planes.pop_back();
    }
    std::puts("section contours: TouchingSameMaterial, CapPickRetained, PerspectiveMirroredDepthTie and TwoPlaneClipping passed");
}

void eligibility(nkgpu_renderer renderer) {
    Fixture f;NativeKitGpuExecutor gpu(renderer);
    auto no_work=[&](){auto plan=f.plan();auto stats=gpu.execute(plan,f.scene->snapshot());assert(stats.result==NKGPU_OK);assert(stats.section_contours_rebuilt==0&&stats.section_contour_cache_hits==0);};
    // Uncut and tangent parts must not create outline work.
    f.view.clip_planes[0].distance=2;no_work();
    f.view.clip_planes[0].distance=.8f;no_work();
    f.view.clip_planes[0].distance=0;
    f.scene->geometry_store().find(f.geometry)->edit_payload().closed=false;no_work();
    f.scene->geometry_store().find(f.geometry)->edit_payload().closed=true;
    auto &materials=f.scene->material_store();
    for(int mode=0;mode<3;++mode) {
        auto &m=materials.find(f.material)->edit_state();m.flags=static_cast<unsigned>(MaterialFlags::Opaque);m.alpha_mode=AlphaMode::Opaque;m.opacity=1;
        if(mode==0)m.flags|=static_cast<unsigned>(MaterialFlags::DoubleSided);
        if(mode==1){m.alpha_mode=AlphaMode::Blend;m.opacity=.5f;}
        if(mode==2)m.alpha_mode=AlphaMode::Mask;
        no_work();auto reference=f.capture(gpu);
        f.scene->geometry_store().find(f.geometry)->edit_payload().closed=false;
        assert(f.capture(gpu)==reference); // No cap or contour from closure alone.
        f.scene->geometry_store().find(f.geometry)->edit_payload().closed=true;
    }
    std::puts("section contours: UncutTangentOpenDoubleSidedTransparentAndMasked exclusions passed");
}

void cache_and_motion(nkgpu_renderer renderer) {
    Fixture f;NativeKitGpuExecutor gpu(renderer);
    auto run=[&](){auto plan=f.plan();auto s=gpu.execute(plan,f.scene->snapshot());assert(s.result==NKGPU_OK);return s;};
    auto stats=run();assert(stats.section_contours_rebuilt==2);
    stats=run();assert(stats.section_contours_rebuilt==0&&stats.section_contour_cache_hits==2);
    f.view.camera.view_projection[12]=.01f;stats=run();assert(stats.section_contours_rebuilt==0&&stats.section_contour_cache_hits==2);
    f.view.camera.view_projection[12]=0;
    f.view.clip_planes.push_back({{{0,-1,0}},0,true});stats=run();
    assert(stats.section_contours_rebuilt==2&&stats.section_contour_cache_hits==2);
    f.view.clip_planes[1].distance=.1f;stats=run();
    assert(stats.section_contours_rebuilt==2&&stats.section_contour_cache_hits==2);
    f.view.clip_planes.pop_back();stats=run();
    assert(stats.section_contours_rebuilt==0&&stats.section_contour_cache_hits==2);
    f.scene->material_store().find(f.material)->edit_state().base_color={.7f,.6f,.3f,1};stats=run();assert(stats.section_contours_rebuilt==0&&stats.section_contour_cache_hits==2);
    // Explode is a display transform: translate one part tangentially, then
    // through the plane. The former reuses local segments; the latter rebuilds.
    LocalTransform exploded;exploded.matrix[12]=.65f;Transaction move(f.scene);move.add_transform(f.right,exploded);f.commit(move);
    stats=run();assert(stats.section_contours_rebuilt==0&&stats.section_contour_cache_hits==2);
    auto image=f.capture(gpu);assert(image[(80*side+108)*4+3]==0);assert(dark(image,120,80)||dark(image,119,80));
    exploded.matrix[14]=.1f;Transaction lift(f.scene);lift.add_transform(f.right,exploded);f.commit(lift);stats=run();assert(stats.section_contours_rebuilt==1&&stats.section_contour_cache_hits==1);
    // A parent joint rotation must change the child's relative cutting plane.
    auto parent=f.scene->reserve_node_id();Transaction create(f.scene);create.add_create(parent);f.commit(create);
    Transaction attach(f.scene);attach.add_parent(f.left,parent);f.commit(attach);run();
    LocalTransform joint;float angle=.3f;joint.matrix[0]=std::cos(angle);joint.matrix[2]=-std::sin(angle);joint.matrix[8]=std::sin(angle);joint.matrix[10]=std::cos(angle);joint.matrix[12]=-.3f*std::sin(angle);joint.matrix[14]=.3f*(1-std::cos(angle));
    Transaction rotate(f.scene);rotate.add_transform(parent,joint);f.commit(rotate);stats=run();assert(stats.section_contours_rebuilt==1&&stats.section_contour_cache_hits==1);
    image=f.capture(gpu);bool moved_boundary=false;for(int x=85;x<90;++x)moved_boundary|=dark(image,x,80);assert(moved_boundary);
    // Dragging changes only relative plane poses, and never accumulates a new
    // cache entry per cursor position. Returning to a stable frame reuses both.
    for(int i=1;i<=40;++i){f.view.clip_planes[0].distance=.001f*i;stats=run();assert(stats.section_contours_rebuilt==2);}
    stats=run();assert(stats.section_contours_rebuilt==0&&stats.section_contour_cache_hits==2);
    // Replacing mesh data at an unchanged pose invalidates the local cut.
    f.scene->geometry_store().find(f.geometry)->edit_payload().vertices[0].position[0]-=.01f;
    stats=run();assert(stats.section_contours_rebuilt==2);
    std::puts("section contours: CameraAndMaterialCacheReuse, ExplodeMotion, ParentJointMotion, PlaneDrag and MeshInvalidation passed");
}

int main() {
    mesh_intersection_and_cache();
    nk_init_options init{};init.struct_size=sizeof(init);init.api_version=NK_API_VERSION;assert(nk_init(&init)==NK_OK);
    nk_window_options options{};options.struct_size=sizeof(options);options.width=options.height=side;options.title="Section contours";
    nk_window window{};nk_surface surface{};nkgpu_renderer renderer{};assert(nk_window_create(&options,&window)==NK_OK);assert(nkgpu_surface_create(window,side,side,&surface)==NKGPU_OK);
    bool ready=false;for(int i=0;i<500&&!ready;++i){nk_event e{};e.struct_size=sizeof(e);assert(nk_poll_event(&e)==NK_OK);ready=e.kind==NK_EVENT_SURFACE_READY&&e.source==surface;nk_event_release(&e);std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    assert(ready&&nkgpu_renderer_create(surface,&renderer)==NKGPU_OK);
    touching_parts_and_clipping(renderer);eligibility(renderer);cache_and_motion(renderer);
    assert(nkgpu_renderer_destroy(renderer)==NKGPU_OK);assert(nkgpu_surface_destroy(surface)==NKGPU_OK);assert(nk_window_destroy(window)==NK_OK);nk_shutdown();
}

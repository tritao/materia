#include "broadphase_scene.hpp"
#include <coal/hfield.h>
#include <cstdio>
#include <cstdlib>

namespace {
int failures=0;
void expect(bool value,const char *what) {
    if(!value) {std::printf("FAILED: %s\n",what);++failures;}
}
bool equal(double a,double b) {return a==b || (std::isnan(a) && std::isnan(b));}
bool equal(const ck::PairResult &a,const ck::PairResult &b) {
    if(a.a!=b.a || a.b!=b.b || a.body_a!=b.body_a || a.body_b!=b.body_b ||
       !equal(a.distance,b.distance) || !equal(a.required,b.required))return false;
    for(int k=0;k<3;++k)if(!equal(a.point_a[k],b.point_a[k]) || !equal(a.point_b[k],b.point_b[k]) ||
                           !equal(a.normal[k],b.normal[k]))return false;
    return true;
}
void equal(const std::vector<ck::PairResult> &a,const std::vector<ck::PairResult> &b,const char *what) {
    if(a.size()!=b.size()) {expect(false,what);return;}
    for(size_t i=0;i<a.size();++i)if(!equal(a[i],b[i])) {expect(false,what);return;}
}
void compare(ck::World &loop,ck::World &tree) {
    std::vector<ck::PairResult> a,b;
    for(double margin : {-0.02,0.0,0.005,0.25}) {
        expect(loop.check(margin,a)==tree.check(margin,b),"check support identical");
        equal(a,b,"check ids, bodies and order identical");
    }
    for(double query : {-0.03,0.0,0.005,0.5}) {
        expect(loop.distances(query,a)==tree.distances(query,b),"distance support identical");
        equal(a,b,"signed distances, witnesses, normals and stable order identical");
    }
    const double table[4]={0.005,0.0005,0.0005,0.001};
    const ck::Margins margins{2,table};
    std::vector<double> inflation(loop.body_count(),0.0);
    for(size_t i=0;i<inflation.size();++i)inflation[i]=0.01*(i%4);
    for(const double *extra : {static_cast<const double *>(nullptr),static_cast<const double *>(inflation.data())}) {
        bool fa=false,fb=false;ck::PairResult pa,pb;
        expect(loop.violation(margins,extra,fa,pa)==tree.violation(margins,extra,fb,pb),"violation support identical");
        expect(fa==fb && (!fa || equal(pa,pb)),"first violation and per-group/body required margin identical");
    }
    bool fa=false,fb=false;ck::PairResult pa,pb;
    expect(loop.closest(margins,fa,pa)==tree.closest(margins,fb,pb),"closest support identical");
    expect(fa==fb && (!fa || equal(pa,pb)),"closest pair, distance, witnesses and required margin identical");
}
const double identity[7]={0,0,0,0,0,0,1};
}

int main() {
    // One authored scene, run above and below the policy threshold. Exact
    // equality includes pair ordering and every numeric result field.
    ck_test::Cell loop(100,std::numeric_limits<size_t>::max()),tree(100,ck::World::kBroadphaseThreshold);
    for(int i=0;i<40;++i) {loop.move(i*0.17);tree.move(i*0.17);compare(loop.world,tree.world);}
    for(auto *cell : {&loop,&tree}) {
        cell->world.set_inflation(cell->links[5],0.02);
        cell->world.set_object_rule(cell->boxes[0],cell->boxes[1],CK_RULE_CHECK,0);
        cell->world.set_object_rule(cell->links[0],cell->boxes[0],CK_RULE_ALLOW,CK_PAIR_PROCESS_CONTACT);
    }
    compare(loop.world,tree.world);
    const auto la=loop.world.allow_overlapping({0,1,2,3,4,5});
    const auto ta=tree.world.allow_overlapping({0,1,2,3,4,5});
    expect(la==ta,"reference-overlap marking identical");
    compare(loop.world,tree.world);

    // Reattach, explicitly checked static pairs, and later fixed-body pose
    // updates exercise both tree membership rebuilds and incremental updates.
    for(auto *cell : {&loop,&tree}) {
        auto fixed=cell->world.add_body(0);cell->world.set_body_static(fixed,true);
        cell->world.attach(cell->boxes[0],fixed,identity);
        double value[7]={0,0,1,0,0,0,1};cell->world.set_body_pose(fixed,value);
    }
    compare(loop.world,tree.world);
    for(auto *cell : {&loop,&tree}) {
        double value[7]={0.12,0,1.1,0,0,0,1};cell->world.set_body_pose(6,value);
        cell->move(2.4);
    }
    compare(loop.world,tree.world);
    // An unbounded query must not discard poses pending in an existing tree.
    loop.move(3.2);tree.move(3.2);
    std::vector<ck::PairResult> a,b;
    loop.world.distances(INFINITY,a);tree.world.distances(INFINITY,b);
    equal(a,b,"unbounded distance query identical");compare(loop.world,tree.world);
    for(auto *cell : {&loop,&tree}) {
        cell->world.remove(cell->boxes.back());
        cell->world.attach(cell->boxes[0],-1,identity);
        cell->world.set_object_rule(cell->boxes[0],cell->boxes[1],CK_RULE_DEFAULT,0);
    }
    compare(loop.world,tree.world);

    // The production policy crosses its threshold as checked pairs disappear
    // and return. Poses made during the loop interval must reach the new tree.
    for(auto *cell : {&loop,&tree}) {
        cell->world.set_body_static(cell->bodies[4],true);
        cell->world.set_body_static(cell->bodies[5],true);
        cell->move(3.6);
    }
    compare(loop.world,tree.world);
    for(auto *cell : {&loop,&tree}) {
        cell->world.set_body_static(cell->bodies[4],false);
        cell->world.set_body_static(cell->bodies[5],false);
        cell->move(3.9);
    }
    compare(loop.world,tree.world);

    // Infinite shapes bypass the finite tree; updated height-field bounds do
    // not require changing topology or rebuilding narrow-phase functions.
    std::array<uint32_t,2> fields;
    size_t i=0;
    for(auto *cell : {&loop,&tree}) {
        double at[7]={0,0,-0.5,0,0,0,1};
        cell->world.add(-1,at,std::make_shared<coal::Halfspace>(coal::Vec3s::UnitZ(),0.0),false);
        coal::MatrixXs heights=coal::MatrixXs::Zero(3,3);
        fields[i++]=cell->world.add(-1,at,std::make_shared<coal::HeightField<coal::OBBRSS>>(2,2,heights,-1),true);
    }
    compare(loop.world,tree.world);
    double heights[9]={0,0.1,0,0.1,0.2,0.1,0,0.1,0};
    expect(loop.world.set_heights(fields[0],heights,9) && tree.world.set_heights(fields[1],heights,9),"heights updated");
    compare(loop.world,tree.world);

    // Equality at zero and at an exact positive AABB margin includes contact,
    // even though coal's tree traversal prunes with strict less-than.
    ck::World small(std::numeric_limits<size_t>::max()),large(0);
    for(auto *world : {&small,&large}) {
        auto body=world->add_body(0);
        world->add(body,identity,std::make_shared<coal::Box>(1,1,1),false);
        double touching[7]={1,0,0,0,0,0,1};
        world->add(-1,touching,std::make_shared<coal::Box>(1,1,1),false);
        touching[0]=1.25;
        world->add(-1,touching,std::make_shared<coal::Box>(1,1,1),false);
    }
    compare(small,large);
    small.check(0,a);large.check(0,b);
    expect(a.size()==1 && b.size()==1,"zero-margin boundary contact retained");
    small.check(0.25,a);large.check(0.25,b);
    expect(a.size()==2 && b.size()==2,"exact positive-margin contact retained");
    if(failures)return EXIT_FAILURE;
    std::printf("broadphase: loop and tree results are exactly identical\n");
}

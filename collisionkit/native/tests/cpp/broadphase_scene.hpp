#pragma once

#include "world.hpp"
#include "collisionkit.h"
#include <coal/shape/geometric_shapes.h>
#include <array>
#include <cmath>
#include <limits>

namespace ck_test {

inline std::array<double,7> pose(const coal::Transform3s &t) {
    coal::Quatf q(t.rotation());
    return {t.translation()[0],t.translation()[1],t.translation()[2],q.x(),q.y(),q.z(),q.w()};
}

/** Six rotary joints with alternating axes, six capsule links, and N cell
 * boxes. Both policies use the identical geometry, rules and FK poses. */
struct Cell {
    ck::World world;
    std::array<uint32_t,6> bodies, links;
    std::vector<uint32_t> boxes;
    static constexpr double lengths[6] = {0.35,0.45,0.4,0.25,0.18,0.12};
    explicit Cell(size_t n, size_t threshold) : world(threshold) {
        for (size_t i=0;i<6;++i) {
            bodies[i]=world.add_body(i==5 ? 1 : 0);
            auto offset=std::array<double,7>{0,0,lengths[i]/2,0,0,0,1};
            links[i]=world.add(bodies[i],offset.data(),std::make_shared<coal::Capsule>(0.045,lengths[i]),false);
            if (i>0) world.set_body_rule(bodies[i-1],bodies[i],CK_RULE_ALLOW,CK_PAIR_ADJACENT);
        }
        for (size_t i=0;i<n;++i) {
            // Racks distributed throughout a 3D cell, rather than duplicated
            // distant objects added only to inflate its candidate count.
            double x=-1.8+0.4*(i%10), y=-1.8+0.4*((i/10)%10), z=0.1+0.25*(i/100);
            auto offset=std::array<double,7>{x,y,z,0,0,std::sin(0.013*i),std::cos(0.013*i)};
            boxes.push_back(world.add(-1,offset.data(),std::make_shared<coal::Box>(0.18,0.18,0.16),false));
        }
        move(0);
    }
    void move(double sample) {
        auto transform=coal::Transform3s::Identity();
        for(size_t i=0;i<6;++i) {
            coal::Vec3s axis=i%3==0 ? coal::Vec3s::UnitZ() : i%3==1 ? coal::Vec3s::UnitY() : coal::Vec3s::UnitX();
            double angle=0.5*std::sin(sample*(0.7+0.11*i)+0.6*i);
            transform=transform*coal::Transform3s(Eigen::AngleAxisd(angle,axis).toRotationMatrix(),coal::Vec3s::Zero());
            auto value=pose(transform);world.set_body_pose(bodies[i],value.data());
            transform=transform*coal::Transform3s(coal::Matrix3s::Identity(),coal::Vec3s(0,0,lengths[i]));
        }
    }
};

} // namespace ck_test

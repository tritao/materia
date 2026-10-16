#include "PhysicsBackend.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
using namespace nksim;
static int checks = 0, physics_checks = 0, api_operations = 0;
void check(bool yes, const char *what) {
    ++checks;
    if (!yes) {
        std::fprintf(stderr, "FAIL %s\n", what);
        std::exit(1);
    }
}
void ok(nksim_result r) {
    ++api_operations;
    check(r == NKSIM_OK, "backend operation");
}
void near(double actual, double expected, const char *what) {
    ++physics_checks;
    if (std::abs(actual - expected) > 1e-9)
        std::fprintf(stderr, "%s: %.15g vs %.15g\n", what, actual, expected);
    check(std::abs(actual - expected) < 1e-9, what);
}
std::uint64_t body(PhysicsBackend &b, bool dynamic, double mass) {
    BackendBodyDesc d;
    d.motion_type = dynamic ? NKSIM_MOTION_DYNAMIC : NKSIM_MOTION_STATIC;
    d.mass = mass;
    d.shape_type = NKSIM_SHAPE_BOX;
    d.shape_parameters = {.1, .1, .1, 0};
    d.has_inertial_properties = true;
    d.inertia_tensor = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::uint64_t id;
    ok(b.body_create(d, &id));
    return id;
}
std::uint64_t joint(PhysicsBackend &b, std::uint64_t a, std::uint64_t child,
                    std::uint32_t kind = NKSIM_JOINT_PRISMATIC, double armature = 0, double lead = 0) {
    BackendJointDesc d;
    d.type = kind;
    d.body_a = a;
    d.body_b = child;
    d.axis_a =
        kind == NKSIM_JOINT_PRISMATIC ? std::array<double, 3>{1, 0, 0} : std::array<double, 3>{0, 0, 1};
    d.lower_limit = -10;
    d.upper_limit = 10;
    d.max_force = 100;
    d.armature = armature;
    d.lead = lead;
    std::uint64_t id;
    ok(b.joint_create(d, &id));
    return id;
}
void effort(PhysicsBackend &b, std::uint64_t j, double f) {
    BackendJointTarget t;
    t.backend_joint = j;
    t.mode = NKSIM_JOINT_TARGET_EFFORT;
    t.target = f;
    ok(b.set_joint_targets(&t, 1));
}
BackendJointState state(PhysicsBackend &b, std::uint64_t j) {
    BackendJointState s;
    s.backend_joint = j;
    ok(b.read_joint_states(&s, 1));
    return s;
}
BackendBodyState pose(PhysicsBackend &b, std::uint64_t id) {
    BackendBodyState s;
    s.backend_body = id;
    ok(b.read_body_states(&s, 1));
    return s;
}
std::unique_ptr<PhysicsBackend> world(double gx = 0, double gy = 0, double gz = 0) {
    auto b = make_test_physics_backend();
    nksim_world_desc d{};
    d.gravity[0] = gx;
    d.gravity[1] = gy;
    d.gravity[2] = gz;
    ok(b->initialize(d));
    return b;
}
void advance(PhysicsBackend &b) {
    for (int i = 0; i < 100; ++i)
        ok(b.step(.001, 1));
}
std::uint64_t collider(PhysicsBackend &b, std::uint32_t motion, double mass, std::array<double, 3> position,
                       std::array<double, 3> half) {
    BackendBodyDesc d;
    d.motion_type = motion;
    d.mass = mass;
    d.shape_type = NKSIM_SHAPE_BOX;
    d.position = position;
    d.shape_parameters = {half[0], half[1], half[2], 0};
    d.collision_layer = d.collision_mask = 1;
    std::uint64_t id;
    ok(b.body_create(d, &id));
    return id;
}
std::array<double, 3> contact_force(PhysicsBackend &b, std::uint64_t id) {
    std::vector<BackendContact> contacts;
    ok(b.read_contacts(contacts));
    std::array<double, 3> f{};
    for (const auto &c : contacts)
        if (c.wrench_valid)
            for (int axis = 0; axis < 3; ++axis) {
                if (c.body_a == id)
                    f[axis] += c.force_on_body_a[axis];
                else if (c.body_b == id)
                    f[axis] -= c.force_on_body_a[axis];
            }
    return f;
}
void near_contact(double actual, double expected, double tolerance, const char *what) {
    ++physics_checks;
    if (std::abs(actual - expected) > tolerance)
        std::fprintf(stderr, "%s: %.15g vs %.15g tolerance %.4g\n", what, actual, expected, tolerance);
    check(std::abs(actual - expected) <= tolerance, what);
}
void grip_probe(bool frictionless) {
    auto b = world(0, 0, -9.81);
    BackendBodyDesc base;
    base.motion_type = NKSIM_MOTION_KINEMATIC;
    base.shape_type = NKSIM_SHAPE_BOX;
    base.shape_parameters = {.001, .001, .001, 0};
    std::uint64_t root;
    ok(b->body_create(base, &root));
    auto left = collider(*b, NKSIM_MOTION_DYNAMIC, 1, {-.09, 0, .3}, {.02, .06, .05});
    auto right = collider(*b, NKSIM_MOTION_DYNAMIC, 1, {.09, 0, .3}, {.02, .06, .05});
    auto object = collider(*b, NKSIM_MOTION_DYNAMIC, 1, {0, 0, .3}, {.05, .05, .05});
    auto jaw = [&](std::uint64_t id, double x) {
        BackendJointDesc d;
        d.type = NKSIM_JOINT_PRISMATIC;
        d.body_a = root;
        d.body_b = id;
        d.axis_a = {1, 0, 0};
        d.anchor_a = {x, 0, .3};
        d.lower_limit = -.04;
        d.upper_limit = .04;
        d.max_force = 100;
        std::uint64_t j;
        ok(b->joint_create(d, &j));
        return j;
    };
    auto leader = jaw(left, -.09), follower = jaw(right, .09);
    BackendJointCoupling c;
    c.leader = leader;
    c.follower = follower;
    c.ratio = -1;
    ok(b->joint_couple(c));
    if (frictionless)
        for (auto pad : {left, right}) {
            BackendContactPair pair;
            pair.body_a = pad;
            pair.body_b = object;
            pair.surface.friction_dimensions = 1;
            pair.surface.friction = {0, 0, 0};
            ok(b->contact_pair_create(pair));
        }
    effort(*b, leader, 80);
    for (int i = 0; i < 200; ++i)
        ok(b->step(.001, 1));
    auto initial = pose(*b, object);
    if (frictionless) {
        check(initial.position[2] < .2 && initial.linear_velocity[2] < -1.5,
              "zero friction cannot hold a part through assumed width/contact");
        return;
    }
    near_contact(initial.linear_velocity[2], 0, 1e-5, "friction holds gravity-loaded work");
    near_contact(contact_force(*b, object)[2], 9.81, .02, "measured contact force balances work weight");
    near_contact(std::abs(contact_force(*b, left)[0]), 40, .02, "one jaw receives half the coupled effort");
    auto carrier = pose(*b, root);
    carrier.linear_velocity = {0, 0, .1};
    for (int i = 0; i < 1000; ++i) {
        carrier.position[2] += .0001;
        ok(b->body_set_state(root, carrier));
        ok(b->step(.001, 1));
    }
    auto lifted = pose(*b, object);
    near_contact(lifted.position[2] - initial.position[2], .1, .001,
                 "friction transfers moving carrier velocity without object attachment");
    near_contact(lifted.linear_velocity[2], .1, 1e-5, "gripped work follows measured carrier speed");
    carrier.linear_velocity = {0, 0, 0};
    ok(b->body_set_state(root, carrier));
    for (int i = 0; i < 100; ++i)
        ok(b->step(.001, 1));
    auto held = pose(*b, object);
    effort(*b, leader, -80);
    for (int i = 0; i < 200; ++i)
        ok(b->step(.001, 1));
    auto released = pose(*b, object);
    check(released.position[2] < held.position[2] - .15 && released.linear_velocity[2] < -1.5,
          "opening real jaw contacts releases the work to gravity");
    std::printf("friction grip height %.9g -> %.9g -> %.9g\n", initial.position[2], lifted.position[2],
                released.position[2]);
}
int main() {
    {
        auto b = world();
        auto a = body(*b, false, 0), c = body(*b, true, 1);
        auto j = joint(*b, a, c);
        effort(*b, j, 10);
        advance(*b);
        auto s = state(*b, j);
        near(s.velocity, 1, "one kg slider velocity");
        near(s.position, .0505, "one kg slider displacement");
    }
    {
        auto b = world();
        auto a = body(*b, false, 0), c = body(*b, true, 1);
        auto j = joint(*b, a, c, NKSIM_JOINT_PRISMATIC, 3);
        effort(*b, j, 10);
        advance(*b);
        auto s = state(*b, j);
        near(s.velocity, .25, "armature contributes to effective mass");
        near(s.position, .012625, "armature displacement");
    }
    {
        auto b = world();
        auto a = body(*b, false, 0), left = body(*b, true, 1), right = body(*b, true, 3);
        auto j = joint(*b, a, left), f = joint(*b, a, right);
        BackendJointCoupling c;
        c.leader = j;
        c.follower = f;
        c.ratio = -1;
        ok(b->joint_couple(c));
        effort(*b, j, 10);
        advance(*b);
        auto s = state(*b, j), other = state(*b, f);
        near(s.velocity, .25, "coupled unequal masses share effort");
        near(s.position, .012625, "coupled displacement");
        near(other.position, -s.position, "mirrored follower coordinate");
    }
    {
        auto b = world();
        auto a = body(*b, true, 1), c = body(*b, true, 1);
        auto j = joint(*b, a, c);
        effort(*b, j, 10);
        advance(*b);
        auto s = state(*b, j);
        auto root = pose(*b, a), child = pose(*b, c);
        near(s.velocity, 2, "floating parent relative velocity");
        near(root.linear_velocity[0], -1, "actuator reaction on floating parent");
        near(child.linear_velocity[0], 1, "child world velocity");
        near(root.position[0], -.0505, "floating root displacement");
        near(child.position[0], .0505, "child world displacement");
    }
    {
        auto b = world();
        auto a = body(*b, false, 0), c = body(*b, true, 2);
        const double lead = .1;
        auto j = joint(*b, a, c, NKSIM_JOINT_HELICAL, .1, lead);
        effort(*b, j, 10);
        advance(*b);
        auto s = state(*b, j);
        const double pitch = lead / (2 * std::acos(-1.0)), m = 1 + 2 * pitch * pitch + .1;
        near(s.velocity, 1 / m, "helical rotational and translational inertia");
        near(s.position, .0505 / m, "helical integrated angle");
        near(pose(*b, c).position[2], s.position * pitch, "helical lead translation");
    }
    {
        auto b = world(0, -9.81, 0);
        auto a = body(*b, false, 0), c = body(*b, true, 1);
        auto root = pose(*b, a);
        root.rotation = {0, 0, std::sin(std::acos(-1.0) / 4), std::cos(std::acos(-1.0) / 4)};
        ok(b->body_set_state(a, root));
        auto j = joint(*b, a, c);
        effort(*b, j, 9.81);
        advance(*b);
        auto st = state(*b, j);
        near(st.velocity, 0, "gravity compensation in rotated parent frame");
        near(st.position, 0, "rotated gravity equilibrium");
    }
    {
        auto b = world(0, -9.81, 0);
        auto a = body(*b, false, 0);
        BackendBodyDesc d;
        d.motion_type = NKSIM_MOTION_DYNAMIC;
        d.mass = 1;
        d.shape_type = NKSIM_SHAPE_BOX;
        d.shape_parameters = {.1, .1, .1, 0};
        d.has_inertial_properties = true;
        d.center_of_mass = {1, 0, 0};
        d.inertia_tensor = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        std::uint64_t c;
        ok(b->body_create(d, &c));
        auto j = joint(*b, a, c, NKSIM_JOINT_REVOLUTE);
        effort(*b, j, 0);
        ok(b->step(.001, 1));
        auto st = state(*b, j);
        near(st.velocity, -.004905, "COM gravity moment and parallel-axis inertia");
        near(st.position, -.000004905, "COM-offset pendulum first step");
    }
    {
        auto b = world();
        auto a = body(*b, false, 0), c = body(*b, true, 1), rotor = body(*b, true, 1);
        auto j = joint(*b, a, c);
        effort(*b, j, 0);
        BackendBodyForce f;
        f.backend_body = rotor;
        f.torque = {0, 0, 10};
        for (int k = 0; k < 100; ++k) {
            ok(b->apply_forces(&f, 1));
            ok(b->step(.001, 1));
        }
        auto st = pose(*b, rotor);
        near(st.angular_velocity[2], 1, "free root angular acceleration");
        near(st.rotation[2], std::sin(.0505 / 2), "world angular velocity integrates quaternion");
        near(st.rotation[3], std::cos(.0505 / 2), "integrated quaternion remains normalized");
    }
    {
        auto b = world();
        auto a = body(*b, false, 0), c = body(*b, true, 1);
        auto j = joint(*b, a, c);
        effort(*b, j, 0);
        BackendBodyDesc d;
        d.motion_type = NKSIM_MOTION_DYNAMIC;
        d.mass = 1;
        d.shape_type = NKSIM_SHAPE_BOX;
        d.shape_parameters = {.1, .1, .1, 0};
        d.has_inertial_properties = true;
        d.inertia_tensor = {1, 0, 0, 0, 2, 0, 0, 0, 3};
        std::uint64_t rotor;
        ok(b->body_create(d, &rotor));
        auto st = pose(*b, rotor);
        st.angular_velocity = {.5, .75, 1};
        ok(b->body_set_state(rotor, st));
        ok(b->step(.001, 1));
        st = pose(*b, rotor);
        near(st.angular_velocity[0], .5 - .00075, "gyroscopic x acceleration");
        near(st.angular_velocity[1], .75 + .0005, "gyroscopic y acceleration");
        near(st.angular_velocity[2], 1 - .000125, "gyroscopic z acceleration");
    }
    {
        auto b = world();
        auto a = body(*b, false, 0), c = collider(*b, NKSIM_MOTION_DYNAMIC, 1, {0, 0, 0}, {.05, .05, .05});
        collider(*b, NKSIM_MOTION_STATIC, 0, {.15, 0, 0}, {.05, .2, .2});
        auto j = joint(*b, a, c);
        effort(*b, j, 10);
        for (int k = 0; k < 1000; ++k)
            ok(b->step(.001, 1));
        auto st = state(*b, j);
        near_contact(st.position, .05, 2e-6, "contact stops an effort-driven slider at the wall");
        near_contact(st.velocity, 0, 1e-6, "blocked slider reaches rest");
        near_contact(contact_force(*b, c)[0], -10, .01, "blocked slider has measured reaction force");
    }
    {
        auto b = world();
        auto a = body(*b, false, 0), c = body(*b, true, 1);
        BackendJointDesc d;
        d.type = NKSIM_JOINT_PRISMATIC;
        d.body_a = a;
        d.body_b = c;
        d.axis_a = {1, 0, 0};
        d.lower_limit = -.02;
        d.upper_limit = .02;
        d.max_force = 100;
        std::uint64_t j;
        ok(b->joint_create(d, &j));
        effort(*b, j, 10);
        for (int k = 0; k < 1000; ++k)
            ok(b->step(.001, 1));
        auto st = state(*b, j);
        near(st.position, .02, "force-aware upper joint limit");
        near(st.velocity, 0, "joint limit reaction cancels outward velocity");
    }
    {
        auto b = world(0, 0, -9.81);
        auto a = body(*b, false, 0), c = body(*b, true, 1);
        auto j = joint(*b, a, c);
        effort(*b, j, 0);
        collider(*b, NKSIM_MOTION_STATIC, 0, {0, 0, 0}, {1, 1, .05});
        auto object = collider(*b, NKSIM_MOTION_DYNAMIC, 1, {0, 0, .2}, {.05, .05, .05});
        for (int k = 0; k < 1000; ++k)
            ok(b->step(.001, 1));
        auto st = pose(*b, object);
        near_contact(st.position[2], .1, 2e-6, "four-point floor manifold supports a free body");
        near_contact(st.linear_velocity[2], 0, 1e-5, "supported free body reaches rest");
        near_contact(contact_force(*b, object)[2], 9.81, .01, "floor contact impulse reports weight");
    }
    for (double armature : {0.0, 2.0}) {
        auto b = world();
        auto base = body(*b, false, 0), a = body(*b, true, 1), c = body(*b, true, 2), f = body(*b, true, 3);
        auto j1 = joint(*b, base, a), j2 = joint(*b, base, c),
             jf = joint(*b, base, f, NKSIM_JOINT_PRISMATIC, armature);
        for (auto leader : {j1, j2}) {
            BackendJointCoupling coupling;
            coupling.leader = leader;
            coupling.follower = jf;
            coupling.ratio = 1;
            ok(b->joint_couple(coupling));
        }
        effort(*b, j1, 10);
        effort(*b, j2, 0);
        advance(*b);
        const double v1 = armature == 0 ? 5.0 / 11 : 7.0 / 17, v2 = armature == 0 ? -3.0 / 11 : -5.0 / 17;
        near(state(*b, j1).velocity, v1, "differential first master inertia");
        near(state(*b, j2).velocity, v2, "differential second master reaction");
        near(state(*b, jf).velocity, v1 + v2, "differential sums both master velocities");
        near(state(*b, jf).position, (v1 + v2) * .0505,
             "differential displacement includes armature cross terms");
    }
    {
        auto b = world(0, 0, -9.81);
        auto root = body(*b, false, 0), dummy = body(*b, true, 1);
        auto j = joint(*b, root, dummy);
        effort(*b, j, 0);
        collider(*b, NKSIM_MOTION_STATIC, 0, {0, 0, 0}, {1, 1, .05});
        BackendBodyDesc d;
        d.motion_type = NKSIM_MOTION_DYNAMIC;
        d.mass = 1;
        d.shape_type = NKSIM_SHAPE_CONVEX;
        d.position = {0, 0, .2};
        d.collision_layer = d.collision_mask = 1;
        d.has_inertial_properties = false;
        for (int x : {-1, 1})
            for (int y : {-1, 1})
                for (int z : {-1, 1}) {
                    d.shape_vertices.push_back(x * .05f);
                    d.shape_vertices.push_back(y * .05f);
                    d.shape_vertices.push_back(z * .05f);
                }
        std::uint64_t object;
        ok(b->body_create(d, &object));
        for (int i = 0; i < 1000; ++i)
            ok(b->step(.001, 1));
        near_contact(pose(*b, object).position[2], .1, 2e-6, "convex face manifold supports free body");
        near_contact(pose(*b, object).linear_velocity[2], 0, 1e-5, "convex supported body rests");
        near_contact(contact_force(*b, object)[2], 9.81, .01,
                     "convex manifold publishes actual support force");
    }
    for (bool over_gap : {false, true}) {
        auto b = world(0, 0, -9.81);
        auto root = body(*b, false, 0), dummy = body(*b, true, 1);
        auto j = joint(*b, root, dummy);
        effort(*b, j, 0);
        BackendBodyDesc fixture;
        fixture.motion_type = NKSIM_MOTION_STATIC;
        fixture.shape_type = NKSIM_SHAPE_BOX;
        fixture.collision_layer = fixture.collision_mask = 1;
        fixture.position = {.3, .4, .1};
        fixture.rotation = {0, 0, std::sqrt(.5), std::sqrt(.5)};
        for (double x : {-.2, .2}) {
            BackendShapePart part;
            part.type = NKSIM_SHAPE_BOX;
            part.parameters = {.05, .2, .05, 0};
            part.position = {x, 0, 0};
            fixture.shape_parts.push_back(part);
        }
        std::uint64_t fixture_id;
        ok(b->body_create(fixture, &fixture_id));
        auto object = collider(*b, NKSIM_MOTION_DYNAMIC, 1, {.3, over_gap ? .4 : .6, .3}, {.04, .04, .05});
        for (int i = 0; i < 400; ++i)
            ok(b->step(.001, 1));
        auto st = pose(*b, object);
        if (over_gap) {
            check(st.position[2] < -.4, "compound empty gap stays open");
            near_contact(contact_force(*b, object)[2], 0, 1e-12,
                         "empty compound gap invents no support force");
        } else {
            near_contact(st.position[2], .2, 2e-6, "transformed compound piece supports body");
            near_contact(contact_force(*b, object)[2], 9.81, .01, "compound support reports actual weight");
            std::vector<BackendContact> contacts;
            ok(b->read_contacts(contacts));
            bool correct = false;
            for (auto c : contacts)
                if (c.body_a == fixture_id && c.body_b == object) {
                    check(c.part_a == 1 && c.part_b == -1, "compound contact preserves source piece indices");
                    correct = true;
                }
            check(correct, "compound contact is published");
        }
    }
    {
        auto b = world();
        auto base = body(*b, false, 0), dummy = body(*b, true, 1);
        auto j = joint(*b, base, dummy);
        effort(*b, j, 0);
        BackendBodyDesc d;
        d.motion_type = NKSIM_MOTION_DYNAMIC;
        d.mass = 2;
        for (double x : {-1, 1}) {
            BackendShapePart part;
            part.type = NKSIM_SHAPE_BOX;
            part.parameters = {.1, .1, .1, 0};
            part.position = {x, 0, 0};
            d.shape_parts.push_back(part);
        }
        std::uint64_t object;
        ok(b->body_create(d, &object));
        BackendBodyForce force;
        force.backend_body = object;
        force.torque = {0, 2 * (1 + .02 / 3), 0};
        for (int i = 0; i < 100; ++i) {
            ok(b->apply_forces(&force, 1));
            ok(b->step(.001, 1));
        }
        near(pose(*b, object).angular_velocity[1], .1,
             "compound parallel-axis inertia determines torque response");
        ok(b->body_set_mass(object, 4));
        for (int i = 0; i < 100; ++i) {
            ok(b->apply_forces(&force, 1));
            ok(b->step(.001, 1));
        }
        near(pose(*b, object).angular_velocity[1], .15, "mass change scales inferred rotational inertia");
    }
    for (unsigned dimensions : {3u, 4u}) {
        auto b = world(0, 0, -9.81);
        auto base = body(*b, false, 0), dummy = body(*b, true, 1);
        auto j = joint(*b, base, dummy);
        effort(*b, j, 0);
        auto floor = collider(*b, NKSIM_MOTION_STATIC, 0, {0, 0, 0}, {1, 1, .05});
        BackendBodyDesc d;
        d.motion_type = NKSIM_MOTION_DYNAMIC;
        d.mass = 1;
        d.shape_type = NKSIM_SHAPE_BOX;
        d.shape_parameters = {.0001, .0001, .0001, 0};
        d.position = {0, 0, .0501};
        d.collision_layer = d.collision_mask = 1;
        d.has_inertial_properties = true;
        d.inertia_tensor = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        std::uint64_t object;
        ok(b->body_create(d, &object));
        BackendContactPair pair;
        pair.body_a = floor;
        pair.body_b = object;
        pair.surface.friction_dimensions = dimensions;
        pair.surface.friction = {.5, .01, 0};
        ok(b->contact_pair_create(pair));
        BackendBodyForce force;
        force.backend_body = object;
        force.torque = {0, 0, .02};
        for (int i = 0; i < 100; ++i) {
            ok(b->apply_forces(&force, 1));
            ok(b->step(.001, 1));
        }
        auto st = pose(*b, object);
        if (dimensions == 3)
            check(st.angular_velocity[2] > .0018, "sliding contact cannot invent torsional holding");
        else {
            near_contact(st.angular_velocity[2], 0, 1e-7,
                         "declared torsional friction resists normal-axis torque");
            std::vector<BackendContact> contacts;
            ok(b->read_contacts(contacts));
            double torque = 0;
            for (auto c : contacts)
                if (c.body_b == object)
                    torque -= c.torque_on_body_a[2];
            near_contact(torque, -.02, .001, "torsional impulse publishes actual contact couple");
        }
    }
    for (double load : {.02, .09}) {
        auto b = world(0, 0, -9.81);
        auto base = body(*b, false, 0), dummy = body(*b, true, 1);
        auto j = joint(*b, base, dummy);
        effort(*b, j, 0);
        auto floor = collider(*b, NKSIM_MOTION_STATIC, 0, {0, 0, 0}, {1, 1, .05});
        BackendBodyDesc d;
        d.motion_type = NKSIM_MOTION_DYNAMIC;
        d.mass = 1;
        d.shape_type = NKSIM_SHAPE_BOX;
        d.shape_parameters = {.0001, .0001, .0001, 0};
        d.position = {0, 0, .0501};
        d.collision_layer = d.collision_mask = 1;
        d.has_inertial_properties = true;
        d.inertia_tensor = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        std::uint64_t object;
        ok(b->body_create(d, &object));
        BackendContactPair pair;
        pair.body_a = floor;
        pair.body_b = object;
        pair.surface.friction_dimensions = 6;
        pair.surface.friction = {.5, .01, .01};
        ok(b->contact_pair_create(pair));
        BackendBodyForce force;
        force.backend_body = object;
        force.torque = {load, load, 0};
        for (int i = 0; i < 100; ++i) {
            ok(b->apply_forces(&force, 1));
            ok(b->step(.001, 1));
        }
        auto st = pose(*b, object);
        std::vector<BackendContact> contacts;
        ok(b->read_contacts(contacts));
        double tx = 0, ty = 0, normal = 0;
        for (auto c : contacts)
            if (c.body_b == object) {
                tx -= c.torque_on_body_a[0];
                ty -= c.torque_on_body_a[1];
                normal += c.force_on_body_a[2] * -1;
            }
        check(std::hypot(tx, ty) <= .01 * normal + 1e-8, "rolling couple obeys shared circular limit");
        if (load == .02) {
            near_contact(st.angular_velocity[0], 0, 1e-7,
                         "rolling friction holds first oblique torque component");
            near_contact(st.angular_velocity[1], 0, 1e-7,
                         "rolling friction holds second oblique torque component");
            near_contact(tx, -load, .001, "measured rolling couple opposes first torque");
            near_contact(ty, -load, .001, "measured rolling couple opposes second torque");
        } else
            check(st.angular_velocity[0] > .001 && st.angular_velocity[1] > .001,
                  "over-limit diagonal torque rolls instead of receiving independent axis caps");
    }
    for (int mode = 0; mode < 6; ++mode) {
        auto b = world(0, 0, -9.81);
        auto base = body(*b, false, 0), dummy = body(*b, true, 1);
        auto j = joint(*b, base, dummy);
        effort(*b, j, 0);
        BackendBodyDesc d;
        d.motion_type = NKSIM_MOTION_STATIC;
        d.collision_layer = 1;
        d.collision_mask = mode == 5 ? 0 : 1;
        BackendShapePart part;
        part.type = NKSIM_SHAPE_BOX;
        part.parameters = {1, 1, .05, 0};
        part.contact_filter = mode == 1 || mode == 2 ? NKSIM_CONTACT_PAIRS_ONLY
                              : mode == 3            ? NKSIM_CONTACT_PAIRS_AND_ENVIRONMENT
                                                     : NKSIM_CONTACT_LAYERS;
        d.shape_parts.push_back(part);
        if (mode == 0 || mode == 2)
            d.collision_layer = d.collision_mask = 0;
        std::uint64_t floor;
        ok(b->body_create(d, &floor));
        auto object = collider(*b, NKSIM_MOTION_DYNAMIC, 1, {0, 0, .2}, {.05, .05, .05});
        if (mode == 2) {
            BackendContactPair pair;
            pair.body_a = object;
            pair.body_b = floor;
            pair.surface.friction_dimensions = 1;
            ok(b->contact_pair_create(pair));
        }
        if (mode == 4)
            ok(b->body_exclude_pair(floor, object));
        for (int i = 0; i < 400; ++i)
            ok(b->step(.001, 1));
        bool supports = mode == 2 || mode == 3 || mode == 5;
        if (supports) {
            near_contact(pose(*b, object).position[2], .1, 2e-6,
                         "declared pair/environment/layer permits physical support");
            near_contact(contact_force(*b, object)[2], 9.81, .01, "enabled filtered contact reports weight");
        } else {
            check(pose(*b, object).position[2] < -.5, "filtered or excluded contact permits free fall");
            near_contact(contact_force(*b, object)[2], 0, 1e-12, "disabled contact invents no force");
        }
    }
    {
        auto b = world();
        auto root = body(*b, false, 0), child = body(*b, true, 1);
        auto j = joint(*b, root, child);
        effort(*b, j, 0);
        ok(b->step(.001, 1));
        auto carrier = pose(*b, root);
        carrier.position[0] = .001;
        carrier.linear_velocity[0] = 1;
        ok(b->body_set_state(root, carrier));
        ok(b->step(.001, 1));
        near(state(*b, j).velocity, -1,
             "accelerating prescribed carrier produces relative inertial response");
        near(pose(*b, child).linear_velocity[0], 0,
             "free slider preserves world velocity under carrier acceleration");
        carrier.position[0] += .001;
        ok(b->body_set_state(root, carrier));
        ok(b->step(.001, 1));
        near(state(*b, j).velocity, -1, "constant carrier velocity invents no further acceleration");
        carrier.linear_velocity[0] = 0;
        ok(b->body_set_state(root, carrier));
        ok(b->step(.001, 1));
        near(state(*b, j).velocity, 0, "carrier stop preserves world momentum");
    }
    {
        auto b = world();
        auto root = body(*b, false, 0), child = body(*b, true, 1);
        auto j = joint(*b, root, child, NKSIM_JOINT_REVOLUTE);
        effort(*b, j, 0);
        ok(b->step(.001, 1));
        auto carrier = pose(*b, root);
        carrier.rotation = {0, 0, std::sin(.0005), std::cos(.0005)};
        carrier.angular_velocity = {0, 0, 1};
        ok(b->body_set_state(root, carrier));
        ok(b->step(.001, 1));
        near(state(*b, j).velocity, -1, "rotating carrier transfers angular acceleration bias");
        near(pose(*b, child).angular_velocity[2], 0, "free rotor preserves world angular momentum");
    }
    {
        auto b = world();
        auto root = body(*b, false, 0), spare = body(*b, true, 1), parent = body(*b, true, 1),
             child = body(*b, true, 1);
        auto removed = joint(*b, root, spare), prescribed = joint(*b, root, parent),
             free = joint(*b, parent, child);
        BackendJointTarget target;
        target.backend_joint = prescribed;
        target.mode = NKSIM_JOINT_TARGET_VELOCITY;
        target.target = 1;
        ok(b->set_joint_targets(&target, 1));
        effort(*b, free, 0);
        ok(b->step(.001, 1));
        ok(b->joint_destroy(removed));
        joint(*b, root, spare);
        target.target = 2;
        ok(b->set_joint_targets(&target, 1));
        ok(b->step(.001, 1));
        near(state(*b, free).velocity, -1,
             "joint replacement with unchanged count retains correct acceleration history");
        near(pose(*b, child).linear_velocity[0], 1, "topology change preserves unconstrained child momentum");
    }
    {
        auto b = world();
        auto root = body(*b, false, 0), parent = body(*b, true, 1), child = body(*b, true, 1);
        auto p = joint(*b, root, parent), free = joint(*b, parent, child);
        BackendJointTarget target;
        target.backend_joint = p;
        target.mode = NKSIM_JOINT_TARGET_VELOCITY;
        target.target = 1;
        ok(b->set_joint_targets(&target, 1));
        effort(*b, free, 0);
        ok(b->step(.001, 1));
        BackendBodyState reset;
        ok(b->body_set_state(parent, reset));
        ok(b->step(.001, 1));
        near(state(*b, free).velocity, 0, "explicit joint reset clears stale acceleration history");
        ok(b->set_joint_targets(&target, 1));
        ok(b->step(.001, 1));
        near(state(*b, free).velocity, -1, "new motion after reset uses reset velocity as baseline");
    }
    {
        auto b = world();
        auto root = body(*b, false, 0), child = body(*b, true, 1);
        auto j = joint(*b, root, child);
        effort(*b, j, 10);
        advance(*b);
        BackendJointTarget target;
        target.backend_joint = j;
        target.mode = NKSIM_JOINT_TARGET_VELOCITY;
        target.target = .5;
        ok(b->set_joint_targets(&target, 1));
        ok(b->step(.001, 1));
        effort(*b, j, 0);
        ok(b->step(.001, 1));
        near(state(*b, j).velocity, .5,
             "return from prescribed velocity to effort preserves commanded velocity");
    }
    {
        auto b = world();
        auto base = body(*b, false, 0), dummy = body(*b, true, 1);
        auto j = joint(*b, base, dummy);
        effort(*b, j, 0);
        std::vector<std::uint64_t> objects;
        std::vector<BackendBodyForce> forces;
        for (int i = 0; i < 64; ++i) {
            auto id = body(*b, true, i + 1);
            objects.push_back(id);
            BackendBodyForce f;
            f.backend_body = id;
            f.force = {10, 0, 0};
            forces.push_back(f);
        }
        for (int i = 0; i < 100; ++i) {
            ok(b->apply_forces(forces.data(), forces.size()));
            ok(b->step(.001, 1));
        }
        for (int i = 0; i < 64; ++i) {
            near(pose(*b, objects[i]).linear_velocity[0], 1.0 / (i + 1),
                 "independent inventory bodies preserve their own mass response");
            near(pose(*b, objects[i]).position[0], .0505 / (i + 1),
                 "independent inventory displacements remain decoupled");
        }
    }
    for (double tau : {.02, .04})
        for (double timestep : {.0005, .001, .002}) {
            auto b = world(0, 0, -9.81);
            auto base = body(*b, false, 0);
            auto floor = collider(*b, NKSIM_MOTION_STATIC, 0, {0, 0, 0}, {1, 1, .05});
            auto object = collider(*b, NKSIM_MOTION_DYNAMIC, 1, {0, 0, .1}, {.05, .05, .05});
            BackendJointDesc jd;
            jd.type = NKSIM_JOINT_PRISMATIC;
            jd.body_a = base;
            jd.body_b = object;
            jd.axis_a = {0, 0, 1};
            jd.anchor_a = {0, 0, .1};
            jd.lower_limit = -1;
            jd.upper_limit = 1;
            std::uint64_t j;
            ok(b->joint_create(jd, &j));
            effort(*b, j, 0);
            BackendContactPair pair;
            pair.body_a = floor;
            pair.body_b = object;
            pair.surface.friction_dimensions = 1;
            pair.surface.contact_time_constant = tau;
            pair.surface.contact_damping_ratio = 1;
            ok(b->contact_pair_create(pair));
            for (int i = 0; i < static_cast<int>(2 / timestep); ++i)
                ok(b->step(timestep, 1));
            near_contact(pose(*b, object).position[2], .1 - 9.81 * tau * tau / 4, 1e-7,
                         "declared compliant contact matches four-point spring equilibrium");
            near_contact(pose(*b, object).linear_velocity[2], 0, 1e-7, "damped compliant support settles");
            near_contact(contact_force(*b, object)[2], 9.81, 1e-5,
                         "compliant impulse reports real supported weight");
        }
    {
        auto b = world(0, 0, -9.81);
        auto base = body(*b, false, 0), dummy = body(*b, true, 1);
        auto j = joint(*b, base, dummy);
        effort(*b, j, 0);
        collider(*b, NKSIM_MOTION_STATIC, 0, {0, 0, 0}, {1, 1, .05});
        BackendBodyDesc d;
        d.motion_type = NKSIM_MOTION_DYNAMIC;
        d.mass = 1;
        d.shape_type = NKSIM_SHAPE_CONVEX;
        d.position = {0, 0, .2};
        d.collision_layer = d.collision_mask = 1;
        for (int side = 0; side < 32; ++side) {
            const double angle = side * 2 * std::acos(-1.0) / 32;
            for (double z : {-.05, .05}) {
                d.shape_vertices.push_back(.05 * std::cos(angle));
                d.shape_vertices.push_back(.05 * std::sin(angle));
                d.shape_vertices.push_back(z);
            }
        }
        std::uint64_t object;
        ok(b->body_create(d, &object));
        for (int i = 0; i < 500; ++i)
            ok(b->step(.001, 1));
        near_contact(pose(*b, object).position[2], .1, 2e-6,
                     "many-edge convex prism rests after axis deduplication");
        near_contact(contact_force(*b, object)[2], 9.81, .01,
                     "many-edge convex prism reports its supported weight");
    }
    grip_probe(false);
    grip_probe(true);
    std::printf("Generalized effort: %d analytical assertions and %d API operations passed; box, convex and "
                "compound contacts\n",
                checks - api_operations, api_operations);
}

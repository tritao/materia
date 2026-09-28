// Imports an MJCF robot description into a RobotModel v6 artifact.
//
// The file is compiled by the vendored MuJoCo, so default classes, includes,
// angle conventions, `fromto` geometry and inertia computed from geometry mean
// exactly what MuJoCo makes of them (robotkit/plans/HUMANOID.md, HU-D2). The
// importer then reads the compiled mjModel:
//
// - each body under the world becomes a link; its single hinge or slide joint,
//   or none (fixed), becomes the joint to its parent; a free joint on the root
//   body sets floatingBase;
// - geoms that take part in contact (nonzero contype or conaffinity, or named
//   in an explicit contact pair) become primitive collision shapes;
// - the remaining mesh geoms of a body are merged into one binary STL in the
//   link frame and referenced as its visual geometry;
// - joint actuators become actuators with a simple transmission;
// - sites read by accelerometer, gyro or orientation sensors become IMU frames.
//
// Usage: robotkit_mjcf_import <model.xml> <output-directory>
// Writes <output-directory>/robot.json and <output-directory>/meshes/*.stl.

#include <mujoco/mujoco.h>

#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>; // x, y, z, w

Quat from_mujoco(const mjtNum *wxyz) {
    Quat q{wxyz[1], wxyz[2], wxyz[3], wxyz[0]};
    double norm = 0.0;
    for (double value : q) norm += value * value;
    norm = std::sqrt(norm);
    for (double &value : q) value /= norm;
    return q;
}

Vec3 rotate(const Quat &q, const Vec3 &v) {
    const Vec3 u{q[0], q[1], q[2]};
    const Vec3 t{2.0 * (u[1] * v[2] - u[2] * v[1]), 2.0 * (u[2] * v[0] - u[0] * v[2]),
                 2.0 * (u[0] * v[1] - u[1] * v[0])};
    return {v[0] + q[3] * t[0] + u[1] * t[2] - u[2] * t[1],
            v[1] + q[3] * t[1] + u[2] * t[0] - u[0] * t[2],
            v[2] + q[3] * t[2] + u[0] * t[1] - u[1] * t[0]};
}

Vec3 vec(const mjtNum *values) { return {values[0], values[1], values[2]}; }

std::string number(double value) {
    if (!std::isfinite(value)) throw std::runtime_error("non-finite value in model");
    if (value == 0.0) return "0";
    // The shortest text that reads back as exactly this double.
    char buffer[32];
    const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
    return std::string(buffer, result.ptr);
}

std::string json_string(const std::string &text) {
    std::string out = "\"";
    for (const char c : text) {
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\n': out += "\\n"; break;
        case '\t': out += "\\t"; break;
        default:
            if (static_cast<unsigned char>(c) < 0x20) {
                char buffer[8];
                std::snprintf(buffer, sizeof(buffer), "\\u%04x", c);
                out += buffer;
            } else {
                out += c;
            }
        }
    }
    return out + "\"";
}

template <std::size_t N>
std::string array(const std::array<double, N> &values) {
    std::string out = "[";
    for (std::size_t i = 0; i < N; ++i) out += (i ? ", " : "") + number(values[i]);
    return out + "]";
}

std::string array(const std::vector<double> &values) {
    std::string out = "[";
    for (std::size_t i = 0; i < values.size(); ++i) out += (i ? ", " : "") + number(values[i]);
    return out + "]";
}

std::string name_of(const mjModel *m, mjtObj type, int id, const char *fallback) {
    const char *name = mj_id2name(m, type, id);
    return name && *name ? name : std::string(fallback) + std::to_string(id);
}

struct Triangle {
    std::array<Vec3, 3> vertices;
};

void write_stl(const std::filesystem::path &path, const std::vector<Triangle> &triangles) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + path.string());
    char header[80] = "RobotKit MJCF import, link frame, metres";
    out.write(header, sizeof(header));
    const auto count = static_cast<uint32_t>(triangles.size());
    out.write(reinterpret_cast<const char *>(&count), sizeof(count));
    for (const auto &triangle : triangles) {
        const auto &a = triangle.vertices[0], &b = triangle.vertices[1], &c = triangle.vertices[2];
        Vec3 n{(b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]),
               (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
               (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])};
        const double length = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        float values[12];
        for (int axis = 0; axis < 3; ++axis)
            values[axis] = length > 0.0 ? static_cast<float>(n[axis] / length) : 0.0f;
        for (int vertex = 0; vertex < 3; ++vertex)
            for (int axis = 0; axis < 3; ++axis)
                values[3 + vertex * 3 + axis] = static_cast<float>(triangle.vertices[vertex][axis]);
        out.write(reinterpret_cast<const char *>(values), sizeof(values));
        const uint16_t attributes = 0;
        out.write(reinterpret_cast<const char *>(&attributes), sizeof(attributes));
    }
}

struct Summary {
    int links = 0, joints = 0, actuators = 0, shapes = 0, meshes = 0, imus = 0;
    std::vector<std::string> notes;
};

std::string import_model(const mjModel *m, const std::filesystem::path &out_dir, Summary &summary) {
    // Links: every body except the world, in MuJoCo's depth-first order.
    int root = -1;
    for (int body = 1; body < m->nbody; ++body)
        if (m->body_parentid[body] == 0) {
            if (root >= 0) throw std::runtime_error("the model has more than one body under the world");
            root = body;
        }
    if (root < 0) throw std::runtime_error("the model has no bodies");

    // A geom named in an explicit contact pair takes the first such pair's
    // friction and solver settings, which override the geoms' own.
    std::map<int, int> contact_pairs;
    for (int pair = 0; pair < m->npair; ++pair) {
        contact_pairs.emplace(m->pair_geom1[pair], pair);
        contact_pairs.emplace(m->pair_geom2[pair], pair);
    }
    const auto collides = [&](int geom) {
        return m->geom_contype[geom] != 0 || m->geom_conaffinity[geom] != 0 ||
               contact_pairs.count(geom) != 0;
    };
    const auto surface_of = [&](int geom) {
        const auto pair = contact_pairs.find(geom);
        std::array<double, 3> friction;
        double time_constant, damping_ratio;
        int dimensions;
        if (pair != contact_pairs.end()) {
            const auto *values = m->pair_friction + 5 * pair->second; // slide, slide, spin, roll, roll
            friction = {values[0], values[2], values[3]};
            time_constant = m->pair_solref[2 * pair->second];
            damping_ratio = m->pair_solref[2 * pair->second + 1];
            dimensions = m->pair_dim[pair->second];
        } else {
            friction = {m->geom_friction[3 * geom], m->geom_friction[3 * geom + 1],
                        m->geom_friction[3 * geom + 2]};
            time_constant = m->geom_solref[2 * geom];
            damping_ratio = m->geom_solref[2 * geom + 1];
            dimensions = m->geom_condim[geom];
        }
        return "{\"friction\": " + array(friction) + ", \"frictionDimensions\": " +
               std::to_string(dimensions) + ", \"contactTimeConstant\": " + number(time_constant) +
               ", \"contactDampingRatio\": " + number(damping_ratio) + "}";
    };

    bool floating = false;
    std::vector<std::string> links, joints, frames, sensors, actuators;
    std::map<int, std::string> link_ids, joint_ids;
    int armature = 0, damping = 0, friction = 0;
    int damping_ratio_actuators = 0;
    std::filesystem::create_directories(out_dir / "meshes");

    for (int body = 1; body < m->nbody; ++body) {
        const auto body_name = name_of(m, mjOBJ_BODY, body, "body");
        const auto link_id = "link/" + body_name;
        link_ids[body] = link_id;
        if (m->body_mass[body] <= 0.0)
            throw std::runtime_error("body " + body_name + " has no mass");

        // Inertia about the centre of mass, in link axes: R diag(I) R^T.
        const auto iq = from_mujoco(m->body_iquat + 4 * body);
        std::array<double, 9> inertia{};
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    Vec3 axis{0.0, 0.0, 0.0};
                    axis[k] = 1.0;
                    const auto r = rotate(iq, axis);
                    sum += r[row] * m->body_inertia[3 * body + k] * r[column];
                }
                inertia[row * 3 + column] = sum;
            }

        std::vector<std::string> shapes;
        std::vector<Triangle> visual;
        for (int geom = m->body_geomadr[body];
             geom >= 0 && geom < m->body_geomadr[body] + m->body_geomnum[body]; ++geom) {
            const auto type = m->geom_type[geom];
            const auto pos = vec(m->geom_pos + 3 * geom);
            const auto rot = from_mujoco(m->geom_quat + 4 * geom);
            const auto *size = m->geom_size + 3 * geom;
            if (collides(geom)) {
                std::string kind;
                std::vector<double> sizes;
                switch (type) {
                case mjGEOM_BOX: kind = "box"; sizes = {size[0], size[1], size[2]}; break;
                case mjGEOM_SPHERE: kind = "sphere"; sizes = {size[0]}; break;
                case mjGEOM_CAPSULE: kind = "capsule"; sizes = {size[0], size[1]}; break;
                case mjGEOM_CYLINDER: kind = "cylinder"; sizes = {size[0], size[1]}; break;
                default:
                    summary.notes.push_back("skipped colliding " +
                        std::string(type == mjGEOM_MESH ? "mesh" : "non-primitive") + " geom " +
                        name_of(m, mjOBJ_GEOM, geom, "geom") + " on " + body_name);
                    continue;
                }
                shapes.push_back("{\"kind\": " + json_string(kind) + ", \"size\": " + array(sizes) +
                                 ", \"position\": " + array(pos) + ", \"rotation\": " +
                                 array(rot) + ", \"surface\": " + surface_of(geom) + "}");
            } else if (type == mjGEOM_MESH && m->geom_dataid[geom] >= 0) {
                const int mesh = m->geom_dataid[geom];
                const float *vertices = m->mesh_vert + 3 * m->mesh_vertadr[mesh];
                const int *faces = m->mesh_face + 3 * m->mesh_faceadr[mesh];
                for (int face = 0; face < m->mesh_facenum[mesh]; ++face) {
                    Triangle triangle;
                    for (int corner = 0; corner < 3; ++corner) {
                        const float *v = vertices + 3 * faces[3 * face + corner];
                        const auto local = rotate(rot, {v[0], v[1], v[2]});
                        triangle.vertices[corner] = {pos[0] + local[0], pos[1] + local[1],
                                                     pos[2] + local[2]};
                    }
                    visual.push_back(triangle);
                }
            }
        }
        std::string visual_ref = "null";
        if (!visual.empty()) {
            const auto file = "meshes/" + body_name + ".stl";
            write_stl(out_dir / file, visual);
            visual_ref = json_string(file);
            ++summary.meshes;
        }
        summary.shapes += static_cast<int>(shapes.size());
        std::string shape_list = "[";
        for (std::size_t i = 0; i < shapes.size(); ++i) shape_list += (i ? ", " : "") + shapes[i];
        shape_list += "]";
        links.push_back("{\"id\": " + json_string(link_id) + ", \"name\": " + json_string(body_name) +
                        ", \"mass\": " + number(m->body_mass[body]) + ", \"centerOfMass\": " +
                        array(vec(m->body_ipos + 3 * body)) + ", \"inertiaTensor\": " +
                        array(inertia) + ", \"visualGeometry\": " + visual_ref +
                        ", \"collisionGeometry\": null, \"collisionShapes\": " + shape_list + "}");
        ++summary.links;

        // The joint that attaches this body to its parent.
        const int joint_count = m->body_jntnum[body];
        if (body == root) {
            if (joint_count == 1 && m->jnt_type[m->body_jntadr[body]] == mjJNT_FREE)
                floating = true;
            else if (joint_count != 0)
                throw std::runtime_error("root body " + body_name +
                                         " must have a free joint or none");
            continue;
        }
        if (joint_count > 1)
            throw std::runtime_error("body " + body_name + " has more than one joint");
        const auto parent_pos = vec(m->body_pos + 3 * body);
        const auto parent_rot = from_mujoco(m->body_quat + 4 * body);
        Vec3 anchor{0.0, 0.0, 0.0}, axis{0.0, 0.0, 1.0};
        std::string type = "fixed", joint_name = body_name + "_fixed";
        double lower = 0.0, upper = 0.0, effort = 0.0;
        double joint_armature = 0.0, joint_damping = 0.0, joint_friction = 0.0;
        if (joint_count == 1) {
            const int joint = m->body_jntadr[body];
            const auto joint_type = m->jnt_type[joint];
            if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE)
                throw std::runtime_error("body " + body_name + " has an unsupported joint type");
            joint_name = name_of(m, mjOBJ_JOINT, joint, "joint");
            anchor = vec(m->jnt_pos + 3 * joint);
            axis = vec(m->jnt_axis + 3 * joint);
            const bool limited = m->jnt_limited[joint] != 0;
            type = joint_type == mjJNT_SLIDE ? "prismatic" : limited ? "revolute" : "continuous";
            if (limited) {
                lower = m->jnt_range[2 * joint];
                upper = m->jnt_range[2 * joint + 1];
            }
            if (m->jnt_actfrclimited[joint])
                effort = std::max(std::abs(m->jnt_actfrcrange[2 * joint]),
                                  std::abs(m->jnt_actfrcrange[2 * joint + 1]));
            const int dof = m->jnt_dofadr[joint];
            joint_armature = m->dof_armature[dof];
            joint_damping = m->dof_damping[dof];
            joint_friction = m->dof_frictionloss[dof];
            armature += joint_armature != 0.0;
            damping += joint_damping != 0.0;
            friction += joint_friction != 0.0;
            joint_ids[joint] = "joint/" + joint_name;
        }
        // The joint frame sits at the joint anchor with the child body's axes:
        // parent_T_joint = body pose * T(anchor), child_T_joint = T(anchor).
        const auto offset = rotate(parent_rot, anchor);
        const Vec3 joint_pos{parent_pos[0] + offset[0], parent_pos[1] + offset[1],
                             parent_pos[2] + offset[2]};
        joints.push_back(
            "{\"id\": " + json_string("joint/" + joint_name) + ", \"name\": " + json_string(joint_name) +
            ", \"type\": " + json_string(type) + ", \"parentLink\": " +
            json_string(link_ids.at(m->body_parentid[body])) + ", \"childLink\": " + json_string(link_id) +
            ", \"limits\": {\"lower\": " + number(lower) + ", \"upper\": " + number(upper) +
            ", \"velocity\": 0, \"effort\": " + number(effort) + ", \"maxAcceleration\": 0}" +
            ", \"parentFramePosition\": " + array(joint_pos) + ", \"parentFrameRotation\": " +
            array(parent_rot) + ", \"childFramePosition\": " + array(anchor) +
            ", \"childFrameRotation\": [0, 0, 0, 1], \"axis\": " + array(axis) +
            ", \"dynamics\": {\"armature\": " + number(joint_armature) + ", \"damping\": " +
            number(joint_damping) + ", \"frictionLoss\": " + number(joint_friction) + "}}");
        ++summary.joints;
    }

    for (int actuator = 0; actuator < m->nu; ++actuator) {
        const auto name = name_of(m, mjOBJ_ACTUATOR, actuator, "actuator");
        const int joint = m->actuator_trnid[2 * actuator];
        if (m->actuator_trntype[actuator] != mjTRN_JOINT || !joint_ids.count(joint)) {
            summary.notes.push_back("skipped actuator " + name + " without a single-joint transmission");
            continue;
        }
        const double gear = m->actuator_gear[6 * actuator];
        double effort = 0.0;
        if (m->actuator_forcelimited[actuator])
            effort = std::max(std::abs(m->actuator_forcerange[2 * actuator]),
                              std::abs(m->actuator_forcerange[2 * actuator + 1]));
        else if (m->jnt_actfrclimited[joint])
            effort = std::max(std::abs(m->jnt_actfrcrange[2 * joint]),
                              std::abs(m->jnt_actfrcrange[2 * joint + 1])) / std::abs(gear);
        // A position servo is gain kp with an affine bias of -kp q - kv qdot.
        // A positive bias[2] is a damping ratio that MuJoCo resolves at run
        // time from the joint's inertia, which a fixed gain cannot express.
        double stiffness = 0.0, servo_damping = 0.0;
        const auto *gain = m->actuator_gainprm + mjNGAIN * actuator;
        const auto *bias = m->actuator_biasprm + mjNBIAS * actuator;
        if (m->actuator_gaintype[actuator] == mjGAIN_FIXED &&
            m->actuator_biastype[actuator] == mjBIAS_AFFINE && gain[0] > 0.0 &&
            std::abs(bias[1] + gain[0]) < 1e-12 * gain[0]) {
            stiffness = gain[0];
            if (bias[2] <= 0.0) servo_damping = -bias[2];
            else ++damping_ratio_actuators;
        }
        actuators.push_back("{\"id\": " + json_string("actuator/" + name) + ", \"maxEffort\": " +
                            number(effort) + ", \"maxRate\": 0, \"servoStiffness\": " +
                            number(stiffness) + ", \"servoDamping\": " + number(servo_damping) +
                            ", \"transmission\": " +
                            "{\"kind\": \"simple\", \"jointId\": " + json_string(joint_ids.at(joint)) +
                            ", \"ratio\": " + number(gear) + ", \"offset\": 0}}");
        ++summary.actuators;
    }

    std::set<int> imu_sites;
    for (int sensor = 0; sensor < m->nsensor; ++sensor) {
        const auto type = m->sensor_type[sensor];
        if ((type == mjSENS_ACCELEROMETER || type == mjSENS_GYRO || type == mjSENS_FRAMEQUAT) &&
            m->sensor_objtype[sensor] == mjOBJ_SITE)
            imu_sites.insert(m->sensor_objid[sensor]);
    }
    for (const int site : imu_sites) {
        const auto name = name_of(m, mjOBJ_SITE, site, "site");
        frames.push_back("{\"id\": " + json_string("frame/" + name) + ", \"name\": " + json_string(name) +
                         ", \"link\": " + json_string(link_ids.at(m->site_bodyid[site])) +
                         ", \"position\": " + array(vec(m->site_pos + 3 * site)) +
                         ", \"rotation\": " + array(from_mujoco(m->site_quat + 4 * site)) + "}");
        sensors.push_back("{\"id\": " + json_string("sensor/" + name) + ", \"name\": " + json_string(name) +
                          ", \"kind\": \"imu\", \"updateRate\": 0, \"frame\": " +
                          json_string("frame/" + name) + ", \"rayCount\": 8, \"maxRange\": 10, " +
                          "\"startAngleRadians\": 0, \"fieldOfViewRadians\": " +
                          number(2.0 * M_PI) + ", \"noiseStddev\": 0, \"noiseSeed\": 1}");
        ++summary.imus;
    }

    summary.notes.push_back("joint dynamics: armature on " + std::to_string(armature) +
                            ", damping on " + std::to_string(damping) + ", friction loss on " +
                            std::to_string(friction) + " joints");
    if (damping_ratio_actuators)
        summary.notes.push_back(std::to_string(damping_ratio_actuators) +
                                " actuators set a damping ratio, not stored; servoDamping is 0");
    if (m->npair)
        summary.notes.push_back(std::to_string(m->npair) + " explicit contact pairs approximated: " +
                                "their geoms collide with everything, using the pair's surface");
    const char *integrators[] = {"euler", "rk4", "implicit", "implicitfast"};
    summary.notes.push_back("scene solver: timestep " + number(m->opt.timestep) + ", integrator " +
                            (m->opt.integrator >= 0 && m->opt.integrator < 4
                                 ? integrators[m->opt.integrator] : "other") +
                            ", cone " + (m->opt.cone == mjCONE_ELLIPTIC ? "elliptic" : "pyramidal"));
    if (floating) {
        const auto root_pos = vec(m->body_pos + 3 * root);
        summary.notes.push_back("floating base: authored root position " + array(root_pos));
    }

    const auto join = [](const std::vector<std::string> &items) {
        std::string out = "[";
        for (std::size_t i = 0; i < items.size(); ++i) out += (i ? ",\n    " : "\n    ") + items[i];
        return out + (items.empty() ? "]" : "\n  ]");
    };
    std::string name = m->names; // The first name is the model's.
    if (name.empty()) name = "mjcf-robot";
    return "{\n  \"schemaVersion\": 6,\n  \"name\": " + json_string(name) +
           ",\n  \"collisionApproximation\": \"none\",\n  \"floatingBase\": " +
           (floating ? "true" : "false") + ",\n  \"links\": " + join(links) +
           ",\n  \"joints\": " + join(joints) + ",\n  \"actuators\": " + join(actuators) +
           ",\n  \"couplings\": [],\n  \"frames\": " + join(frames) + ",\n  \"sensors\": " +
           join(sensors) + ",\n  \"mobileBase\": null,\n  \"forkMechanism\": null\n}\n";
}

} // namespace

int main(int argc, char **argv) {
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s <model.xml> <output-directory>\n", argv[0]);
        return 2;
    }
    char error[1024] = "";
    mjModel *model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
    if (!model) {
        std::fprintf(stderr, "mjcf_import: %s\n", error);
        return 1;
    }
    try {
        const std::filesystem::path out_dir(argv[2]);
        Summary summary;
        const auto json = import_model(model, out_dir, summary);
        std::ofstream out(out_dir / "robot.json");
        out << json;
        if (!out) throw std::runtime_error("cannot write robot.json");
        std::printf("imported %d links, %d joints, %d actuators, %d collision shapes, "
                    "%d visual meshes, %d IMUs\n", summary.links, summary.joints,
                    summary.actuators, summary.shapes, summary.meshes, summary.imus);
        for (const auto &note : summary.notes) std::printf("note: %s\n", note.c_str());
    } catch (const std::exception &failure) {
        std::fprintf(stderr, "mjcf_import: %s\n", failure.what());
        mj_deleteModel(model);
        return 1;
    }
    mj_deleteModel(model);
    return 0;
}

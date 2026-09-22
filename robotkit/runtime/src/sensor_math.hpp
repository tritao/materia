#ifndef ROBOTKIT_SENSOR_MATH_HPP
#define ROBOTKIT_SENSOR_MATH_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robotkit::sensors {

inline void multiply(const double a[4], const double b[4], double out[4]) {
    out[0] = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    out[1] = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
    out[2] = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
    out[3] = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
}

inline double gaussian(uint32_t &seed) {
    auto uniform = [&]() {
        seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
        return (static_cast<double>(seed) + 0.5) / 4294967296.0;
    };
    const double u = uniform(), v = uniform();
    return std::sqrt(-2.0 * std::log(u)) * std::cos(6.283185307179586 * v);
}

// Unit xyzw quaternion. Inverse rotates world vectors into the sensor frame.
inline void rotate(const double q[4], const double v[3], double out[3], bool inverse = false) {
    const double sign = inverse ? -1.0 : 1.0;
    const double x = sign*q[0], y = sign*q[1], z = sign*q[2];
    const double t[3] = {2*(y*v[2]-z*v[1]), 2*(z*v[0]-x*v[2]), 2*(x*v[1]-y*v[0])};
    out[0] = v[0] + q[3]*t[0] + y*t[2] - z*t[1];
    out[1] = v[1] + q[3]*t[1] + z*t[0] - x*t[2];
    out[2] = v[2] + q[3]*t[2] + x*t[1] - y*t[0];
}

inline double ray_box(const double origin[3], const double direction[3],
                      const double position[3], const double rotation[4],
                      const double extents[3], double maximum) {
    double delta[3], local_origin[3], local_direction[3];
    for (int i = 0; i < 3; ++i) delta[i] = origin[i] - position[i];
    rotate(rotation, delta, local_origin, true);
    rotate(rotation, direction, local_direction, true);
    double near = 0.0, far = maximum;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(local_direction[i]) < 1e-12) {
            if (std::abs(local_origin[i]) > extents[i]) return maximum;
        } else {
            double a = (-extents[i] - local_origin[i]) / local_direction[i];
            double b = (extents[i] - local_origin[i]) / local_direction[i];
            if (a > b) std::swap(a, b);
            near = std::max(near, a);
            far = std::min(far, b);
            if (near > far) return maximum;
        }
    }
    return near;
}

inline void imu(const double rotation[4], const double angular_velocity[3],
                const double acceleration[3], const double gravity[3], double out[6]) {
    double specific_force[3];
    for (int i = 0; i < 3; ++i) specific_force[i] = acceleration[i] - gravity[i];
    rotate(rotation, angular_velocity, out, true);
    rotate(rotation, specific_force, out + 3, true);
}
}
#endif

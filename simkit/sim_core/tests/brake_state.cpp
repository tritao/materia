#include "../src/BrakeState.hpp"

#include <cassert>
#include <limits>

int main() {
    nksim::BrakeState brake({4.0, 0.05, 0.03});
    assert(!brake.released());
    assert(brake.friction_effort() == 4.0);
    brake.set_release_coil(true);
    brake.advance(0.049);
    assert(!brake.released());
    // Repeated controller outputs must not restart the physical delay.
    brake.set_release_coil(true);
    brake.advance(0.002);
    assert(brake.released());
    assert(brake.friction_effort() == 0.0);
    brake.set_release_coil(false);
    brake.advance(0.029);
    assert(brake.released());
    brake.advance(0.002);
    assert(!brake.released());
    assert(brake.friction_effort() == 4.0);
    // A cancelled release cannot make the brake release later.
    brake.set_release_coil(true);
    brake.advance(0.02);
    brake.set_release_coil(false);
    brake.advance(1.0);
    assert(!brake.released());
    // Restoring power during application keeps an already released contact open.
    brake.set_release_coil(true);
    brake.advance(0.051);
    brake.set_release_coil(false);
    brake.advance(0.01);
    brake.set_release_coil(true);
    brake.advance(0.1);
    assert(brake.released());

    nksim::BrakeState immediate({2.0, 0.0, 0.0});
    immediate.set_release_coil(true);
    assert(immediate.released());
    immediate.set_release_coil(false);
    assert(!immediate.released());

    bool rejected = false;
    try { nksim::BrakeState invalid({-1.0, 0.0, 0.0}); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
    rejected = false;
    try { brake.advance(std::numeric_limits<double>::quiet_NaN()); }
    catch (const std::invalid_argument &) { rejected = true; }
    assert(rejected);
}

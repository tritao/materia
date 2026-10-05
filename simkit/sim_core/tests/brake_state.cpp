#include "../src/BrakeState.hpp"

#include <cassert>
#include <limits>

int main() {
    auto state = nksim::BrakeState::create({4.0, 0.05, 0.03});
    assert(state);
    auto &brake = *state;
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

    auto immediate_state = nksim::BrakeState::create({2.0, 0.0, 0.0});
    assert(immediate_state);
    auto &immediate = *immediate_state;
    immediate.set_release_coil(true);
    assert(immediate.released());
    immediate.set_release_coil(false);
    assert(!immediate.released());

    assert(!nksim::BrakeState::create({-1.0, 0.0, 0.0}));
    assert(!brake.advance(std::numeric_limits<double>::quiet_NaN()));
}

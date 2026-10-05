#include "../include/brake_release_interlock.hpp"
#include <cassert>

using namespace robotkit::internal;

int main() {
    assert(brake_feedback_cadence_valid(40.0, 10'000'000, 30'000'000));
    assert(!brake_feedback_cadence_valid(40.0, 20'000'000, 30'000'000));
    assert(!brake_feedback_cadence_valid(0.0, 50'000'000, 30'000'000));
    BrakeReleaseInterlock interlock(250, 30);
    interlock.update(BrakePowerMode::power_off, 0, {1, 0, false});
    assert(!interlock.release_coil() && !interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 10, {2, 10, true});
    assert(interlock.release_coil() && !interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 20, {2, 10, true});
    assert(!interlock.motion_ready()); // old true cannot acknowledge this request
    interlock.update(BrakePowerMode::enabled, 30, {3, 30, false});
    assert(!interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 60, {4, 60, true});
    assert(interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 80, {4, 60, true}); // normal hold preserves release
    assert(interlock.release_coil() && interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 91, {4, 60, true});
    assert(interlock.fault() == BrakeReleaseFault::contact_lost);
    assert(!interlock.release_coil() && !interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 100, {5, 100, true});
    assert(!interlock.motion_ready()); // fault latches
    interlock.reset();
    interlock.update(BrakePowerMode::enabled, 110, {5, 100, true});
    interlock.update(BrakePowerMode::enabled, 120, {5, 100, true});
    assert(!interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 130, {6, 130, true});
    assert(interlock.motion_ready());
    interlock.update(BrakePowerMode::power_off, 140, {7, 140, true}); // emergency stop drops coil
    assert(!interlock.release_coil() && !interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 150, {7, 140, true});
    interlock.update(BrakePowerMode::enabled, 160, {8, 160, true});
    assert(interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 170, {9, 170, false});
    assert(interlock.fault() == BrakeReleaseFault::contact_lost);
    interlock.reset();
    interlock.update(BrakePowerMode::enabled, 200, {});
    interlock.update(BrakePowerMode::enabled, 449, {});
    assert(interlock.fault() == BrakeReleaseFault::none);
    interlock.update(BrakePowerMode::enabled, 450, {});
    assert(interlock.fault() == BrakeReleaseFault::release_timeout);
    interlock.reset();
    interlock.update(BrakePowerMode::enabled, 0, {});
    interlock.update(BrakePowerMode::enabled, 250, {1, 250, true});
    assert(interlock.fault() == BrakeReleaseFault::release_timeout); // late acknowledgement
    interlock.reset();
    interlock.update(BrakePowerMode::enabled, 500, {});
    interlock.update(BrakePowerMode::enabled, 510, {1, 520, true}); // future timestamp
    assert(!interlock.motion_ready());
    interlock.update(BrakePowerMode::enabled, 509, {});
    assert(interlock.fault() == BrakeReleaseFault::clock_reversed);
    interlock.reset();
    interlock.update(BrakePowerMode::enabled, 1000, {1, 1000, false});
    interlock.update(BrakePowerMode::enabled, 1010, {2, 1010, true});
    assert(interlock.motion_ready());
    interlock.update(BrakePowerMode::controlled_stop, 1020, {3, 1020, false});
    assert(!interlock.release_coil() && interlock.holding_required()); // pre-coil sample is not acknowledgement
    const auto staged = interlock;
    interlock.update(BrakePowerMode::controlled_stop, 1030, {4, 1030, true});
    assert(interlock.holding_required());
    interlock.update(BrakePowerMode::controlled_stop, 1040, {5, 1040, false});
    assert(!interlock.holding_required());
    interlock = staged;
    assert(interlock.holding_required()); // tick rollback restores pending engagement
    interlock.update(BrakePowerMode::controlled_stop, 1270, {});
    assert(interlock.fault() == BrakeReleaseFault::engagement_timeout && interlock.holding_required());
    interlock.update(BrakePowerMode::controlled_stop, 1280, {6, 1280, false});
    assert(!interlock.holding_required()); // confirmation still hands off after the latched diagnostic
    interlock.reset();
    interlock.update(BrakePowerMode::enabled, 1300, {});
    interlock.update(BrakePowerMode::controlled_stop, 1310, {});
    assert(interlock.holding_required());
    interlock.update(BrakePowerMode::power_off, 1320, {});
    assert(!interlock.release_coil() && !interlock.holding_required()); // no motor holding without power
    interlock.reset();
    interlock.update(BrakePowerMode::enabled, 1400, {});
    interlock.update(BrakePowerMode::controlled_stop, 1410, {});
    assert(interlock.holding_required());
    interlock.update(BrakePowerMode::power_off, 1300, {}); // even a reversed clock cannot retain unpowered holding
    assert(!interlock.release_coil() && !interlock.holding_required());
    BrakeReleaseInterlock invalid(10, 20);
    assert(invalid.fault() == BrakeReleaseFault::invalid_policy);
    invalid.reset();
    assert(invalid.fault() == BrakeReleaseFault::invalid_policy);
}

#include "../include/brake_release_interlock.hpp"
#include <cassert>

using namespace robotkit::internal;

int main() {
    assert(brake_feedback_cadence_valid(40.0, 10'000'000, 30'000'000));
    assert(!brake_feedback_cadence_valid(40.0, 20'000'000, 30'000'000));
    assert(!brake_feedback_cadence_valid(0.0, 50'000'000, 30'000'000));
    BrakeReleaseInterlock interlock(250, 30);
    interlock.update(false, 0, {1, 0, false});
    assert(!interlock.release_coil() && !interlock.motion_ready());
    interlock.update(true, 10, {2, 10, true});
    assert(interlock.release_coil() && !interlock.motion_ready());
    interlock.update(true, 20, {2, 10, true});
    assert(!interlock.motion_ready()); // old true cannot acknowledge this request
    interlock.update(true, 30, {3, 30, false});
    assert(!interlock.motion_ready());
    interlock.update(true, 60, {4, 60, true});
    assert(interlock.motion_ready());
    interlock.update(true, 80, {4, 60, true}); // normal hold preserves release
    assert(interlock.release_coil() && interlock.motion_ready());
    interlock.update(true, 91, {4, 60, true});
    assert(interlock.fault() == BrakeReleaseFault::contact_lost);
    assert(!interlock.release_coil() && !interlock.motion_ready());
    interlock.update(true, 100, {5, 100, true});
    assert(!interlock.motion_ready()); // fault latches
    interlock.reset();
    interlock.update(true, 110, {5, 100, true});
    interlock.update(true, 120, {5, 100, true});
    assert(!interlock.motion_ready());
    interlock.update(true, 130, {6, 130, true});
    assert(interlock.motion_ready());
    interlock.update(false, 140, {7, 140, true}); // emergency stop drops coil
    assert(!interlock.release_coil() && !interlock.motion_ready());
    interlock.update(true, 150, {7, 140, true});
    interlock.update(true, 160, {8, 160, true});
    assert(interlock.motion_ready());
    interlock.update(true, 170, {9, 170, false});
    assert(interlock.fault() == BrakeReleaseFault::contact_lost);
    interlock.reset();
    interlock.update(true, 200, {});
    interlock.update(true, 449, {});
    assert(interlock.fault() == BrakeReleaseFault::none);
    interlock.update(true, 450, {});
    assert(interlock.fault() == BrakeReleaseFault::release_timeout);
    interlock.reset();
    interlock.update(true, 0, {});
    interlock.update(true, 250, {1, 250, true});
    assert(interlock.fault() == BrakeReleaseFault::release_timeout); // late acknowledgement
    interlock.reset();
    interlock.update(true, 500, {});
    interlock.update(true, 510, {1, 520, true}); // future timestamp
    assert(!interlock.motion_ready());
    interlock.update(true, 509, {});
    assert(interlock.fault() == BrakeReleaseFault::clock_reversed);
    BrakeReleaseInterlock invalid(10, 20);
    assert(invalid.fault() == BrakeReleaseFault::invalid_policy);
    invalid.reset();
    assert(invalid.fault() == BrakeReleaseFault::invalid_policy);
}

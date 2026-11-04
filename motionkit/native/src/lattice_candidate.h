#pragma once
#include "motionkit.h"

namespace motionkit {
/** Internal full record used by indexed and coarse ladder search. */
struct LadderCandidate {
    uint32_t struct_size MK_STRUCT_SIZE;
    double joints[MK_MAX_JOINTS];
    int32_t wraps[MK_MAX_JOINTS];
    uint32_t external_coordinates[MK_MAX_JOINTS];
    uint32_t roll_index;
    uint32_t tilt_index;
    uint32_t azimuth_index;
    uint32_t branch;
    uint32_t singular;
};
}

#include "cadkit.h"

int cadkit_c_header_smoke(void) {
    cad_vec3 point = {0.0, 0.0, 0.0};
    cad_bounds bounds = {point, point};
    cad_shape shape = 0;
    return (shape == 0 && bounds.min.x == 0.0) ? 0 : 1;
}


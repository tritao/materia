// The four macros proxsuite's own CMake (jrl-cmakemodules) would generate for
// the pinned v0.7.3; see THIRD_PARTY.md. Only the header-only dense solver is used.
#pragma once
#define PROXSUITE_MAJOR_VERSION 0
#define PROXSUITE_MINOR_VERSION 7
#define PROXSUITE_PATCH_VERSION 3
#define PROXSUITE_VERSION_AT_LEAST(major, minor, patch)                        \
  (PROXSUITE_MAJOR_VERSION > (major) ||                                        \
   (PROXSUITE_MAJOR_VERSION == (major) &&                                      \
    (PROXSUITE_MINOR_VERSION > (minor) ||                                      \
     (PROXSUITE_MINOR_VERSION == (minor) && PROXSUITE_PATCH_VERSION >= (patch)))))

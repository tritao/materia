#ifndef VISIONKIT_H
#define VISIONKIT_H

#include <stdint.h>

#if defined(_WIN32)
#  if defined(VK_BUILDING_LIBRARY)
#    define VK_API __declspec(dllexport)
#  else
#    define VK_API __declspec(dllimport)
#  endif
#else
#  define VK_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum vk_result {
    VK_OK = 0,
    VK_ERROR_INVALID_ARGUMENT = -1,
    VK_ERROR_INVALID_HANDLE = -2,
    VK_ERROR_OUT_OF_MEMORY = -3,
    VK_ERROR_BACKEND = -4,
    VK_ERROR_UNSUPPORTED = -5
} vk_result;

/** Version of the VisionKit C ABI. */
VK_API uint32_t vk_version(void);

#ifdef __cplusplus
}
#endif

#endif

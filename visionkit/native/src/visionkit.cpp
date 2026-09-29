#include "visionkit.h"
#include <opencv2/core.hpp>

extern "C" VK_API uint32_t vk_version(void) {
    static const bool configured = [] {
        cv::setNumThreads(VK_OPENCV_THREADS);
        return true;
    }();
    (void)configured;
    return 1;
}

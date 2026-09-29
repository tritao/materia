#include "visionkit.h"
#include <cassert>

int main() {
    assert(vk_version() == 1);
    assert(VK_ERROR_INVALID_ARGUMENT < VK_OK);
}

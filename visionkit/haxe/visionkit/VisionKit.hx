package visionkit;

import VisionKitNative;

class VisionKit {
    public static function version():Int {
        return VisionKitNative.vk_version();
    }
}

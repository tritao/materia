package tests;

import visionkit.VisionKit;

class VisionKitSmokeTests {
    static function main():Void {
        if (VisionKit.version() != 1) throw "unexpected VisionKit ABI version";
        trace("VisionKit smoke passed");
    }
}

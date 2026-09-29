package tests;

import visionkit.VisionKit;
import visionkit.CameraModel;
import visionkit.CameraCalibration;
import visionkit.ImageView;
import visionkit.UndistortMap;
import haxe.io.Bytes;

class VisionKitSmokeTests {
    static function main():Void {
        if (VisionKit.version() != 1) throw "unexpected VisionKit ABI version";
        var model = new CameraModel(64, 48, 40, 40, 32, 24,
          1, -0.1, 0.01, 0.002, -0.001, 0.003);
        var centre = model.project([{x: 1.0, y: 0.0, z: 0.0}])[0];
        if (centre.x < 31.9 || centre.x > 32.1 || centre.y < 23.9 || centre.y > 24.1)
          throw "projection failed";
        var ray = model.unproject([centre])[0];
        if (ray.x < 0.999 || ray.y < -0.001 || ray.y > 0.001 ||
            ray.z < -0.001 || ray.z > 0.001) throw "unprojection failed";
        var calibration = new CameraCalibration(model, 0.2,
          "2026-09-29T00:00:00Z", "chessboard 9x6", "test fixture");
        var decoded = CameraCalibration.fromJson(calibration.toJson());
        if (decoded.model.fx != model.fx || decoded.model.k3 != model.k3 ||
            decoded.rmsReprojectionError != calibration.rmsReprojectionError ||
            decoded.calibrationTime != calibration.calibrationTime ||
            decoded.boardDescription != calibration.boardDescription ||
            decoded.source != calibration.source)
          throw "calibration JSON round-trip failed";
        var rejected = false;
        try CameraCalibration.fromJson('{"version":2,"cameraModel":{}}')
          catch (_:Dynamic) rejected = true;
        if (!rejected) throw "unsupported calibration version accepted";
        var map = new UndistortMap(new CameraModel(64, 48, 40, 40, 32, 24));
        var input = Bytes.alloc(64 * 48); input.set(24 * 64 + 32, 173);
        var output = Bytes.alloc(64 * 48);
        map.remap(new ImageView(64, 48, 64, 2, input),
          new ImageView(64, 48, 64, 2, output));
        if (output.get(24 * 64 + 32) != 173) throw "image remap failed";
        map.dispose();
        trace("VisionKit smoke passed");
    }
}

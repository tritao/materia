package visionkit;

import haxe.Json;

/** Deployment calibration data, independent of RobotKit. */
class CameraCalibration {
  public static inline final VERSION = 1;

  public final model:CameraModel;
  public final rmsReprojectionError:Float;
  public final calibrationTime:String;
  public final boardDescription:String;
  public final source:String;

  public function new(model:CameraModel, rmsReprojectionError:Float,
      calibrationTime:String, boardDescription:String, source:String) {
    if (model == null || !Math.isFinite(rmsReprojectionError) || rmsReprojectionError < 0 ||
        calibrationTime == null || calibrationTime == "" ||
        boardDescription == null || boardDescription == "" ||
        source == null || source == "") throw "Invalid camera calibration";
    this.model = model;
    this.rmsReprojectionError = rmsReprojectionError;
    this.calibrationTime = calibrationTime;
    this.boardDescription = boardDescription;
    this.source = source;
  }

  public function toJson():String {
    return Json.stringify({
      version: VERSION,
      cameraModel: {
        width: model.width, height: model.height,
        fx: model.fx, fy: model.fy, cx: model.cx, cy: model.cy,
        distortionModel: model.distortionModel,
        k1: model.k1, k2: model.k2, p1: model.p1, p2: model.p2, k3: model.k3
      },
      rmsReprojectionError: rmsReprojectionError,
      calibrationTime: calibrationTime,
      boardDescription: boardDescription,
      source: source
    });
  }

  public static function fromJson(text:String):CameraCalibration {
    var root:Dynamic = Json.parse(text);
    if (root == null || Reflect.field(root, "version") != VERSION)
      throw "Unsupported camera calibration version";
    var m:Dynamic = Reflect.field(root, "cameraModel");
    if (m == null) throw "Missing camera model";
    for (field in ["width", "height", "fx", "fy", "cx", "cy",
        "distortionModel", "k1", "k2", "p1", "p2", "k3"])
      if (!Reflect.hasField(m, field)) throw 'Missing camera model field $field';
    for (field in ["rmsReprojectionError", "calibrationTime",
        "boardDescription", "source"])
      if (!Reflect.hasField(root, field)) throw 'Missing calibration field $field';
    var model = new CameraModel(Reflect.field(m, "width"), Reflect.field(m, "height"),
      Reflect.field(m, "fx"), Reflect.field(m, "fy"), Reflect.field(m, "cx"),
      Reflect.field(m, "cy"), Reflect.field(m, "distortionModel"),
      Reflect.field(m, "k1"), Reflect.field(m, "k2"), Reflect.field(m, "p1"),
      Reflect.field(m, "p2"), Reflect.field(m, "k3"));
    return new CameraCalibration(model, Reflect.field(root, "rmsReprojectionError"),
      Reflect.field(root, "calibrationTime"), Reflect.field(root, "boardDescription"),
      Reflect.field(root, "source"));
  }
}

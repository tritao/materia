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

  static function number(value:Dynamic, field:String):Float {
    var fieldValue:Dynamic = Reflect.field(value, field);
    if ((!Std.isOfType(fieldValue, Int) && !Std.isOfType(fieldValue, Float)) ||
        !Math.isFinite(fieldValue)) throw 'Invalid numeric calibration field $field';
    return fieldValue;
  }

  static function integer(value:Dynamic, field:String):Int {
    var fieldValue:Dynamic = Reflect.field(value, field);
    if (!Std.isOfType(fieldValue, Int)) throw 'Invalid integer calibration field $field';
    return fieldValue;
  }

  static function string(value:Dynamic, field:String):String {
    var fieldValue:Dynamic = Reflect.field(value, field);
    if (!Std.isOfType(fieldValue, String) || fieldValue == "")
      throw 'Invalid text calibration field $field';
    return fieldValue;
  }

  public static function fromJson(text:String):CameraCalibration {
    var root:Dynamic = Json.parse(text);
    if (root == null || Std.isOfType(root, Array) || Reflect.field(root, "version") != VERSION)
      throw "Unsupported camera calibration version";
    var m:Dynamic = Reflect.field(root, "cameraModel");
    if (m == null || Std.isOfType(m, Array)) throw "Missing camera model";
    var model = new CameraModel(integer(m, "width"), integer(m, "height"),
      number(m, "fx"), number(m, "fy"), number(m, "cx"), number(m, "cy"),
      integer(m, "distortionModel"), number(m, "k1"), number(m, "k2"),
      number(m, "p1"), number(m, "p2"), number(m, "k3"));
    return new CameraCalibration(model, number(root, "rmsReprojectionError"),
      string(root, "calibrationTime"), string(root, "boardDescription"),
      string(root, "source"));
  }
}

package materia.project;

import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Nominal polygonal CAD patches, including fixtures; lengths are metres in the named work frame. */
typedef SceneContactFace = {
  var member:String;
  var face:String;
  var target:Bool;
  var normal:Array<Float>;
  var centre:Array<Float>;
  var chords:Array<Array<Array<Float>>>;
}

/** Contact registration after parking. Nominal is assembly_T_work; uncertainty rotates about the chassis origin. */
typedef SceneContactWork = {
  var frame:String;
  var nominal:AssemblyFrame;
  var translation:Array<Float>;
  var rotation:Array<Float>;
  var measurementError:Float;
  var contactOffset:Float;
  var faces:Array<SceneContactFace>;
}

/** Strict saved-data boundary; execution and uncertainty policies belong to ProcessKit. */
class SceneContactRegistration {
  static function fail():Void throw "Scene artifact contact registration is invalid";
  static function array(value:Dynamic):Array<Dynamic> {
    if (!Std.isOfType(value, Array)) fail();
    return cast value;
  }
  static function number(value:Dynamic):Float {
    if (!Std.isOfType(value, Float) && !Std.isOfType(value, Int)) fail();
    return cast value;
  }
  static function text(value:Dynamic):String {
    if (!Std.isOfType(value, String)) fail();
    return cast value;
  }
  static function vector(value:Dynamic):Array<Float> {
    var values = array(value);
    if (values.length != 3) fail();
    return [for (item in values) number(item)];
  }
  public static function decode(raw:Dynamic):SceneContactWork {
    if (raw == null) fail();
    var pose:Dynamic = Reflect.field(raw, "nominal");
    if (pose == null) fail();
    var nominal:AssemblyFrame = {x: number(Reflect.field(pose, "x")), y: number(Reflect.field(pose, "y")),
      z: number(Reflect.field(pose, "z")), qx: number(Reflect.field(pose, "qx")), qy: number(Reflect.field(pose, "qy")),
      qz: number(Reflect.field(pose, "qz")), qw: number(Reflect.field(pose, "qw"))};
    var faces:Array<SceneContactFace> = [for (face in array(Reflect.field(raw, "faces"))) {
      var target:Dynamic = Reflect.field(face, "target");
      if (!Std.isOfType(target, Bool)) fail();
      {member: text(Reflect.field(face, "member")), face: text(Reflect.field(face, "face")), target: cast target,
        normal: vector(Reflect.field(face, "normal")), centre: vector(Reflect.field(face, "centre")),
        chords: [for (edge in array(Reflect.field(face, "chords"))) [for (point in array(edge)) vector(point)]]};
    }];
    return {frame: text(Reflect.field(raw, "frame")), nominal: nominal,
      translation: vector(Reflect.field(raw, "translation")), rotation: vector(Reflect.field(raw, "rotation")),
      measurementError: number(Reflect.field(raw, "measurementError")), contactOffset: number(Reflect.field(raw, "contactOffset")), faces: faces};
  }
  static function finiteVector(value:Array<Float>):Bool {
    if (value == null || value.length != 3) return false;
    for (item in value) if (!Math.isFinite(item)) return false;
    return true;
  }
  static function square(value:Array<Float>):Float return value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
  public static function validate(work:SceneContactWork, occurrences:Array<String>):Void {
    if (work == null || occurrences == null || occurrences.indexOf(work.frame) < 0 || work.nominal == null ||
        !finiteVector(work.translation) || !finiteVector(work.rotation) || !Math.isFinite(work.measurementError) ||
        !(work.measurementError > 0) || !Math.isFinite(work.contactOffset) || work.contactOffset < 0 || work.contactOffset > 0.002 ||
        work.faces == null || work.faces.length < 3 || work.faces.length > 256) fail();
    for (i in 0...3) if (work.translation[i] < 0 || work.rotation[i] < 0 || work.rotation[i] >= Math.PI / 2) fail();
    var pose = work.nominal;
    for (value in [pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw]) if (!Math.isFinite(value)) fail();
    if (Math.abs(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw - 1) > 1e-8) fail();
    var seen:Map<String, Bool> = new Map();
    var normals:Array<Array<Float>> = [];
    for (face in work.faces) {
      if (face == null || occurrences.indexOf(face.member) < 0 || face.face == null || face.face.length == 0 ||
          !finiteVector(face.normal) || Math.abs(square(face.normal) - 1) > 1e-8 || !finiteVector(face.centre) ||
          face.chords == null || face.chords.length < 3 || face.chords.length > 512) fail();
      var key = face.member + ":" + face.face;
      if (seen.exists(key)) fail();
      seen.set(key, true);
      if (face.target) normals.push(face.normal);
      var endpoints:Array<Array<Float>> = [];
      for (edge in face.chords) {
        if (edge == null || edge.length != 2 || !finiteVector(edge[0]) || !finiteVector(edge[1])) fail();
        var length = 0.0;
        for (i in 0...3) length += (edge[0][i] - edge[1][i]) * (edge[0][i] - edge[1][i]);
        if (!(length > 1e-20)) fail();
        for (point in edge) {
          var residual = 0.0;
          for (i in 0...3) residual += face.normal[i] * (point[i] - face.centre[i]);
          if (Math.abs(residual) > 1e-8) fail();
          endpoints.push(point);
        }
      }
      // Every contour vertex must meet exactly two edges, including hole contours.
      for (point in endpoints) {
        var degree = 0;
        for (other in endpoints) {
          var distance = 0.0;
          for (i in 0...3) distance += (point[i] - other[i]) * (point[i] - other[i]);
          if (distance < 1e-20) degree++;
        }
        if (degree != 2) fail();
      }
    }
    var independent = false;
    for (i in 0...normals.length) for (j in i + 1...normals.length) for (k in j + 1...normals.length) {
      var a = normals[i], b = normals[j], c = normals[k];
      var determinant = (a[1] * b[2] - a[2] * b[1]) * c[0] + (a[2] * b[0] - a[0] * b[2]) * c[1] +
        (a[0] * b[1] - a[1] * b[0]) * c[2];
      if (Math.abs(determinant) > 1e-3) independent = true;
    }
    if (!independent) fail();
  }
}

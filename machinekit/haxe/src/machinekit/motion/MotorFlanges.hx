package machinekit.motion;

/** A face that receives a motor flange: its pilot recess, bolt positions and, if tapped, their thread. */
typedef MotorReceivingFace = {
  var pilot:Float;
  var pilotDepth:Null<Float>;
  var bolts:Array<{x:Float, y:Float}>;
  /** The face's tapped thread, or null when it is drilled for screws into a tapped motor. */
  var thread:Null<String>;
  var threadDepth:Null<Float>;
}

/** Whether a motor's flange fits a face that receives it, as findings; no findings means it fits nominally. */
class MotorFlanges {
  public static function findings(face:MotorReceivingFace, flange:Null<MotorFlange>):Array<String> {
    if (flange == null) return ["Motor mounting dimensions are unspecified"];
    var out:Array<String> = [];
    if (Math.abs(flange.pilotDiameter - face.pilot) > 0.001) out.push("Motor pilot diameter differs");
    if (face.pilotDepth == null) out.push("Motor pilot recess depth is unspecified");
    else if (flange.pilotHeight > face.pilotDepth) out.push("Motor pilot bottoms before face seats");
    if (flange.holes.length != face.bolts.length) out.push('Face expects ${face.bolts.length} motor holes');
    for (hole in flange.holes) {
      var matched = false;
      for (bolt in face.bolts) if (Math.abs(hole.x - bolt.x) < 0.01 && Math.abs(hole.y - bolt.y) < 0.01) matched = true;
      if (!matched) out.push("Motor bolt pattern differs");
    }
    if (face.thread != null && flange.thread != null) out.push("Face and motor are both tapped");
    else if (face.thread != null) {
      var diameter = Std.parseFloat(face.thread.substr(1));
      if (flange.holeDiameter < diameter) out.push("Face thread exceeds motor clearance hole");
      if (face.threadDepth == null) out.push("Motor thread depth is not published; screw engagement unqualified");
    } else if (flange.thread == null) out.push("Neither face nor motor is tapped");
    return out;
  }
}

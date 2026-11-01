package gearkit;

/** Cam lookup by name: cosine, ellipse, four-roller, resal. */
class Cams {
  public static function make(name:String, neutralRadius:Float, w0:Float):Cam {
    return switch name {
      case "cosine": new CosineCam(w0);
      case "ellipse": new EllipseCam(neutralRadius, w0);
      case "four-roller": new FourRollerCam(neutralRadius, w0);
      case "resal": new ResalCam(w0);
      default: throw 'unknown cam "$name"';
    };
  }
}

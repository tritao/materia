package gearkit;

/** Cam lookup by the names `StrainWaveInput.CAMS` allows. */
class Cams {
  public static function make(name:String, neutralRadius:Float, w0:Float):Cam {
    return switch name {
      case "cosine": new CosineCam(w0);
      case "ellipse": new EllipseCam(neutralRadius, w0);
      case "four-roller": new FourRollerCam(neutralRadius, w0);
      case "resal": throw "cam \"resal\" is not implemented: the Resal curve equation (Gravagno 2016) has not been read";
      default: throw 'unknown cam "$name"';
    };
  }
}

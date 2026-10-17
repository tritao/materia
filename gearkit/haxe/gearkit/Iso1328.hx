package gearkit;

/** ISO 1328-1:2013, 5.2/5.3 and Annex E. Inputs mm, tolerances micrometres.
    Outside scope these numbers are engineering extrapolations, never ISO classes.
    Source and qualification requirements: gearkit/METROLOGY.md. */
class Iso1328 {
  public static final parameters = ["fp", "Fp", "Fr", "Falpha", "ffalpha", "fHalpha", "Fbeta", "ffbeta", "fHbeta"];

  public static function outsideScope(s:GearSpec):Array<String> {
    var out:Array<String> = [];
    if (s.teeth < 5 || s.teeth > 1000) out.push("teeth outside 5..1000");
    if (s.pitchDiameter() < 5 || s.pitchDiameter() > 15000) out.push("diameter outside 5..15000 mm");
    if (s.module < 0.5 || s.module > 70) out.push("module outside 0.5..70 mm");
    if (s.faceWidth < 4 || s.faceWidth > 1200) out.push("face width outside 4..1200 mm");
    return out;
  }
  public static function tolerance(s:GearSpec, parameter:String, grade:Int, rounded:Bool = true):Float {
    if (grade < 1 || grade > 11) throw "ISO flank class must be 1..11";
    var d = s.pitchDiameter(), m = s.module, b = s.faceWidth;
    var ph = 0.4*m + 0.001*d + 4, pf = 0.55*m + 5;
    var bh = 0.05*Math.sqrt(d) + 0.35*Math.sqrt(b) + 4;
    var bf = 0.07*Math.sqrt(d) + 0.45*Math.sqrt(b) + 4;
    var cumulative = 0.002*d + 0.55*Math.sqrt(d) + 0.7*m + 12;
    var base = switch parameter {
      case "fp": 0.001*d + 0.4*m + 5;
      case "Fp": cumulative;
      case "Fr": 0.9*cumulative;
      case "Falpha": Math.sqrt(ph*ph + pf*pf);
      case "ffalpha": pf;
      case "fHalpha": ph;
      case "Fbeta": Math.sqrt(bh*bh + bf*bf);
      case "ffbeta": bf;
      case "fHbeta": bh;
      default: throw 'unknown ISO parameter $parameter';
    };
    var value = base*Math.pow(2, (grade-5)/2);
    if (!rounded) return value;
    var step = value > 10 ? 1.0 : value >= 5 ? 0.5 : 0.1;
    return Math.round(value/step)*step;
  }
  /** null means worse than class 11; never clamp a failing measurement. */
  public static function grade(s:GearSpec, parameter:String, micrometres:Float):Null<Int> {
    if (!Math.isFinite(micrometres)) throw "deviation must be finite";
    for (a in 1...12) if (Math.abs(micrometres) <= tolerance(s, parameter, a)) return a;
    return null;
  }
}

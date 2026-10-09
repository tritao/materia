package toolpathkit.path;

/** Queries over the printing operations of `ToolpathOp`. */
class PrinterOps {
  /** Does this operation exist only for printing? */
  public static function isPrinter(op:ToolpathOp):Bool
    return switch op {
      case Deposit(_, _, _, _), DepositOff(_), FilamentMove(_, _, _), Heat(_, _, _, _), Fan(_, _, _),
          Layer(_, _, _), Feature(_, _), PrintObject(_, _, _): true;
      case _: false;
    };

  /** The source of a printing operation. */
  public static function provenance(op:ToolpathOp):Provenance
    return switch op {
      case Deposit(_, _, _, span), DepositOff(span), FilamentMove(_, _, span), Heat(_, _, _, span),
          Fan(_, _, span), Layer(_, _, span), Feature(_, span), PrintObject(_, _, span): span;
      case _: throw "not a printer operation";
    };

  /** The first printing operation of `ops`, or null. */
  public static function first(ops:Array<ToolpathOp>):Null<ToolpathOp> {
    for (op in ops) if (isPrinter(op)) return op;
    return null;
  }

  /** Throws if a printing operation is malformed. `index` names it in the message. */
  public static function validate(op:ToolpathOp, index:Int):Void {
    function fail(message:String):Void throw 'toolpath op $index: $message';
    switch op {
      case Deposit(area, width, height, _):
        if (!Math.isFinite(area) || !Math.isFinite(width) || !Math.isFinite(height) ||
            area <= 0.0 || width <= 0.0 || height <= 0.0)
          fail("Deposit needs a positive finite area, width and height");
        // The bead's section is a rounded rectangle inside its footprint.
        if (area > width * height * (1.0 + 1e-9))
          fail('Deposit area $area m^2 exceeds its ${width} x ${height} m footprint');
      case FilamentMove(length, speed, _):
        if (!Math.isFinite(length) || length == 0.0) fail("FilamentMove needs a nonzero finite length");
        if (!Math.isFinite(speed) || speed <= 0.0) fail("FilamentMove needs a positive finite speed");
      case Heat(heater, celsius, _, _):
        if (heater == null || heater.length == 0) fail("Heat needs a heater name");
        if (!Math.isFinite(celsius) || celsius < 0.0) fail("Heat needs a finite temperature of at least 0 C");
      case Fan(fan, duty, _):
        if (fan == null || fan.length == 0) fail("Fan needs a fan name");
        if (!Math.isFinite(duty) || duty < 0.0 || duty > 1.0) fail("Fan duty must be from 0 to 1");
      case Layer(layer, z, _):
        if (layer < 0 || !Math.isFinite(z)) fail("Layer needs a nonnegative index and a finite height");
      case PrintObject(name, _, _):
        if (name == null || name.length == 0) fail("PrintObject needs a name");
      case Feature(Other(name), _):
        if (name == null || name.length == 0) fail("Feature needs a name");
      case _:
    }
  }

  /**
    Where a program lays a bead or pushes filament before `heater` has been waited up to `minimumCelsius`, the
    material's minimum extrusion temperature: a `Heat` that waits for at least that, with no later `Heat` below it,
    is what makes extruding safe. The device faults a cold extrusion too; this finds it before the print starts.
    Each finding is the source of the offending operation and says what it did.
  **/
  public static function coldExtrusion(ops:Array<ToolpathOp>, minimumCelsius:Float, ?heater:String = "hotend"):Array<{provenance:Provenance, message:String}> {
    var findings:Array<{provenance:Provenance, message:String}> = [];
    var hot = false, depositing = false;
    for (op in ops) switch op {
      case Heat(name, celsius, wait, _) if (name == heater):
        // Waiting for the minimum makes it hot; turning it down (or only asking, without waiting) does not keep it so.
        if (celsius < minimumCelsius) hot = false;
        else if (wait) hot = true;
      case Deposit(_, _, _, _): depositing = true;
      case DepositOff(_): depositing = false;
      case FilamentMove(length, _, span) if (length > 0.0 && !hot):
        findings.push({provenance: span, message: 'filament is pushed before $heater has been waited up to ${minimumCelsius} C'});
      case Move(Cut, _, _, _, span) if (depositing && !hot):
        findings.push({provenance: span, message: 'a bead is laid before $heater has been waited up to ${minimumCelsius} C'});
      case _:
    }
    return findings;
  }
}

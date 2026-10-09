import toolpathkit.path.GeometryTools;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.PrinterOps;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathFrame;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.path.ArcFitting;
import toolpathkit.setup.Setup;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.tool.ToolLibrary;

/** The printing operations of the IR: validation, pass-through and volume queries. */
class PrinterOpCoverage {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  static function near(actual:Float, expected:Float, message:String, ?tolerance:Float = 1e-12):Void
    check(Math.abs(actual - expected) <= tolerance, '$message: expected $expected, got $actual');

  static function rejects(action:Void->Void, fragment:String):Void {
    var message = "";
    try action() catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf(fragment) >= 0, 'expected "$fragment" in "$message"');
  }

  static var span = Provenance.cam(1);

  static function program(ops:Array<ToolpathOp>):ToolpathProgram
    return new ToolpathProgram(ops, new ToolLibrary(), [new Setup("1", new Point3(0, 0, 0))]);

  static function line(x0:Float, x1:Float, ?z:Float = 0.0):ToolpathOp
    return ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(x0, 0, z), new Point3(x1, 0, z)), 0.05, 0.0, span);

  static function tag(op:ToolpathOp):String
    return switch op {
      case Heat(_, _, _, _): "heat";
      case Layer(_, _, _): "layer";
      case Feature(_, _): "feature";
      case Deposit(_, _, _, _): "deposit";
      case DepositOff(_): "depositOff";
      case FilamentMove(_, _, _): "filament";
      case Fan(_, _, _): "fan";
      case PrintObject(_, _, _): "object";
      case Move(Cut, _, _, _, _): "cut";
      case _: "other";
    };

  public static function run():Int {
    assertions = 0;
    validation();
    passThrough();
    volumes();
    return assertions;
  }

  static function validation():Void {
    // 0.45 x 0.2 mm bead: a stadium holds a little less than the rectangle.
    var width = 0.00045, height = 0.0002;
    var stadium = height * (width - height) + Math.PI * height * height / 4.0;
    check(stadium < width * height, "the bead footprint bounds its area");
    program([ToolpathOp.Deposit(stadium, width, height, span), ToolpathOp.DepositOff(span)]);
    program([ToolpathOp.Deposit(width * height, width, height, span)]);
    rejects(() -> program([ToolpathOp.Deposit(0.0, width, height, span)]), "Deposit needs a positive");
    rejects(() -> program([ToolpathOp.Deposit(stadium, -width, height, span)]), "Deposit needs a positive");
    rejects(() -> program([ToolpathOp.Deposit(stadium, width, Math.NaN, span)]), "Deposit needs a positive");
    rejects(() -> program([ToolpathOp.Deposit(width * height * 1.01, width, height, span)]), "exceeds its");
    rejects(() -> program([ToolpathOp.FilamentMove(0.0, 0.001, span)]), "nonzero");
    rejects(() -> program([ToolpathOp.FilamentMove(0.001, 0.0, span)]), "positive finite speed");
    rejects(() -> program([ToolpathOp.FilamentMove(0.001, Math.POSITIVE_INFINITY, span)]), "positive finite speed");
    program([ToolpathOp.FilamentMove(0.001, 0.02, span), ToolpathOp.FilamentMove(-0.001, 0.02, span)]);
    rejects(() -> program([ToolpathOp.Heat("", 210, true, span)]), "heater name");
    rejects(() -> program([ToolpathOp.Heat("hotend", -1, true, span)]), "temperature");
    program([ToolpathOp.Heat("hotend", 210, true, span), ToolpathOp.Heat("bed", 0, false, span)]);
    rejects(() -> program([ToolpathOp.Fan("", 0.5, span)]), "fan name");
    rejects(() -> program([ToolpathOp.Fan("part", 1.5, span)]), "duty must be from 0 to 1");
    rejects(() -> program([ToolpathOp.Fan("part", -0.1, span)]), "duty must be from 0 to 1");
    program([ToolpathOp.Fan("part", 0.0, span), ToolpathOp.Fan("part", 1.0, span)]);
    rejects(() -> program([ToolpathOp.Layer(-1, 0.0002, span)]), "nonnegative index");
    rejects(() -> program([ToolpathOp.PrintObject("", true, span)]), "needs a name");
    rejects(() -> program([ToolpathOp.Feature(Other(""), span)]), "needs a name");
    // The message names the operation.
    rejects(() -> program([line(0, 1), ToolpathOp.Fan("part", 2.0, span)]), "toolpath op 1");
    check(PrinterOps.isPrinter(ToolpathOp.Layer(0, 0.0002, span)), "markers are printer operations");
    check(!PrinterOps.isPrinter(line(0, 1)), "moves are not printer operations");
    check(PrinterOps.first([line(0, 1), ToolpathOp.DepositOff(span)]) != null, "the first printer operation is found");
    check(PrinterOps.first([line(0, 1)]) == null, "a CNC program has none");
  }

  static function passThrough():Void {
    var bead = 0.00000008;
    var ops:Array<ToolpathOp> = [
      ToolpathOp.Heat("hotend", 210, true, span), ToolpathOp.Layer(0, 0.0002, span),
      ToolpathOp.Feature(ExternalPerimeter, span), ToolpathOp.Deposit(bead, 0.00045, 0.0002, span),
      line(0, 0.01), ToolpathOp.DepositOff(span), ToolpathOp.FilamentMove(-0.001, 0.03, span)];
    // Arc fitting passes every printing operation through in place.
    var fitted = ArcFitting.fit(ops, 0.00002);
    check(fitted.length == ops.length, "arc fitting keeps every operation");
    for (index in 0...ops.length)
      check(tag(fitted[index]) == tag(ops[index]), 'operation $index keeps its kind');
    // The frame and the travel envelope ignore them.
    var all = program(ops);
    var frame = ToolpathFrame.of(all);
    for (op in all.ops) frame.advance(op);
    check(frame.toolNumber == -1 && frame.toolLength == 0.0, "the frame ignores printer operations");
    var envelope = new TravelEnvelope(new Point3(-0.1, -0.1, -0.1), new Point3(0.1, 0.1, 0.1));
    check(envelope.check(all).length == 0, "the travel envelope ignores printer operations");
    check(program([line(0, 0.5)]).ops.length == 1 && envelope.check(program([line(0, 0.5)])).length == 1,
      "and still checks moves");
  }

  static function volumes():Void {
    var wide = 0.00000008, narrow = 0.00000004;
    var travel = ToolpathOp.Move(Rapid, PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.5, 0, 0)), 1.0, 0.0, span);
    var arc = ToolpathOp.Move(Cut, PathGeometry.Arc(new Point3(0, 0, 0.0004), 0.01, 0.0, Math.PI), 0.05, 0.0, span);
    var layers = program([
      ToolpathOp.Deposit(narrow, 0.0004, 0.0002, span), line(0, 0.01), // before any layer: a prime
      ToolpathOp.PrintObject("a", true, span),
      ToolpathOp.Layer(0, 0.0002, span), ToolpathOp.Deposit(wide, 0.00045, 0.0002, span), line(0, 0.02, 0.0002),
      travel, line(0, 0.01, 0.0002),
      ToolpathOp.DepositOff(span), line(0, 0.5, 0.0002),
      ToolpathOp.PrintObject("a", false, span),
      ToolpathOp.PrintObject("b", true, span),
      ToolpathOp.Layer(1, 0.0004, span), ToolpathOp.Deposit(narrow, 0.0004, 0.0002, span), arc,
      ToolpathOp.PrintObject("b", false, span),
      ToolpathOp.Layer(0, 0.0002, span), line(0, 0.005, 0.0002)]);
    var arcLength = Math.PI * 0.01;
    near(program([line(0, 1)]).depositedVolume(), 0.0, "a program with no deposit lays nothing");
    var expected = narrow * 0.01 + wide * 0.03 + narrow * arcLength + narrow * 0.005;
    near(layers.depositedVolume(), expected, "total volume is bead area times cut length", 1e-18);
    var perLayer = layers.layerVolumes();
    check(perLayer.length == 3 && perLayer[0].index == -1 && perLayer[1].index == 0 && perLayer[2].index == 1,
      "layers appear in order, with the prime first");
    near(perLayer[0].volume, narrow * 0.01, "the prime before any layer", 1e-18);
    // A repeated layer index adds to the same entry.
    near(perLayer[1].volume, wide * 0.03 + narrow * 0.005, "layer 0 sums its beads", 1e-18);
    near(perLayer[2].volume, narrow * arcLength, "layer 1 is an arc", 1e-18);
    near(perLayer[1].z, 0.0002, "a layer keeps its height");
    var perObject = layers.objectVolumes();
    check(perObject.length == 2 && perObject[0].name == "a" && perObject[1].name == "b", "objects appear in order");
    near(perObject[0].volume, wide * 0.03, "object a", 1e-18);
    near(perObject[1].volume, narrow * arcLength, "object b", 1e-18);
    var summed = 0.0;
    for (layer in perLayer) summed += layer.volume;
    near(summed, layers.depositedVolume(), "layers add up to the total", 1e-18);
    near(GeometryTools.length(PathGeometry.Arc(new Point3(0, 0, 0), 0.01, 0.0, Math.PI)), arcLength,
      "the arc length is the fixture's");
  }
}

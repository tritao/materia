import camkit.additive.PrinterProfile;
import camkit.additive.Slicer;
import camkit.additive.SlicerSettings;
import camkit.additive.TestParts;
import cadkit.modeling.Part;
import cnckit.CncCompiler;
import cnckit.CncController;
import cnckit.printer.PrinterFlavour;
import cnckit.printer.PrinterWriter;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.PrintFeature;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;

/** The v1 slicer on the test parts: volume, footprint, holes, layers, determinism and the G-code round trip. */
class AdditiveSlicerTests {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  static function near(actual:Float, expected:Float, message:String, tolerance:Float):Void
    check(Math.abs(actual - expected) <= tolerance, '$message: expected $expected, got $actual');

  static function relative(actual:Float, expected:Float, message:String, tolerance:Float):Void
    check(Math.abs(actual - expected) <= tolerance * Math.abs(expected), '$message: expected $expected, got $actual (${Math.round((actual / expected - 1.0) * 10000) / 100} %)');

  public static function run():Int {
    assertions = 0;
    tab();
    cube();
    tower();
    return assertions;
  }

  /** Every point sampled along every deposited move, with the feature it belongs to, in metres. */
  static function beads(program:ToolpathProgram):Array<{x:Float, y:Float, z:Float, feature:String}> {
    var points = [];
    var depositing = false, feature = "", layerFeature = "";
    for (op in program.ops) switch op {
      case Deposit(_, _, _, _): depositing = true;
      case DepositOff(_): depositing = false;
      case Feature(kind, _): feature = Std.string(kind);
      case Move(Cut, geometry, _, _, _):
        if (!depositing) continue;
        var length = GeometryTools.length(geometry);
        var steps = Std.int(Math.max(1, Math.ceil(length / 0.0005)));
        for (step in 0...steps + 1) {
          var point = GeometryTools.pointAt(geometry, Math.min(length, length * step / steps));
          points.push({x: point.x, y: point.y, z: point.z, feature: feature});
        }
      case _:
    }
    return points;
  }

  static function layerHeights(program:ToolpathProgram):Array<Float> {
    var heights = [];
    for (op in program.ops) switch op {
      case Layer(_, z, _): heights.push(z);
      case _:
    }
    return heights;
  }

  static function withPrime(length:Float):PrinterProfile {
    var p = PrinterProfile.bed220();
    return new PrinterProfile(p.bedCenterX,p.bedCenterY,p.purgeStart,p.purgeEnd,
      p.purgeHeight,p.purgeWidth,p.purgeSpeed,p.home,p.parkLift,length);
  }

  static function startupPrime(program:ToolpathProgram):Float {
    var length = 0.0;
    for (op in program.ops) switch op {
      case FilamentMove(value,_,_):
        check(value>=0,"startup never retracts before its first deposited bead");
        length += value;
      case Deposit(_,_,_,_): return length;
      case _:
    }
    throw "Sliced program has no purge bead";
  }

  static function gcode(program:ToolpathProgram):String
    return PrinterWriter.write(program, new CncController(Printer), Klipper);

  static function tab():Void {
    var part = TestParts.smallTab();
    var settings = new SlicerSettings();
    var result = Slicer.slice(part, settings);
    var program = result.program;
    near(startupPrime(program),0.0015,"default startup prime stays 1.5 mm",1e-12);
    for (prime in [0.0,0.0027]) {
      var custom = Slicer.slice(part,settings,withPrime(prime)).program;
      near(startupPrime(custom),prime,"profile controls startup filament",1e-12);
      near(custom.depositedVolume(),program.depositedVolume(),"prime does not alter path deposition",1e-18);
      var roundTrip = CncCompiler.compileDetailed(gcode(custom),new CncController(Printer));
      near(startupPrime(roundTrip.program),prime,"startup prime survives G-code round trip",1e-9);
    }
    for (bad in [-1.0,Math.NaN,Math.POSITIVE_INFINITY]) {
      var rejected = false; try withPrime(bad) catch (_:Dynamic) rejected = true;
      check(rejected,"invalid profile prime rejected before slicing");
    }
    Sys.println('slicing: SmallTab ${result.report.layers} layers in ${Math.round(result.report.seconds * 1000)} ms, ${program.ops.length} operations, ${Math.round(program.depositedVolume() * 1e9 * 10) / 10} mm^3');
    if (Sys.getEnv("SLICER_DEBUG") == "1") Sys.println(gcode(program));
    check(result.report.layers == 6, "SmallTab is six layers");
    var heights = layerHeights(program);
    check(heights.length == 6, "a Layer marker each");
    for (index in 0...6) near(heights[index], 0.0002 * (index + 1), "layer heights go up by 0.2 mm", 1e-12);
    // Four bottom and four top layers cover a 6-layer tab: it is solid throughout.
    var objects = program.objectVolumes();
    check(objects.length == 1 && objects[0].name == "part", "the part is one object");
    relative(objects[0].volume, 120e-9, "a solid tab deposits its CAD volume", 0.03);
    check(program.depositedVolume() > objects[0].volume, "the skirt and the purge line are outside the object");
    // Nothing is deposited outside the footprint plus half a bead, except the skirt and the purge line.
    var centerX = 0.11, centerY = 0.11, half = 0.005 + 0.00045 / 2.0 + 1e-6;
    var outside = 0, skirt = 0;
    for (point in beads(program)) {
      if (point.feature == "Skirt") { skirt++; continue; }
      if (Math.abs(point.x - centerX) > half || Math.abs(point.y - centerY) > half) outside++;
    }
    check(outside == 0 && skirt > 0, 'no bead outside the footprint plus half a width ($outside outside, $skirt skirt points)');
    // The first layer is the only one with a skirt and runs slower than the rest.
    var speeds = new Map<String, Bool>();
    for (op in program.ops) switch op {
      case Move(Cut, _, speed, _, _): speeds.set(Std.string(Math.round(speed * 1e6)), true);
      case _:
    }
    check(speeds.exists("20000"), "the first layer prints at its own speed");
    // Start and end sequences.
    var text = gcode(program);
    check(text.indexOf("M190 S60") >= 0 && text.indexOf("M109 S210") >= 0, "the start heats the bed and the hotend and waits");
    check(text.indexOf("M104 S0") >= 0 && text.indexOf("M140 S0") >= 0 && text.indexOf("M107") >= 0, "the end switches the heaters and the fan off");
    check(text.indexOf("M106 S255") >= 0, "the fan comes on after the first layers");
    // Same input, same program.
    check(gcode(Slicer.slice(part, settings).program) == text, "slicing is deterministic");
    // The G-code reads back to the same volume.
    var back = CncCompiler.compileDetailed(text, new CncController(Printer));
    relative(back.program.depositedVolume(), program.depositedVolume(), "G-code round trip", 0.001);
    check(back.program.layerVolumes().length == program.layerVolumes().length + 0, "and the layers");
    // A first layer of its own height has its own bead spacing: its infill starts where its own last perimeter ends.
    var thick = new SlicerSettings();
    thick.firstLayerHeight = 0.0003;
    var thickProgram = Slicer.slice(part, thick).program;
    var spacing = thick.beadSpacing(thick.firstLayerHeight);
    var inset = thick.lineWidth / 2.0 + (thick.perimeters - 1) * spacing + spacing / 2.0;
    var reach = 0.0;
    for (point in beads(thickProgram)) if (point.feature == "SolidInfill" && Math.abs(point.z - 0.0003) < 1e-9)
      reach = Math.max(reach, Math.max(Math.abs(point.x - centerX), Math.abs(point.y - centerY)));
    near(reach, 0.005 - inset, "a 0.3 mm first layer's infill reaches its own core", 1e-7);
    part.close();
  }

  static function cube():Void {
    var part = TestParts.calibrationCube();
    var settings = new SlicerSettings();
    settings.infillDensity = 1.0;
    var result = Slicer.slice(part, settings);
    var program = result.program;
    var cad = part.volume() * 1e-9;
    Sys.println('slicing: CalibrationCube ${result.report.layers} layers in ${Math.round(result.report.seconds * 1000) / 1000} s, ${program.ops.length} operations, ${result.report.retractions} retractions, ${Math.round(program.depositedVolume() * 1e9)} of ${Math.round(cad * 1e9)} mm^3');
    check(result.report.layers == 100, "a 20 mm cube is 100 layers");
    relative(program.objectVolumes()[0].volume, cad, "a solid cube deposits its CAD volume (within 3 %)", 0.03);
    // The 6 mm hole stays open in every layer: no bead centre nearer its axis than its wall plus half a bead.
    var holeX = 0.11 + 0.002, holeY = 0.11 - 0.001;
    var nearest = Math.POSITIVE_INFINITY;
    var footprint = 0.01 + 0.00045 / 2.0 + 1e-6, outside = 0;
    for (point in beads(program)) {
      if (point.feature == "Skirt") continue;
      nearest = Math.min(nearest, Math.sqrt((point.x - holeX) * (point.x - holeX) + (point.y - holeY) * (point.y - holeY)));
      if (Math.abs(point.x - 0.11) > footprint || Math.abs(point.y - 0.11) > footprint) outside++;
    }
    near(nearest, 0.003 + 0.00045 / 2.0, "the hole's wall is a half bead away from its nearest bead", 5e-6);
    check(outside == 0, "nothing is laid outside the cube's footprint plus a half width");
    // Layers climb by one layer height, and every deposited point is at its layer's height.
    var heights = layerHeights(program);
    for (index in 1...heights.length) near(heights[index] - heights[index - 1], 0.0002, "monotonic layers", 1e-12);
    var level = 0.0, mismatched = 0;
    for (op in program.ops) switch op {
      case Layer(_, z, _): level = z;
      case Move(Cut, geometry, _, _, _):
        var end = GeometryTools.pointAt(geometry, GeometryTools.length(geometry));
        if (level > 0.0 && Math.abs(end.z - level) > 1e-9 && end.z > 0.0004) mismatched++;
      case _:
    }
    check(mismatched == 0, "moves lie at their layer's height");
    // The hole is an exact arc, not a polygon.
    var arcs = 0;
    for (op in program.ops) switch op {
      case Move(Cut, Arc(_, radius, _, _), _, _, _): arcs++;
      case _:
    }
    check(arcs >= 100, 'the hole walls are arcs in every layer ($arcs)');
    // At 15 % infill the cube deposits less than its volume and more than its walls and skins.
    var sparse = new SlicerSettings();
    var sparseResult = Slicer.slice(part, sparse);
    var sparseVolume = sparseResult.program.objectVolumes()[0].volume;
    Sys.println('slicing: CalibrationCube at 15 % infill in ${Math.round(sparseResult.report.seconds * 1000) / 1000} s, ${Math.round(sparseVolume * 1e9)} mm^3 deposited');
    check(sparseVolume < 0.5 * cad && sparseVolume > 0.15 * cad, "sparse infill saves material but keeps walls and skins");
    part.close();
  }

  static function tower():Void {
    var part = TestParts.testTower();
    var settings = new SlicerSettings();
    settings.infillDensity = 1.0;
    var result = Slicer.slice(part, settings);
    var program = result.program;
    var cad = part.volume() * 1e-9;
    Sys.println('slicing: TestTower ${result.report.layers} layers in ${Math.round(result.report.seconds * 1000) / 1000} s, ${program.ops.length} operations, ${Math.round(program.depositedVolume() * 1e9)} of ${Math.round(cad * 1e9)} mm^3');
    check(result.report.layers == 60, "a 12 mm tower is 60 layers");
    relative(program.objectVolumes()[0].volume, cad, "a solid tower deposits its CAD volume (within 3 %)", 0.03);
    var features = new Map<String, Int>();
    for (op in program.ops) switch op {
      case Feature(kind, _):
        var name = Std.string(kind);
        var seen:Int = features.exists(name) ? features.get(name) : 0;
        features.set(name, seen + 1);
      case _:
    }
    check(features.exists("ExternalPerimeter") && features.exists("Perimeter") && features.exists("SolidInfill") && features.exists("TopSolidInfill") && features.exists("Skirt"),
      "perimeters, skin and skirt are all marked: " + [for (key in features.keys()) key]);
    // The boss' counterbore is open: no bead inside radius 4 mm of the axis above z = 10 mm (a half bead from the wall).
    var violations = 0;
    for (point in beads(program)) {
      if (point.feature == "Skirt" || point.z < 0.0102) continue;
      var radius = Math.sqrt((point.x - 0.11) * (point.x - 0.11) + (point.y - 0.11) * (point.y - 0.11));
      if (radius < 0.004 + 0.00045 / 2.0 - 5e-6) violations++;
    }
    check(violations == 0, "the counterbore stays open");
    // Sparse and the G-code round trip on the harder part.
    var sparse = Slicer.slice(part, new SlicerSettings());
    var text = gcode(sparse.program);
    var back = CncCompiler.compileDetailed(text, new CncController(Printer));
    var errors = [for (item in back.diagnostics) if (item.severity == Error) item];
    check(errors.length == 0, "the tower's G-code reads back: " + errors);
    relative(back.program.depositedVolume(), sparse.program.depositedVolume(), "tower G-code round trip", 0.001);
    Sys.println('slicing: TestTower at 15 % infill in ${Math.round(sparse.report.seconds * 1000) / 1000} s, ${sparse.report.retractions} retractions, ${text.split("\n").length} lines of G-code');
    part.close();
  }
}

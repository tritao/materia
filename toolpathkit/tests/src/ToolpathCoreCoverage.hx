import toolpathkit.path.ArcPlane;
import toolpathkit.path.GeometryOffset;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.setup.Fixture;
import toolpathkit.setup.Setup;
import toolpathkit.setup.SetupStock;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.tool.CutterProfile;
import toolpathkit.tool.CutterSegment;
import toolpathkit.tool.Tool;
import toolpathkit.tool.ToolLibrary;

/** Core geometry and validation checks with no producer or motion dependency. */
class ToolpathCoreCoverage {
  static var assertions = 0;

  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }

  static function near(actual:Float, expected:Float, message:String,
      ?tolerance:Float = 1e-9):Void
    check(Math.abs(actual - expected) <= tolerance,
      '$message: expected $expected, got $actual');

  static function rejects(action:Void->Void, fragment:String):Void {
    var message = "";
    try action() catch (error:Dynamic) message = Std.string(error);
    check(message.indexOf(fragment) >= 0,
      'expected "$fragment" in "$message"');
  }

  public static function run():Int {
    geometry();
    offsets();
    setups();
    travel();
    cutters();
    return assertions;
  }

  static function geometry():Void {
    var line = PathGeometry.Line(new Point3(0, 0, 0), new Point3(3, 4, 0));
    near(GeometryTools.length(line), 5, "line length");
    var mid = GeometryTools.pointAt(line, 2.5);
    near(mid.x, 1.5, "line midpoint X");
    near(mid.y, 2, "line midpoint Y");
    var tangent = GeometryTools.tangentAt(line, 1);
    near(tangent.x, 0.6, "line tangent X");
    near(tangent.y, 0.8, "line tangent Y");
    rejects(function() GeometryTools.pointAt(line, 6), "outside");
    rejects(function() GeometryTools.tangentAt(
      PathGeometry.Line(new Point3(0, 0, 0), new Point3(0, 0, 0)), 0),
      "nonzero");

    var arc = PathGeometry.Arc(new Point3(0, 0, 0), 1, 0, Math.PI / 2);
    near(GeometryTools.length(arc), Math.PI / 2, "XY arc length");
    mid = GeometryTools.pointAt(arc, GeometryTools.length(arc) / 2);
    near(mid.x, Math.sqrt(0.5), "XY arc midpoint X");
    near(mid.y, Math.sqrt(0.5), "XY arc midpoint Y");
    tangent = GeometryTools.tangentAt(arc, 0);
    near(tangent.x, 0, "XY arc start tangent X");
    near(tangent.y, 1, "XY arc start tangent Y");
    tangent = GeometryTools.tangentAt(arc, GeometryTools.length(arc));
    near(tangent.x, -1, "XY arc end tangent X");
    var reverse = PathGeometry.Arc(new Point3(0, 0, 0), 1,
      Math.PI / 2, -Math.PI / 2);
    tangent = GeometryTools.tangentAt(reverse, 0);
    near(tangent.x, 1, "negative arc sweep reverses tangent");
    near(GeometryTools.pointAt(reverse, GeometryTools.length(reverse)).x,
      1, "negative arc sweep ends at X");
    var full = PathGeometry.Arc(new Point3(0, 0, 0), 1, 0,
      2 * Math.PI);
    near(GeometryTools.length(full), 2 * Math.PI, "full circle length");
    near(GeometryTools.pointAt(full, GeometryTools.length(full)).x,
      1, "full circle closes");

    for (plane in [ArcPlane.XY, ArcPlane.XZ, ArcPlane.YZ]) {
      var helix = PathGeometry.Circular(new Point3(0, 0, 0), 1, 0,
        Math.PI / 2, plane, 0.2);
      var length = GeometryTools.length(helix);
      near(length, Math.sqrt(Math.PI * Math.PI / 4 + 0.04),
        '$plane helix length');
      var end = GeometryTools.pointAt(helix, length);
      tangent = GeometryTools.tangentAt(helix, length / 2);
      near(Math.sqrt(tangent.x * tangent.x + tangent.y * tangent.y +
        tangent.z * tangent.z), 1, '$plane helix unit tangent');
      switch plane {
        case XY: near(end.z, 0.2, "XY helix rise");
        case XZ: near(end.y, 0.2, "XZ helix rise");
        case YZ: near(end.x, 0.2, "YZ helix rise");
      }
      var backwards = PathGeometry.Circular(new Point3(0, 0, 0), 1,
        Math.PI / 2, -Math.PI / 2, plane, -0.2);
      end = GeometryTools.pointAt(backwards, GeometryTools.length(backwards));
      switch plane {
        case XY: near(end.z, -0.2, "negative XY helix rise");
        case XZ: near(end.y, -0.2, "negative XZ helix rise");
        case YZ: near(end.x, -0.2, "negative YZ helix rise");
      }
    }
  }

  static function offsets():Void {
    var square = [new Point3(0, 0, 0), new Point3(1, 0, 0),
      new Point3(1, 1, 0), new Point3(0, 1, 0)];
    var outside = GeometryOffset.profile(square, 0.1, -0.01, true);
    var inside = GeometryOffset.profile(square, 0.1, -0.01, false);
    check(outside.length == 8, "outside square rounds four corners");
    check(inside.length == 4, "inside square trims four corners");
    var first = GeometryTools.pointAt(outside[0], 0);
    near(first.y, -0.1, "outside offset moves away from stock");
    first = GeometryTools.pointAt(inside[0], 0);
    near(first.x, 0.1, "inside offset trims first corner X");
    near(first.y, 0.1, "inside offset moves toward stock");
    var shifted = GeometryOffset.translate(outside[0], [2, 3, 4]);
    first = GeometryTools.pointAt(shifted, 0);
    near(first.x, 2, "translated line X");
    near(first.y, 2.9, "translated line Y");
    near(first.z, 3.99, "translated line Z");
    var arc = GeometryOffset.translate(PathGeometry.Arc(
      new Point3(1, 2, 3), 0.5, 0, Math.PI), [1, -1, 2]);
    first = GeometryTools.pointAt(arc, 0);
    near(first.x, 2.5, "translated arc X");
    near(first.y, 1, "translated arc Y");
    var circle = GeometryOffset.translate(PathGeometry.Circular(
      new Point3(0, 0, 0), 1, 0, Math.PI, XZ, 0.2), [1, 2, 3]);
    first = GeometryTools.pointAt(circle, 0);
    near(first.x, 2, "translated circular X");
    near(first.y, 2, "translated circular Y");
    rejects(function() GeometryOffset.translate(arc, [1, 2]), "three offsets");
    rejects(function() GeometryOffset.translate(arc, [Math.NaN, 0, 0]), "finite");
    rejects(function() GeometryOffset.profile(square, 0, 0, true), "positive radius");
    rejects(function() GeometryOffset.profile(square, 0.6, 0, false), "narrow feature");
    rejects(function() GeometryOffset.profile([
      new Point3(0, 0, 0), new Point3(1, 0, 0), new Point3(2, 0, 0)],
      0.1, 0, true), "nonzero contour area");
  }

  static function setups():Void {
    var library = new ToolLibrary();
    library.set(new Tool(4, 0, 0.1));
    var clear = new Setup("1", new Point3(0, 0, 0),
      new SetupStock(-1, 1, -1, 1, 0, -0.2, 0.2));
    var span = Provenance.cam(3);
    var cut = [ToolpathOp.ToolChange(4, span), ToolpathOp.Move(Cut,
      PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.2, 0, -0.1)),
      0.01, 0, span)];
    clear.validate(cut, library);
    check(true, "stocked setup accepts a clear cut");
    var shallow = new Setup("1", new Point3(0, 0, 0),
      new SetupStock(-1, 1, -1, 1, 0, -0.05, 0.2));
    rejects(function() shallow.validate(cut, library), "below stock bottom");
    var lowRapid = [ToolpathOp.Move(Rapid, PathGeometry.Line(
      new Point3(0, 0, 0.1), new Point3(0.2, 0, 0.1)),
      0, 0, span)];
    rejects(function() clear.validate(lowRapid, library), "below safe Z");
    var clamp = new Fixture("clamp", 0.08, 0.12, -0.02, 0.02,
      -0.2, 0.15);
    var occupied = new Setup("1", new Point3(0, 0, 0),
      new SetupStock(-1, 1, -1, 1, 0, -0.2, 0.2, [clamp]));
    rejects(function() occupied.validate(cut, library), "clamp");
    rejects(function() new SetupStock(-1, 1, -1, 1, 0,
      -0.2, 0.1, [clamp]), "safe Z");
    rejects(function() new Setup("1", new Point3(0, 0, 0))
      .validate(cut, library), "needs stock");
  }

  static function travel():Void {
    var library = new ToolLibrary();
    var setup = new Setup("1", new Point3(0, 0, 0));
    var span = Provenance.cam(11);
    function violations(geometry:PathGeometry, lower:Point3,
        upper:Point3):Array<toolpathkit.setup.TravelEnvelope.TravelViolation>
      return new TravelEnvelope(lower, upper).check(new ToolpathProgram([
        ToolpathOp.Move(Cut, geometry, 0.01, 0, span)], library, [setup]));
    for (sweep in [Math.PI / 2, -Math.PI / 2]) {
      var start = sweep > 0 ? -Math.PI / 4 : Math.PI / 4;
      var arc = PathGeometry.Arc(new Point3(0, 0, 0), 1, start, sweep);
      var found = violations(arc, new Point3(-2, -2, -2),
        new Point3(0.9, 2, 2));
      check(found.length == 1 && found[0].axis == 0 &&
        found[0].provenance == span,
        "XY arc quadrant high point preserves provenance");
    }
    var xz = PathGeometry.Circular(new Point3(0, 0, 0), 1,
      Math.PI / 4, Math.PI / 2, XZ, 0.1);
    var yz = PathGeometry.Circular(new Point3(0, 0, 0), 1,
      Math.PI / 4, -Math.PI / 2, YZ, 0.1);
    check(violations(xz, new Point3(-2, -2, -2),
      new Point3(2, 2, 0.9))[0].axis == 2,
      "XZ helix finds Z quadrant high point");
    check(violations(yz, new Point3(-2, -2, -2),
      new Point3(2, 0.9, 2))[0].axis == 1,
      "negative YZ helix finds Y quadrant high point");
    check(violations(xz, new Point3(-2, -2, -2),
      new Point3(2, 0.05, 2))[0].axis == 1,
      "helix checks axial rise");
    rejects(function() new TravelEnvelope(new Point3(1, 0, 0),
      new Point3(0, 1, 1)), "ordered bounds");
  }

  static function cutters():Void {
    var library = new ToolLibrary();
    var flat = CutterProfile.flat(0.01, 0.02);
    var ball = CutterProfile.ball(0.01, 0.02);
    var bull = CutterProfile.bullNose(0.01, 0.001, 0.02);
    var vee = CutterProfile.vee(0.01, Math.PI / 2, 0.02);
    for (shape in [flat, ball, bull, vee]) {
      near(shape.cuttingDiameter(), 0.01, "cutter cutting diameter");
      near(shape.fluteLength(), 0.02, "cutter flute length");
    }
    check(switch flat.segments[0] {
      case CutterSegment.Line(0, 0, _, 0, _): true;
      case _: false;
    }, "flat mill has a flat tip");
    check(switch ball.segments[0] {
      case CutterSegment.Arc(_, _, _, _, _, _, _): true;
      case _: false;
    }, "ball mill has a round tip");
    check(switch bull.segments[0] {
      case CutterSegment.Line(0, 0, _, 0, _): true;
      case _: false;
    } && switch bull.segments[1] {
      case CutterSegment.Arc(_, _, _, _, _, _, _): true;
      case _: false;
    }, "bull nose has a flat centre and round corner");
    check(switch vee.segments[0] {
      case CutterSegment.Line(0, 0, _, z, _) if (z > 0): true;
      case _: false;
    }, "V bit rises from a sharp tip");
    var shaped = Tool.shaped(7, 0.03, bull);
    library.set(shaped);
    check(library.tool(7).profile() == bull, "library keeps the exact cutter");
    near(library.tool(7).diameter, 0.01, "tool diameter follows its shape");
    rejects(function() library.tool(8), "Unknown tool");
    rejects(function() CutterProfile.bullNose(0.01, 0.006, 0.02),
      "corner radius");
    rejects(function() CutterProfile.vee(0.01, Math.PI, 0.02),
      "included angle");
  }
}

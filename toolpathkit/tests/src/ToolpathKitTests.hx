import toolpathkit.ToolpathKit;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.path.Provenance.ProvenanceKind;
import toolpathkit.path.SpindleDirection;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.setup.Setup;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.tool.ToolLibrary;
import toolpathkit.tool.Tool;
import toolpathkit.tool.CutterProfile;
import toolpathkit.path.ToolpathFrame;
import toolpathkit.path.ArcFitting;
import toolpathkit.setup.SetupStock;

class ToolpathKitTests {
  static function main():Void {
    var kit = new ToolpathKit();
    if (kit == null) throw "ToolpathKit failed to initialize";
    var line = PathGeometry.Line(new Point3(0, 0, 0), new Point3(3, 4, 0));
    if (GeometryTools.length(line) != 5) throw "line length";
    if (GeometryTools.pointAt(line, 2.5).x != 1.5) throw "line midpoint";
    if (new Provenance(2, 3, 1).kind != ProvenanceKind.GCode)
      throw "G-code provenance";
    if (Provenance.cam(4).operationIndex != 4 ||
        Provenance.cam(4).kind != ProvenanceKind.Cam)
      throw "CAM provenance";
    if (Provenance.cam(4, "face:2").operationId != "cam:4" ||
        Provenance.cam(4, "face:2").featureRef != "face:2")
      throw "CAM feature reference";
    var move = ToolpathOp.Move(MoveKind.Cut, line, 0.01, 0.0001,
      Provenance.cam(4));
    switch move {
      case Move(Cut, _, feed, tolerance, _):
        if (feed != 0.01 || tolerance != 0.0001) throw "move parameters";
      case _: throw "move kind";
    }
    switch ToolpathOp.Spindle(Clockwise, 12000, Provenance.cam(4)) {
      case Spindle(Clockwise, rpm, _) if (rpm == 12000):
      case _: throw "spindle state";
    }
    switch ToolpathOp.Coolant(true, false, Provenance.cam(4)) {
      case Coolant(true, false, _):
      case _: throw "coolant state";
    }
    var halfArc = ToolpathOp.Move(Cut,
      PathGeometry.Arc(new Point3(0, 0, 0), 1, 0, Math.PI),
      0.01, 0, Provenance.cam(5));
    var violations = new TravelEnvelope(new Point3(-2, -2, -1),
      new Point3(2, 0.5, 1)).check(new ToolpathProgram([halfArc],
      new ToolLibrary(), [new Setup("1", new Point3(0, 0, 0))]));
    if (violations.length != 1 || violations[0].axis != 1 ||
        violations[0].provenance.operationIndex != 5)
      throw "arc extreme travel violation";
    var library = new ToolLibrary();
    var first = new Setup("1", new Point3(0, 0, 0));
    var rejected = false;
    try new ToolpathProgram([], library, []) catch (_:Dynamic) rejected = true;
    if (!rejected) throw "empty setup list accepted";
    rejected = false;
    try new ToolpathProgram([], library, [first, first])
    catch (_:Dynamic) rejected = true;
    if (!rejected) throw "duplicate setup accepted";
    rejected = false;
    try new ToolpathProgram([ToolpathOp.SetSetup("missing", Provenance.cam(6))],
      library, [first]) catch (_:Dynamic) rejected = true;
    if (!rejected) throw "unknown setup accepted";
    rejected = false;
    try first.validate([], library) catch (_:Dynamic) rejected = true;
    if (!rejected) throw "stock validation accepted an unstocked setup";
    var placed = new ToolpathProgram([ToolpathOp.Move(Cut,
      PathGeometry.Line(new Point3(0, 0, 0), new Point3(0.01, 0, 0)),
      0.01, 0, Provenance.cam(7))], library,
      [new Setup("1", new Point3(0.1, 0, 0))]);
    var placedViolations = new TravelEnvelope(new Point3(-0.05, -1, -1),
      new Point3(0.05, 1, 1)).check(placed);
    if (placedViolations.length != 1 || placedViolations[0].axis != 0 ||
        placedViolations[0].provenance.operationIndex != 7)
      throw "travel check must place work geometry and preserve provenance";
    // Travel is checked at the controlled point: G43 lifts it by the active length.
    var measured = new ToolLibrary();
    measured.set(new Tool(3, 0.04, 0.006));
    var lifted = new ToolpathProgram([ToolpathOp.ToolChange(3, Provenance.cam(8)),
      ToolpathOp.ToolLengthOffset(3, 0.04, Provenance.cam(8)),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0), new Point3(0, 0, -0.03)),
        0.01, 0, Provenance.cam(8))], measured, [new Setup("1", new Point3(0, 0, 0))]);
    if (new TravelEnvelope(new Point3(-1, -1, 0), new Point3(1, 1, 1)).check(lifted).length != 0)
      throw "travel check must lift G43 moves to the gauge line";
    var frame = ToolpathFrame.of(lifted);
    for (op in lifted.ops) frame.advance(op);
    if (frame.toMachine()[2] != 0.04 || frame.tipShift() != 0.0 || frame.toolNumber != 3)
      throw "frame tracks the tool and its G43 length";
    // A wrong H length moves the real tip, and stock validation sees it.
    var shortH = new ToolpathProgram([ToolpathOp.ToolChange(3, Provenance.cam(9)),
      ToolpathOp.ToolLengthOffset(3, 0.035, Provenance.cam(9)),
      ToolpathOp.Move(Cut, PathGeometry.Line(new Point3(0, 0, 0), new Point3(0, 0, -0.018)),
        0.01, 0, Provenance.cam(9))], measured, [new Setup("1", new Point3(0, 0, 0))]);
    rejected = false;
    try new Setup("1", new Point3(0, 0, 0), new SetupStock(-1, 1, -1, 1, 0, -0.02, 0.01))
      .validate(shortH.ops, measured) catch (_:Dynamic) rejected = true;
    if (!rejected) throw "validation must cut at the physical tip under a wrong H length";
    var profile = CutterProfile.bullNose(0.006, 0.001, 0.02).withShank(0.005, 0.01)
      .withHolder(0.02, 0.015);
    var decoded = CutterProfile.decode(profile.encode());
    if (Std.string(decoded.segments) != Std.string(profile.segments))
      throw "cutter profile codec round trip";
    rejected = false;
    try CutterProfile.decode([[0.0, 7.0, 0, 0, 1, 0]]) catch (_:Dynamic) rejected = true;
    if (!rejected) throw "cutter profile codec rejects unknown zones";
    // Arc fitting: a polygonized circle becomes one arc, a hexagon stays lines,
    // and an S-curve becomes two arcs turning opposite ways.
    function polygon(sides:Int, radius:Float, ?centerX:Float = 0.0, ?from:Float = 0.0,
        ?sweep:Float = 6.283185307179586):Array<Point3>
      return [for (k in 0...(sides + 1)) new Point3(centerX + radius * Math.cos(from + sweep * k / sides),
        radius * Math.sin(from + sweep * k / sides), -0.002)];
    function cuts(points:Array<Point3>, span:Provenance):Array<ToolpathOp>
      return [for (k in 1...points.length) ToolpathOp.Move(Cut, PathGeometry.Line(points[k - 1], points[k]),
        0.01, 0.0, span)];
    var span = Provenance.cam(10);
    var circle = ArcFitting.fit(cuts(polygon(64, 0.005), span), 0.00002);
    switch circle {
      case [Move(Cut, Arc(center, radius, _, sweep), _, _, _)]:
        if (Math.abs(radius - 0.005) > 1e-12 || Math.abs(Math.abs(sweep) - 2 * Math.PI) > 1e-9 ||
            center.distanceTo(new Point3(0, 0, -0.002)) > 1e-12)
          throw "a polygonized circle fits its own circle";
      case _: throw 'a polygonized circle becomes one arc, got $circle';
    }
    if (ArcFitting.fit(cuts(polygon(6, 0.005), span), 0.00002).length != 6)
      throw "a hexagon is not rounded";
    var s = polygon(32, 0.005, 0.0, Math.PI, -Math.PI);
    var second = polygon(32, 0.005, 0.01, Math.PI, Math.PI);
    var curve = ArcFitting.fit(cuts(s.concat(second.slice(1)), span), 0.00002);
    switch curve {
      case [Move(Cut, Arc(_, _, _, first), _, _, _), Move(Cut, Arc(_, _, _, next), _, _, _)]:
        if (!(first < 0.0 && next > 0.0)) throw "an S-curve turns one way, then the other";
      case _: throw 'an S-curve becomes two arcs, got ${curve.length} operations';
    }
    var coverage = ToolpathCoreCoverage.run();
    Sys.println('ToolpathKit tests passed (${22 + coverage} assertions)');
  }
}

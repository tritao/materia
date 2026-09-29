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
    Sys.println("ToolpathKit tests passed (14 assertions)");
  }
}

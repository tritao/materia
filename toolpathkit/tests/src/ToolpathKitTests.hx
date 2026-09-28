import toolpathkit.ToolpathKit;
import toolpathkit.path.GeometryTools;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.path.Provenance.ProvenanceKind;
import toolpathkit.path.ToolpathOp;

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
    var move = ToolpathOp.Move(MoveKind.Cut, line, 0.01, 0.0001,
      Provenance.cam(4));
    switch move {
      case Move(Cut, _, feed, tolerance, _):
        if (feed != 0.01 || tolerance != 0.0001) throw "move parameters";
      case _: throw "move kind";
    }
    Sys.println("ToolpathKit tests passed (5 assertions)");
  }
}

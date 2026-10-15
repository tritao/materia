import toolpathkit.motion.RapidClearance;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.MoveKind;
import toolpathkit.path.Point3;
import toolpathkit.path.Provenance;
import toolpathkit.setup.Setup;
import toolpathkit.tool.Tool;
import toolpathkit.tool.ToolLibrary;

class RapidClearanceTests {
  public static function main():Void {
    var tools = new ToolLibrary(); tools.set(new Tool(1, 0.03, 0.01));
    var span = new Provenance(7, 1, 8);
    var setup = new Setup("work", new Point3(0.1,0.2,0));
    var stocks = [{minimum:[0.09,0.19,0.0],maximum:[0.11,0.21,0.02]}];
    function protect(op:ToolpathOp, length:Float = 0.03):Array<ToolpathOp> {
      var program = new ToolpathProgram([ToolChange(1,span),ToolLengthOffset(1,length,span),op,
        Move(Cut,Line(new Point3(0,0,0.01),new Point3(0,0,0)),0.003,0.0001,span)],tools,[setup]);
      return RapidClearance.apply(program,stocks,[0.001,0.001,0.002]).ops;
    }
    var down = protect(Move(Rapid,Line(new Point3(0,0,0.06),new Point3(0,0,0.01)),0,0,span));
    if (down.length != 5) throw "Descent must split before the expanded stock";
    switch down[2] {
      case Move(Rapid,Line(start,end),_,tolerance,source):
        if (Math.abs(end.z-0.022)>1e-12 || start.z!=0.06 || tolerance!=0 || source.line!=7 || source==span)
          throw "Rapid must end at stock top plus stretch, with its source retained";
      case _: throw "Expected clear rapid portion";
    }
    switch down[3] {
      case Move(Cut,Line(start,end),feed,tolerance,source):
        if (Math.abs(start.z-0.022)>1e-12 || Math.abs(end.z-0.01)>1e-12 || feed!=0.003 || tolerance!=0 || source==span)
          throw "Close approach must actually run at cutting feed with an exact stop";
      case _: throw "Expected feed approach";
    }
    // G53 ignores G54/G43 geometry translation but the real 30 mm tool still hangs from the gauge line.
    var absolute = protect(MachineMove(Rapid,Line(new Point3(0.1,0.2,0.09),new Point3(0.1,0.2,0.04)),0,0,span),0.1);
    switch absolute[2] {
      case MachineMove(Rapid,Line(_,end),_,_,_): if (Math.abs(end.z-0.052)>1e-12) throw "G53 clearance used a work or H offset";
      case _: throw "Expected machine rapid split";
    }
    var wrongH = protect(Move(Rapid,Line(new Point3(0,0,0.06),new Point3(0,0,0.01)),0,0,span),0.02);
    switch wrongH[2] {
      case Move(Rapid,Line(_,end),_,_,_): if (Math.abs(end.z-0.032)>1e-12) throw "Clearance must use the loaded tool, even when G43 has the wrong length";
      case _: throw "Expected an approach adjusted for actual tool length";
    }
    // A traverse above the expanded stock stays rapid; a near-wall traverse feeds before its cutter can enter the band.
    var high = protect(Move(Rapid,Line(new Point3(-0.04,0,0.03),new Point3(0.04,0,0.03)),0,0,span));
    if (high.length!=4) throw "A clear rapid must stay unchanged";
    var across = protect(Move(Rapid,Line(new Point3(-0.04,0,0.01),new Point3(0.04,0,0.01)),0,0,span));
    if (across.length!=6) throw "Traverse must feed through the cutter-expanded stock and retain clear ends";
    switch across[2] {
      case Move(Rapid,Line(_,end),_,_,_): if (Math.abs(end.x+0.016)>1e-12) throw "Cutter radius or XY stretch missing";
      case _: throw "Expected rapid outside the wall";
    }
    var up = protect(Move(Rapid,Line(new Point3(0,0,0.01),new Point3(0,0,0.06)),0,0,span));
    switch up[2] { case Move(Cut,_,_,_,_): case _: throw "Retracting from the stock must feed until clear"; }
    for (kind in [MoveKind.Link, MoveKind.Retract]) {
      var linked = protect(Move(kind,Line(new Point3(0,0,0.01),new Point3(0,0,0.06)),0,0,span));
      switch linked[2] { case Move(Cut,_,_,_,_): case _: throw "All rapid-like move kinds need the feed clearance portion"; }
      switch linked[3] { case Move(outside,_,_,_,_): if (outside!=kind) throw "Clear portion must keep the authored move role"; case _: throw "Missing clear portion"; }
    }
    // Without a cutting feed, an unsafe rapid cannot silently become a zero-speed or full-speed approach.
    var refused = false;
    try { RapidClearance.apply(new ToolpathProgram([ToolChange(1,span),Move(Rapid,
      Line(new Point3(0,0,0.06),new Point3(0,0,0.01)),0,0,span)],tools,[setup]),stocks,[0.001,0.001,0.002]); }
    catch (_:Dynamic) refused = true;
    if (!refused) throw "A risky rapid with no machining feed must be refused";
    Sys.println("Rapid clearance checks passed: work/G53 frames, actual tool length, cutter radius, approach/retract/traverse and source identity");
  }
}

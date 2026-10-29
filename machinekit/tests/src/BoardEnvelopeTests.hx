import cadkit.modeling.Align;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.MachineAssembly;
import machinekit.electrical.BoardEnvelope;
import machinekit.hmi.PanelSolid;
import machinekit.pendant.*;
import sys.FileSystem;
import sys.io.File;

/** machinekit.electrical.BoardEnvelope on a small assembly, and the pendant board's record kept current. */
class BoardEnvelopeTests {
  static function check(ok:Bool, message:String):Void if (!ok) throw message;

  static function box(x0:Float, y0:Float, z0:Float, x1:Float, y1:Float, z1:Float):Part {
    var at = Part.box(x1 - x0, y1 - y0, z1 - z0, Min, Min, Min);
    var moved = at.placed(Location.translation(new Vector(x0, y0, z0)));
    at.close();
    return moved;
  }

  /** A 40 x 30 board, 1.6 thick, with a 2.7 hole at (10, 10); a wall over its right half 4.4 above it. */
  static function sample(interiorCut:Bool):MachineAssembly {
    var assembly = new MachineAssembly();
    assembly.addComponent("board", new PanelSolid("TEST-BOARD", "board", () -> {var plate = box(0, 0, 0, 40, 30, 1.6);
    var hole = Part.cylinderSpan(1.35, -1, 3, 10, 10);
    var cut = plate.subtract(hole);
    plate.close();
    hole.close();
    if (!interiorCut) return cut;
    var square = box(25, 10, -1, 30, 15, 3);
    var holed = cut.subtract(square);
    cut.close();
    square.close();
    return holed;
    }));
    assembly.addComponent("wall", new PanelSolid("TEST-WALL", "wall", () -> box(20, -5, 6.0, 45, 35, 8.0)));
    return assembly;
  }

  static function spec():BoardEnvelopeSpec {
    var s = new BoardEnvelopeSpec();
    s.board = "sample";
    s.member = "board";
    s.topUp = true;
    var hw = new EnvelopeHardware("M2.5 screw", 5.0, 1.5, EnvelopeSource.assumed("test"));
    s.holes = [new EnvelopeHole("h1", 10, 10, 2.7, false, true, hw, hw, EnvelopeSource.assumed("test"))];
    return s;
  }

  public static function run():Void {
    var text = BoardEnvelope.write(sample(false), spec());
    // Top faces +z: x right as is, y down from the top edge (y = 30), so the hole at (10, 10) is at (10, 20).
    check(text.indexOf('"pos_mm": [10, 20]') >= 0, "hole in the board frame:\n" + text);
    check(text.indexOf('"matrix": [\n') >= 0 && text.indexOf('"offset_mm": [0, 30]') >= 0, "frame offset:\n" + text);
    check(text.indexOf('"thickness_mm": {') >= 0 && text.indexOf('"value": 1.6') >= 0, "thickness from the solid");
    // The wall starts 4.4 above the top surface: the first cut in it is at 4.5, so the room recorded is 4.0, over the
    // board's part of the wall (x from 20 to 40, all of y).
    check(text.indexOf('"wall-top-8"') >= 0, "zone named after the member, side and level:\n" + text);
    check(text.indexOf('"rects_mm": [\n        [20, 0, 40, 30]') >= 0, "the wall's rectangle on the board:\n" + text);
    check(
      text.indexOf('"value": 4,') >= 0 || text.indexOf('"value": 4\n') >= 0,
      "4 mm of room under the wall:\n" + text
    );
    check(text.indexOf('"bottom"') >= 0 && text.indexOf('"wall-bottom') < 0, "nothing below the board");
    var refused = false;
    try BoardEnvelope.write(sample(true), spec()) catch (_:Dynamic) refused = true;
    check(refused, "an interior cut-out that is not a declared hole is refused");
    pendantRecord();
  }

  /** The pendant board's envelope, `pendant.envelope.json` in its pcbkit folder, is what the model gives today. */
  static function pendantRecord():Void {
    var pendantCase = ReferencePendant.pendantCase();
    var text = BoardEnvelope.write(
      ReferencePendant.assembly(),
      pendantCase.boardEnvelope("ReferencePendant, reference layout")
    );
    var path = repoRoot() + "/machinekit/examples/pendant/electronics/pendant/pendant.envelope.json";
    if (Sys.getEnv("PENDANT_ENVELOPE") == "write") File.saveContent(path, text);
    else if (!FileSystem.exists(path) || File.getContent(path)
      != text) throw "The pendant board's envelope differs from the case model; review and run with PENDANT_ENVELOPE=write:\n"
      + text;
  }

  static function repoRoot():String {
    var dir = Sys.getCwd();
    for (_ in 0...8) {
      if (FileSystem.exists(dir + "/machinekit/examples/pendant")) return dir;
      dir = dir + "/..";
    }
    throw "Cannot find machinekit/examples/pendant";
  }

  public static function main():Void run();
}

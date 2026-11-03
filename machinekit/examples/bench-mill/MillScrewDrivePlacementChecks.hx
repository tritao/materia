import machinekit.assembly.*;
import machinekit.milling.*;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.SupportedScrewDrive;
import machinekit.motion.SupportedScrewDrive.ScrewDriveAxis;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** `SupportedScrewDrive.poses` against the poses the mill's Y stage and Z axis place their drives at by hand. */
class MillScrewDrivePlacementChecks {
  static function same(expected:AssemblyFrame, actual:AssemblyFrame, label:String):Void {
    function near(a:Float, b:Float, what:String):Void {
      if (Math.abs(a - b) > 1e-6) throw '$label: $what is $b, the mill places it at $a';
    }
    near(expected.x, actual.x, "x");
    near(expected.y, actual.y, "y");
    near(expected.z, actual.z, "z");
    for (axis in [[1.0, 0, 0], [0.0, 1, 0], [0.0, 0, 1]]) {
      var a = AssemblyFrames.transformVector(expected, axis[0], axis[1], axis[2]);
      var b = AssemblyFrames.transformVector(actual, axis[0], axis[1], axis[2]);
      near(a.x, b.x, "axis x");
      near(a.y, b.y, "axis y");
      near(a.z, b.z, "axis z");
    }
  }

  static function compare(
    drive:SupportedScrewDrive,
    axis:ScrewDriveAxis,
    length:Float,
    mill:Map<String, AssemblyFrame>,
    ids:Array<Array<String>>,
    label:String
  ):Void {
    var mine = drive.poses(axis, length);
    for (pair in ids) {
      var expected = mill.get(pair[1]), actual = mine.get(pair[0]);
      if (expected == null || actual == null) throw '$label: missing ${pair[0]} / ${pair[1]}';
      same(expected, actual, label + " " + pair[1]);
    }
  }

  public static function main():Void {
    var reference = new BenchMill(false, false), f = MillFramePreview.layout(reference);
    var guide = LinearRailBlock.metric(BenchMill.RAIL, LinearRailBlock.SUPPLIER);
    var z = MillFramePreview.zLayout(reference, f, guide);
    // Y: screw along -Y at x = 0, supports standing on a seat below it (+Z away from the seat).
    var y = new MillXYStageLayout(f, z.spindleY, BenchMill.RAIL, guide, SupportedScrewDrive.m60dBk12());
    var yDrive = y.yDrive;
    if (yDrive == null) throw "Mill Y has no supported drive";
    var stage = new MillXYStage(y);
    compare(
      yDrive,
      {start: [0.0, y.yScrewStart, y.yScrewZ], direction: [0.0, -1, 0], normal: [0.0, 0, 1]},
      y.yDriveScrew.totalLength,
      stage.solvedPoses(),
      [["mount", "motorMountY"], ["motor", "motorY"], ["motor", "motorShaftY"], ["screw", "screwY"],
        ["coupling", "couplingY"], ["floating", "floatingBearingY"], ["ring", "floatingRingY"]],
      "Y drive"
    );
    // Z: screw down at x = 0, supports standing on the column's front (+Y away from it).
    var zl = MillFramePreview.zLayout(reference, f);
    var axis = new MillZAxis(zl);
    compare(
      zl.drive,
      {start: [0.0, zl.screwY, zl.screwTop], direction: [0.0, 0, -1], normal: [0.0, 1, 0]},
      zl.screw.totalLength,
      axis.solvedPoses(),
      [["mount", "motorMount"], ["motor", "motor"], ["motor", "motorShaft"], ["screw", "screw"], ["coupling", "coupling"],
        ["floating", "floatingBearing"], ["ring", "floatingRing"]],
      "Z drive"
    );
    // The seat holes come from the same datums: two bolts and two dowels for the floating support, as many bolts as the
    // mount has base holes plus two dowels for the fixed mount.
    var holes = yDrive.seatHoles(
      {start: [0.0, y.yScrewStart, y.yScrewZ], direction: [0.0, -1, 0], normal: [0.0, 0, 1]},
      y.yDriveScrew.totalLength
    );
    var tapped = holes.filter(h -> h.kind == "tapped"), dowels = holes.filter(h -> h.kind == "dowel");
    if (tapped.length != yDrive.fixed.spec.baseHoles.length + 2 || dowels.length != 4) throw 'Y seat holes: ${tapped.length} tapped, ${dowels.length} dowels';
    // Every tapped hole lies on the seat plane, a shaft height below the axis.
    for (hole in tapped) if (Math.abs(hole.position.z - (y.yScrewZ - yDrive.fixed.spec.shaftHeight)) > 1e-6 &&
      Math.abs(hole.position.z - (y.yScrewZ - yDrive.floating.spec.shaftHeight)) > 1e-6) throw "Y seat hole off the seat plane: " + hole.position.z;
    Sys.println("screw drive placement matches the mill's Y and Z drives");
  }
}

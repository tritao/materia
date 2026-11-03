import cadkit.modeling.Part;
import machinekit.assembly.*;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.MountingFace;
import machinekit.component.Solids;
import machinekit.milling.*;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.SupportedScrewDrive;
import machinekit.motion.SupportedScrewDrive.ScrewDriveAxis;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A 15 mm slab in x and y from -150 to 150 with its top at z = 15, cut for a part's seat when it is given one. */
class SeatSlab extends MachineComponent {
  final seat:Null<{face:MountingFace, pose:AssemblyFrame}>;

  public function new(name:String, ?seat:{face:MountingFace, pose:AssemblyFrame}) {
    super("SEAT-SLAB-" + name, "Seat slab " + name, "aluminium 6061", true);
    this.seat = seat;
  }

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part return Solids.building([], owned -> {
    var slab = Part.box(300, 300, 15);
    owned.push(slab);
    var cut = seat;
    if (cut == null) return slab;
    var local = cut.face.tool(15);
    var placed = PosedParts.moved(local, cut.pose);
    local.close();
    owned.push(placed);
    return Solids.cut(slab, [placed]);
  });
}

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

  /** A mount and a support stand on a slab: bare, they find no tapped holes under their feet; on a slab cut with `mountingFace`, they do. */
  static function seats(drive:SupportedScrewDrive):Void {
    var axis:ScrewDriveAxis = {start: [0.0, 0, 0], direction: [1.0, 0, 0], normal: [0.0, 0, 1]};
    for (kind in ["mount", "floating"]) {
      var part:MachineComponent = kind == "mount" ? drive.fixed : drive.floating;
      var height = kind == "mount" ? drive.fixed.spec.shaftHeight : drive.floating.spec.shaftHeight;
      // The axis stands a shaft height above the slab top (z = 15).
      var raised:ScrewDriveAxis = {start: [0.0, 0, 15 + height], direction: axis.direction, normal: axis.normal};
      var pose = drive.poses(raised, 100).get(kind);
      var face = part.mountingFace();
      if (pose == null || face == null) throw '$kind has no seat face';
      function on(slab:SeatSlab):AxisBuilder {
        var builder = new AxisBuilder();
        builder.place("slab", slab, AssemblyFrames.translation(0, 0, 0));
        builder.attach("part", part, pose, "slab");
        return builder;
      }
      var bare = AssemblyIntegrity.check(on(new SeatSlab(kind + "-bare")), {flangeCutouts: true});
      if ([for (item in bare.errors()) item.code].indexOf("integrity.mount.cutout") < 0) throw '$kind on a bare slab must lack its seat holes, found: ' + [for (item in bare.errors()) item.code];
      var cut = AssemblyIntegrity.check(
        on(new SeatSlab(kind + "-cut", {face: face, pose: AssemblyFrames.compose(pose, face.pose)})),
        {flangeCutouts: true}
      );
      var mount = [for (item in cut.errors()) if (StringTools.startsWith(item.code, "integrity.mount")) item.message];
      if (mount.length > 0) throw '$kind on a slab cut for it: ' + mount.join("; ");
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
    seats(yDrive);
    Sys.println("screw drive placement matches the mill's Y and Z drives");
  }
}

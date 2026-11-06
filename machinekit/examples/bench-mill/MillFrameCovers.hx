import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AxisBuilder;
import machinekit.component.MachineComponent;
import machinekit.component.BoltedFace;
import machinekit.cover.WayCover;
import machinekit.cover.WayCoverPlan;
import machinekit.cover.WayCoverCatalog;
import machinekit.cover.WayCoverMount;
import machinekit.cover.WayCoverMount.MountStock;
import machinekit.cover.WayCoverMount.MountAttachment;
import machinekit.milling.MillXYStage;
import machinekit.milling.MillCoverOutrigger;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Authored Small mill protection. Mounts refer to independently defined steel/casting datums. */
class MillFrameCovers {
  public static inline var WALL_X:Float = 400;

  static function stock(x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):MountStock return {
    x0: x0,
    x1: x1,
    y0: y0,
    y1: y1,
    z0: z0,
    z1: z1
  };
  static function face(
    x:Float,
    y:Float,
    z:Float,
    nx:Float,
    ny:Float,
    nz:Float,
    ax:Float = 1,
    ay:Float = 0,
    az:Float = 0
  ):AssemblyFrame return BoltedFace.frameOf(
    {x: x, y: y, z: z},
    {x: nx, y: ny, z: nz},
    {x: ax, y: ay, z: az}
  );

  /** Fresh parts: assembling a concept never drills a caller's reusable stage. */
  public static function stage(xy:MillXYStage):MachineAssembly {
    var l = xy.layout, result = xy.snapshot(), placer = new MillFrameCoverPlacer(result);
    for (side in [-1, 1]) {
      var id = side < 0 ? "coverXLeft" : "coverXRight", wallId = id + "Wall";
      var xShape = WayCoverPlan.boxCover(80, 116, 97, true);
      var plan = new WayCoverPlan(
        WayCoverCatalog.get(BenchMill.COVER_ROW),
        183,
        67,
        2 * l.xHalfTravel,
        WALL_X - 200 - l.xHalfTravel,
        xShape
      );
      var tabs:MountAttachment = {
        stock: [for (s in [-1, 1]) stock(s < 0 ? -88 : 74, s < 0 ? -74 : 88, -16, 0, -20, 0)],
        bolted : [for (s in [-1, 1]) BoltedFace.clamped(
          'mount$s',
          face(s * 74, -8, -10, s, 0, 0, 0, 0, 1),
          [{x : 0, y : 0}],
          5.5,
          14,
          "M5",
          null,
          false
        )],
        pinned : []
      };
      for (ySide in [-1, 1]) placer.attach(
        '$id-beam$ySide',
        new MillCoverOutrigger(side, ySide),
        F.translation(side * 225, l.centreY + ySide * 62, l.saddleBottom),
        "saddle"
      );
      placer.attach(
        wallId,
        new WayCoverMount('MILL-FRAME-X-WALL$side',
        plan.fullPlateHalf() + 4, plan.plateHigh() + 4, 14, [], [], 0, tabs),
        AxisBuilder.orient(side * WALL_X, l.centreY, l.xRailPlane, [0, 0, 1], [-side, 0, 0]),
        "saddle"
      );
      var cover = WayCover.installOn(
        result,
        placer,
        {
          id: id,
          row: BenchMill.COVER_ROW,
          fixedParent: wallId,
          movingParent: "table",
          leader: {id: "x", lower: -l.xHalfTravel, upper: l.xHalfTravel, initial: 0},
          origin: [side * WALL_X, l.centreY, l.xRailPlane],
          along: [-side, 0, 0],
          up: [0, 0, 1],
          width: 183,
          height: 67,
          space: plan.space,
          shape: xShape
        }
      );
      if (side < 0) {
        result.addServicePrerequisites("xScrewWithdrawal", cover.members().concat([wallId]));
        result.addServicePrerequisites("xMotorWithdrawal", cover.members().concat([wallId]));
      }
    }
    return result;
  }

  public static function finish(result:MachineAssembly):Void {
    var clamped = [for (entry in result.components()) if (StringTools.startsWith(entry.id, "xyStage/cover")) entry.id];
    result.deriveFasteners({clampedMembers: clamped});
    var wipers = 0;
    for (entry in result.components()) if (Std.isOfType(entry.component,
      machinekit.motion.LinearRailBlock)) wipers += 2;
    result.addBomItem(machinekit.cover.WayWiperCatalog.bomItem("WIPER-HG15-SCRAPER", wipers));
  }
}

private class MillFrameCoverPlacer implements machinekit.cover.WayCover.CoverMembers {
  final mill:MachineAssembly;
  final poses:Map<String, AssemblyFrame>;
  public function new(mill:MachineAssembly) {
    this.mill = mill;
    poses = mill.solvedPoses();
  }
  public function axisDirection(id:String):Array<Float> return switch (id) {
    case "x":
      [-1, 0, 0];
    case "y":
      [0, -1, 0];
    case _:
      throw id;
  };
  public function attach(id:String, c:MachineComponent, p:AssemblyFrame, parent:String):Void {
    hang(id, c, p, parent);
    mill.addMate(id + "-mount", "fixed", parent, "to-" + id, id, "attach-" + id);
  }
  public function hang(id:String, c:MachineComponent, p:AssemblyFrame, parent:String):Void {
    mill.addComponent(id, c);
    poses.set(id, p);
    var at = F.translation(p.x, p.y, p.z);
    mill.addMemberConnector(parent, "to-" + id, F.compose(F.inverse(poses.get(parent)), at));
    mill.addMemberConnector(id, "attach-" + id, F.compose(F.inverse(p), at));
  }
}


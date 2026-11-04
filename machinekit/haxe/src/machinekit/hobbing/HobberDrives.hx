package machinekit.hobbing;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.AxisBuilder;
import machinekit.motion.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Where one hobber axis's supported screw drive sits (work frame, mm) and what carries its nut. */
typedef HobberDriveAxis = {
  var id:String;
  var drive:SupportedScrewDrive;
  var axis:ScrewDriveAxis;
  /** Screw length. */
  var length:Float;
  /** From the screw start to the start of the nut part (its body begins there, the flange follows). */
  var nutStation:Float;
  /** The direction the nut part's local +Y points in: away from the carriage face the nut block stands on. */
  var nutUp:Array<Float>;
  /** Height of the screw axis above the face the nut block stands on. */
  var blockHeight:Float;
}

/** A hole in a part that carries or is carried by a drive: world position of its mouth, unit direction into the part. */
typedef HobberDriveHole = {
  var position:Array<Float>;
  var direction:Array<Float>;
  var diameter:Float;
  var depth:Float;
}

/**
 * The X, Y and Z screw drives of the hobber as `SupportedScrewDrive` placements: HZMotion M60D-BK12-C mount, BF12 floating
 * support and a Leadshine 400 W servo on each, Z braked. Screw supports stand on pads (X and Z) or brackets (Y) that exist
 * only under the two supports, so the nut block travels between them with room to spare; their seats carry the tapped
 * holes and dowels from `SupportedScrewDrive.seatHoles`, aligned to the nut and reamed at assembly.
 */
class HobberDrives {
  /** Width of the ground bar pads under the X and Z supports, mm. */
  public static inline final PAD_WIDTH = 80.0;

  /** Nut-part start from the nut body's centre station. */
  static function partStart(centre:Float):Float return centre - new BallNut().bodyLength / 2;

  static var xCache:Null<HobberDriveAxis>;
  static var yCache:Null<HobberDriveAxis>;
  static var zCache:Null<HobberDriveAxis>;

  /** X: screw along -x from the motor end at the back of the bed, supports on pads 15 mm above the bed insert. */
  public static function x():HobberDriveAxis {
    if (xCache == null) {
      var l = HobberXLayout.front();
      xCache = {id: "X", drive: SupportedScrewDrive.m60dBk12(true), axis: {start: [l.screwStart, 0.0, 180.0], direction: [-1.0, 0, 0], normal: [0.0, 0, 1]},
        length: l.screwLength, nutStation: partStart(l.nutStation), nutUp: [0.0, 0, 1], blockHeight: 80.0};
    }
    return xCache;
  }

  /** Y: screw along -y at z 508 (rel 145), supports on brackets standing on the swivel plate's top edge. */
  public static function y():HobberDriveAxis {
    if (yCache == null) yCache = {id: "Y", drive: SupportedScrewDrive.m60dBk12(true),
      axis: {start: [70.0, HobberYLayout.NUT_STATION, 508.0], direction: [0.0, -1.0, 0], normal: [1.0, 0, 0]},
      length: HobberYLayout.SCREW_LENGTH, nutStation: partStart(HobberYLayout.NUT_STATION), nutUp: [0.0, 0, -1], blockHeight: HobberYLayout.NUT_BLOCK_HEIGHT};
    return yCache;
  }

  static var zDriveCache:Null<SupportedScrewDrive>;

  /** Z with its screw at `y` across the machine (not cached). */
  public static function zAt(y:Float):HobberDriveAxis {
    if (zDriveCache == null) zDriveCache = SupportedScrewDrive.m60dBk12Braked(true);
    return {id: "Z", drive: zDriveCache,
      axis: {start: [HobberZLayout.AXIS_X, y, HobberZLayout.SHAFT_TOP], direction: [0.0, 0, -1.0], normal: [-1.0, 0, 0]},
      length: HobberZLayout.SHAFT_LENGTH, nutStation: partStart(HobberZLayout.NUT_STATION), nutUp: [-1.0, 0, 0], blockHeight: HobberZLayout.AXIS_X - 148};
  }

  /** Z: screw straight down at x 176, y `HobberZLayout.axisY()` (derived: clear of the A axis housing) from the motor at the top, supports on ground bar pads against the column. */
  public static function z():HobberDriveAxis {
    if (zCache == null) zCache = zAt(HobberZLayout.axisY());
    return zCache;
  }

  public static function all():Array<HobberDriveAxis> return [x(), y(), z()];

  public static function poses(d:HobberDriveAxis):Map<String, AssemblyFrame> {
    var at = d.drive.poses(d.axis, d.length, d.nutStation);
    var nut = at.get("nut");
    if (nut == null) throw "Hobber drive lost its nut pose";
    at.set("nut", AxisBuilder.orient(nut.x, nut.y, nut.z, d.nutUp, d.axis.direction));
    return at;
  }

  /** The nut block for this axis, on the barrel side of the nut's flange, its base end `blockHeight` from the axis. */
  public static function nutBlock(d:HobberDriveAxis, nut:BallNut):NutBlock return new NutBlock("HOBBER-" + d.id + "-NUT-BLOCK", nut, d.blockHeight);

  /** The nut block's world frame: the nut's pose composed with the nut's flange seat. */
  public static function nutBlockPose(d:HobberDriveAxis, nut:BallNut):AssemblyFrame {
    var face = nut.mountingFace();
    if (face == null) throw "Hobber nut has no mounting face";
    return F.compose(poses(d).get("nut"), face.pose);
  }

  /** The block's two carriage bolts: mouths on its base end, direction out of the block into the carriage. */
  public static function nutBlockHoles(d:HobberDriveAxis, nut:BallNut, block:NutBlock):Array<HobberDriveHole> {
    var pose = nutBlockPose(d, nut), out = F.transformVector(pose, 0, -1, 0);
    return [for (x in block.baseHoleXs()) {
      var p = F.transformPoint(pose, x, -block.centreHeight, block.thickness / 2);
      {position: [p.x, p.y, p.z], direction: [out.x, out.y, out.z], diameter: 6.6, depth: 0.0};
    }];
  }

  /** The tapped holes and dowels `SupportedScrewDrive.seatHoles` asks for under one support (`"fixed"` or `"floating"`). */
  public static function seatHoles(d:HobberDriveAxis, support:String):Array<HobberDriveHole> {
    return [for (h in d.drive.seatHoles(d.axis, d.length)) if (h.support == support) {
      var drill = h.kind == "tapped" ? machinekit.standard.SocketHeadCapScrew.catalog().get(h.thread).tapDrill : h.diameter;
      {position: [h.position.x, h.position.y, h.position.z], direction: [h.direction.x, h.direction.y, h.direction.z], diameter: drill, depth: h.depth};
    }];
  }

  /** The same holes as `seatHoles`, with their kind and thread, for a drawing's hole table (world frame). */
  public static function seatRouteHoles(d:HobberDriveAxis, support:String):Array<machinekit.component.ManufacturingRoute.RouteHole> {
    var counts = new Map<String, Int>(), result = [];
    for (h in d.drive.seatHoles(d.axis, d.length)) if (h.support == support) {
      var spec = h.kind == "tapped" ? machinekit.standard.SocketHeadCapScrew.catalog().get(h.thread) : null;
      var n = (counts.get(h.kind) ?? 0) + 1;
      counts.set(h.kind, n);
      result.push({
        id: (h.kind == "dowel" ? "D" : "T") + n,
        kind: h.kind,
        thread: h.thread,
        diameter: spec == null ? h.diameter : spec.tapDrill,
        nominal: h.diameter,
        fit: h.kind == "dowel" ? "H7" : null,
        depth: h.depth,
        position: [h.position.x, h.position.y, h.position.z],
        direction: [h.direction.x, h.direction.y, h.direction.z]
      });
    }
    return result;
  }

  /**
   * The relief the fixed pad needs under the motor mount: the part of the mount's drawn body that hangs below its base plane (the
   * motor flange plate hangs 5 mm below it), measured on the mount's geometry and placed by its pose, plus 0.5 mm all round, as
   * {x0, x1, y0, y1, z0, z1} in world mm.
   */
  public static function mountRelief(d:HobberDriveAxis):Array<Float> {
    var mount = d.drive.fixed, body = mount.geometry();
    var raw = Part.box(400, 10, 400), slab = raw.translated(new Vector(0, -5, -100));
    raw.close();
    var hanging = body.intersect(slab);
    body.close();
    slab.close();
    var bounds = hanging.shape.bounds(), lo = bounds.get_min(), hi = bounds.get_max();
    hanging.close();
    var pose = poses(d).get("mount"), result = [1e9, 1e9, 1e9, -1e9, -1e9, -1e9];
    for (x in [lo.get_x(), hi.get_x()]) for (y in [lo.get_y() - 0.5, 0.0]) for (z in [lo.get_z(), hi.get_z()]) {
      var p = F.transformPoint(pose, x, y, z), q = [p.x, p.y, p.z];
      for (i in 0...3) {
        result[i] = Math.min(result[i], q[i] - 0.5);
        result[i + 3] = Math.max(result[i + 3], q[i] + 0.5);
      }
    }
    return result;
  }

  /**
   * The box of a pad under one support: the holes' bounds plus `margin` along the plane, `width` across the screw
   * and `thickness` down from the seat plane, as {x0, x1, y0, y1, z0, z1}.
   */
  public static function padBounds(d:HobberDriveAxis, support:String, width:Float, thickness:Float, margin:Float = 18):Array<Float> {
    var holes = seatHoles(d, support), lo = [1e9, 1e9, 1e9], hi = [-1e9, -1e9, -1e9];
    for (h in holes) for (i in 0...3) {
      lo[i] = Math.min(lo[i], h.position[i]);
      hi[i] = Math.max(hi[i], h.position[i]);
    }
    var n = d.axis.normal, a = d.axis.direction, result = [0.0, 0, 0, 0, 0, 0];
    for (i in 0...3) {
      if (Math.abs(n[i]) > 0.5) {
        // Normal axis: the seat plane is where the holes are; the pad lies on the far side of it from the screw.
        var plane = lo[i];
        result[i] = n[i] > 0 ? plane - thickness : plane;
        result[i + 3] = n[i] > 0 ? plane : plane + thickness;
      } else if (Math.abs(a[i]) > 0.5) {
        result[i] = lo[i] - margin;
        result[i + 3] = hi[i] + margin;
      } else {
        var centre = (lo[i] + hi[i]) / 2;
        result[i] = centre - width / 2;
        result[i + 3] = centre + width / 2;
      }
    }
    return result;
  }
}

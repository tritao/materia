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

  /** X guard threads use the edge lands of 100 mm stock on the existing 100 mm insert. */
  public static inline final X_PAD_WIDTH = 100.0;

  /** Nut-part start from the nut body's centre station. */
  static function partStart(centre:Float):Float return centre - new BallNut().bodyLength / 2;

  /** The three axes, each built once and shared: no caller changes them. */
  static final xCache = new machinekit.cache.Once<HobberDriveAxis>();
  static final yCache = new machinekit.cache.Once<HobberDriveAxis>();
  static final zCache = new machinekit.cache.Once<HobberDriveAxis>();

  /** X: screw along -x from the motor end at the back of the bed, supports on pads 15 mm above the bed insert. */
  public static function x():HobberDriveAxis return xCache.get(() -> {
    var l = HobberXLayout.front();
    var axis:HobberDriveAxis = {id: "X", drive: SupportedScrewDrive.m60dBk12(true), axis: {start: [l.screwStart, 0.0, 180.0], direction: [-1.0, 0, 0], normal: [0.0, 0, 1]},
      length: l.screwLength, nutStation: l.nutStation, nutUp: [0.0, 0, 1], blockHeight: 80.0};
    pin(axis);
    return axis;
  });

  /** Y is the library module's screw in the unchanged hobber work frame. */
  public static function y():HobberDriveAxis return yCache.get(()->{
    var s=HobberYLayout.moduleSpec(),pose=HobberYLayout.modulePose(),local=s.axis();
    var p=F.transformPoint(pose,local.start[0],local.start[1],local.start[2]);
    var d=F.transformVector(pose,local.direction[0],local.direction[1],local.direction[2]);
    var n=F.transformVector(pose,local.normal[0],local.normal[1],local.normal[2]);
    var up=F.transformVector(F.compose(pose,s.nutPose()),0,1,0);
    return {id:"Y",drive:s.drive,axis:{start:[p.x,p.y,p.z],direction:[d.x,d.y,d.z],normal:[n.x,n.y,n.z]},length:s.screwLength,nutStation:s.nutStation,nutUp:[up.x,up.y,up.z],blockHeight:s.carrierHeight()};
  });
  public static function jointId(id:String):String return id=="Y"?HobberYLayout.JOINT:id;
  public static function member(id:String,role:String):String {
    if(id=="Y")return "Ymodule/"+switch(role){case "mount":"motorMount";case "floating":"floatingBearing";case "ring":"floatingRing";case _:role;};
    return id+switch(role){case "mount":"fixed";case "nutCarrier":"nutBlock";case "motorShaft":"motor-shaft";case "ring":"floatingRing";case _:role;};
  }
  public static function railMember(id:String,side:String):String return id=="Y"?"Ymodule/rail"+side:"rail"+id+side;

  /** Both supports of an axis are located by dowels, reamed through the foot and the seat at assembly (`ScrewSupportUnit.footPinned`). */
  static function pin(axis:HobberDriveAxis):Void {
    axis.drive.fixed.footPinned = true;
    axis.drive.floating.footPinned = true;
  }

  static final zDriveCache = new machinekit.cache.Once<SupportedScrewDrive>();

  /** Z with its screw at `y` across the machine (not cached). */
  public static function zAt(y:Float):HobberDriveAxis {
    var drive = zDriveCache.get(() -> {
      var built = SupportedScrewDrive.m60dBk12Braked(true);
      built.fixed.footPinned = true;
      built.floating.footPinned = true;
      return built;
    });
    return {id: "Z", drive: drive,
      axis: {start: [HobberZLayout.AXIS_X, y, HobberZLayout.SHAFT_TOP], direction: [0.0, 0, -1.0], normal: [-1.0, 0, 0]},
      length: HobberZLayout.SHAFT_LENGTH, nutStation: partStart(HobberZLayout.NUT_STATION), nutUp: [-1.0, 0, 0], blockHeight: HobberZLayout.AXIS_X - 148};
  }

  /** Z: screw straight down at x 176, y `HobberZLayout.axisY()` (derived: clear of the A axis housing) from the motor at the top, supports on ground bar pads against the column. */
  public static function z():HobberDriveAxis {
    return zCache.get(() -> zAt(HobberZLayout.axisY()));
  }

  public static function all():Array<HobberDriveAxis> return [x(), y(), z()];

  public static function poses(d:HobberDriveAxis):Map<String, AssemblyFrame> {
    if(d.id=="Y") {var at=d.drive.poses(d.axis,d.length,d.nutStation);at.set("nut",F.compose(HobberYLayout.modulePose(),HobberYLayout.moduleSpec().nutPose()));return at;}
    var at = d.drive.poses(d.axis, d.length, d.nutStation);
    var nut = at.get("nut");
    if (nut == null) throw "Hobber drive lost its nut pose";
    at.set("nut", AxisBuilder.orient(nut.x, nut.y, nut.z, d.nutUp, d.id == "X" ? [for(c in d.axis.direction) -c] : d.axis.direction));
    return at;
  }

  /** X and Z nut blocks; Y uses the module's integral nut carrier. */
  public static function nutBlock(d:HobberDriveAxis, nut:BallNut):NutBlock {
    if(d.id=="Y") throw "Y uses ScrewLinearModule nutCarrier";
    return new NutBlock("HOBBER-"+d.id+"-NUT-BLOCK",nut,d.blockHeight,null,null,NutBlock.BASE_EDGE,NutBlock.BASE_DEPTH,true);
  }

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
    var mount = d.drive.fixed, pose = poses(d).get("mount");
    return reliefs.get(mount.geometryKey() + "|" + haxe.Json.stringify(pose), () -> measureRelief(mount, pose));
  }

  /** Reliefs by the mount's geometry and pose: the saddle's pad and pieces ask for the same mount's relief, and each answer
   * cuts the supplier body. */
  static final reliefs = new machinekit.cache.Memo<Array<Float>>("hobber-mount-reliefs", 64, bounds -> bounds.copy());

  static function measureRelief(mount:machinekit.component.MachineComponent, pose:materia.assembly.AssemblyRecord.AssemblyFrame):Array<Float> {
    var body = mount.geometry();
    var raw = Part.box(400, 10, 400), slab = raw.translated(new Vector(0, -5, -100));
    raw.close();
    var hanging = body.combineKept(slab, cadkit.modeling.Mode.Intersect);
    body.close();
    slab.close();
    var bounds = hanging.shape.bounds(), lo = bounds.get_min(), hi = bounds.get_max();
    hanging.close();
    var result = [1e9, 1e9, 1e9, -1e9, -1e9, -1e9];
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
  /** Matched short ground bars: height is the bought support's foot plane
   * minus the cast insert plane, never a new typed axis height. */
  public static function barThickness(d:HobberDriveAxis):Float {
    var kind=d.id=="X"?"bed":"column",row=FoundationLayout.placements(kind).filter(r->r.id=="driveSeat")[0];
    var top=F.transformPoint(F.compose(FoundationLayout.worldPose(kind),row.pose),0,0,row.part.boxDimensions(row.id).height);
    var p=seatHoles(d,"fixed")[0].position,n=d.axis.normal;
    var thickness=(p[0]-top.x)*n[0]+(p[1]-top.y)*n[1]+(p[2]-top.z)*n[2];
    return Math.round(thickness*1000)/1000;
  }
  public static function barBounds(d:HobberDriveAxis,support:String):Array<Float> {
    var ni=0,ai=0;for(i in 0...3){if(Math.abs(d.axis.normal[i])>0.5)ni=i;if(Math.abs(d.axis.direction[i])>0.5)ai=i;}
    var ci=3-ni-ai,holes=seatRouteHoles(d,support).filter(h->h.kind=="tapped");
    var alo=1e9,ahi=-1e9,clo=1e9,chi=-1e9,diameter=0.0;
    for(h in holes){alo=Math.min(alo,h.position[ai]);ahi=Math.max(ahi,h.position[ai]);clo=Math.min(clo,h.position[ci]);chi=Math.max(chi,h.position[ci]);diameter=Math.max(diameter,h.nominal);}
    var width=Math.ceil((chi-clo+3*diameter)/5)*5,length=Math.ceil((ahi-alo+3*diameter)/5)*5;
    if(support=="floating") {
      var face=d.drive.floating.mountingFace();
      if(face==null)throw "Floating support needs a declared bearing face";
      // Cover the declared foot contact extent as well as the bolt-edge lands.
      length=Math.ceil(Math.max(length,Math.max(d.drive.floating.length,face.height))/5)*5;
    }
    return padBounds(d,support,width,barThickness(d),(length-(ahi-alo))/2);
  }
  public static function barMember(axis:String,support:String):String return axis+"drive"+(support=="fixed"?"Fixed":"Floating")+"Bar";
  public static function barDesign(axis:String,support:String):String return "GE-"+axis+"-"+support+"-drive-bar";

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

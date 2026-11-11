import machinekit.assembly.PosedParts;
import machinekit.assembly.PosedParts.PartBox;
import machinekit.enclosure.EnclosureLayout;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Swept machine and loading envelopes used to fit the unchanged enclosure. */
class HobberEnclosureFit {
  /** Bounds come from the declared extrusion profiles, without constructing casting solids. */
  public static function castingBox(body:machinekit.structural.MineralCasting, pose:AssemblyFrame):PartBox {
    var low=[Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY];
    var high=[Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY];
    for(prism in body.castingEnvelope()) {
      var frame=F.compose(pose,prism.pose);
      for(point in prism.profile) for(z in [0.0,prism.height]) {
        var p=F.transformPoint(frame,point.x,point.y,z);
        low[0]=Math.min(low[0],p.x); low[1]=Math.min(low[1],p.y); low[2]=Math.min(low[2],p.z);
        high[0]=Math.max(high[0],p.x); high[1]=Math.max(high[1],p.y); high[2]=Math.max(high[2],p.z);
      }
    }
    if(!Math.isFinite(low[0]))throw "Missing declared casting envelope";
    return {minX:low[0],minY:low[1],minZ:low[2],maxX:high[0],maxY:high[1],maxZ:high[2]};
  }
  static function union(a:PartBox, b:PartBox):PartBox return {
    minX: Math.min(a.minX, b.minX),
    minY: Math.min(a.minY, b.minY),
    minZ: Math.min(a.minZ, b.minZ),
    maxX: Math.max(a.maxX, b.maxX),
    maxY: Math.max(a.maxY, b.maxY),
    maxZ: Math.max(a.maxZ, b.maxZ)
  };
  static function grow(b:PartBox, p:Float):PartBox return {
    minX: b.minX - p,
    minY: b.minY - p,
    minZ: b.minZ - p,
    maxX: b.maxX + p,
    maxY: b.maxY + p,
    maxZ: b.maxZ + p
  };
  static function radius(b:PartBox, x:Float, y:Float, z:Float):Float {
    var dx = Math.max(Math.abs(b.minX - x), Math.abs(b.maxX - x)), dy = Math.max(
      Math.abs(b.minY - y),
      Math.abs(b.maxY - y)
    ), dz = Math.max(
      Math.abs(b.minZ - z),
      Math.abs(b.maxZ - z)
    );
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }
  public static function external(id:String):Bool return id == "electricalPanel"
    || id == "powerSupply" || StringTools.endsWith(
      id,
      "-driver"
    );
  public static function fit(machine:FlexsplineHobber): {
    layout: EnclosureLayout,
    samples: Int,
    paddingMm: Float,
    excluded: Array<String>
  } {
    var model = new cadkit.modeling.AssemblyModel("mm");
    machine.addTo(model, "");
    var state = model.initialState(), definition = state.definition;
    var parents:Map<String, String> = [], rotors:Map<String, String> = [];
    for (j in definition.joints) {
      parents.set(j.child, j.parent);
      if (j.id == FlexsplineHobber.B || j.id == FlexsplineHobber.C) rotors.set(j.child, j.id);
    }
    function rotor(id:String):Null<String> {
      var current = id;
      while (current != null) {
        if (rotors.exists(current)) return rotors.get(current);
        current = parents.get(current);
      }
      return null;
    }
    var cache = new PosedParts(), members = machine.components(),
      swept:Null<PartBox> = null, plinth:Null<PartBox> = null, samples = 0, radiusBound = 0.0;
    try {
      var ranges = [for (t in machine.travel)[t.lower, t.upper]];
      var aj = [for (j in definition.joints) if (j.id == FlexsplineHobber.A) j][0];
      if (aj.limits.lower == null || aj.limits.upper == null) throw "Enclosure needs bounded swivel travel";
      var lower:Float = aj.limits.lower,
        upper:Float = aj.limits.upper, steps = Std.int(Math.ceil((upper - lower) / (Math.PI / 180)));
      if (steps < 1) throw "Enclosure needs nonzero swivel travel";
      var step = (upper - lower) / steps;
      for (x in ranges[0]) for (y in ranges[1]) for (z in ranges[2]) for (a in 0...steps + 1) {
        state.setJoint("X", x);
        state.setJoint(FlexsplineHobber.Y, y);
        state.setJoint("Z", z);
        state.setJoint(FlexsplineHobber.A, lower + a * step);
        state.forwardKinematics();
        var pivot = state.worldConnector(aj.parent, aj.parentConnector);
        for (m in members) if (!external(m.id)) {
          var pose = state.worldPose(m.id);
          var box = Std.isOfType(m.component, machinekit.structural.MineralCasting)
            ? castingBox(cast(m.component, machinekit.structural.MineralCasting), pose)
            : cache.box(m.component, pose);
          var r = rotor(m.id);
          if (r != null) {
            var j = [for (j in definition.joints) if (j.id == r) j][0], p = state.worldConnector(
              j.parent,
              j.parentConnector
            ), rr = radius(
              box,
              p.x,
              p.y,
              p.z
            );
            box = {minX: p.x - rr, minY: p.y - rr, minZ: p.z - rr, maxX: p.x + rr, maxY: p.y + rr, maxZ: p.z + rr};
          }
          radiusBound = Math.max(radiusBound, radius(box, pivot.x, pivot.y, pivot.z));
          swept = swept == null ? box : union(swept, box);
          if (m.id == "bed/casting") plinth = box;
        }
        samples++;
      }
      if (swept == null || plinth == null) throw "Missing machine/plinth envelope";
      var padding = radiusBound * step / 2;
      swept = grow(swept, padding);
      // OD140 side-jaw reservation matches HobberLoadingChecks; the robot itself is not modeled.
      var r = Math.max(70, machine.spec.tipRadius()), origin = machine.workOrigin;
      var loading:PartBox = {
        minX: origin.x - r - 25,
        maxX: origin.x + r + 25,
        minY: Math.min(origin.y - hobber.HobberSkeleton.reference().frontApproach - r - 80, swept.minY - 225),
        maxY: origin.y + 15,
        minZ: origin.z - 60,
        maxZ: origin.z + 80 + hobber.HobberSkeleton.reference().frontLift
      };
      // Tray overlaps the casting perimeter by the rebate width.
      var a = machinekit.enclosure.TraySeat.WIDTH;
      var opening:PartBox = {
        minX: plinth.minX + a,
        minY: plinth.minY + a,
        minZ: plinth.minZ,
        maxX: plinth.maxX - a,
        maxY: plinth.maxY - a,
        maxZ: plinth.maxZ
      };
      var floor = plinth.maxZ;
      var result = {
        layout: new EnclosureLayout({
          swept: swept,
          loading: loading,
          plinth: opening,
          floorZ: floor,
          margin: 75,
          openingWidth: 400,
          opposedDoors: true
        }),
        samples: samples,
        paddingMm: padding,
        excluded: [for (m in members) if (external(m.id)) m.id]
      };
      cache.close();
      return result;
    } catch (error:Dynamic) {
      cache.close();
      throw error;
    }
  }
}

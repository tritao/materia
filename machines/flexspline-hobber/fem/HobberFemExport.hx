import hobber.*;
import hobber.HobberSkeleton;
import machinekit.hobbing.*;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames as F;

/** Read-only export for the FEM checks (fem/hobber_fem.py): STEP of the four parts in their authored frame, their holes, and the masses and
    centres the loads need (servo group on the cradle, head on the A flange), all in the authored frame (work frame, z measured from HobberSkeleton.reference().nativeHobCentreZ).
    Usage: hl main.hl <output dir>. Nothing here changes the model. */
class HobberFemExport {
  static function r(v:Float, digits:Int = 4):Float {
    var f = Math.pow(10, digits);
    return Math.round(v * f) / f;
  }

  /** One Y plate-path model; all coordinates are in the included module frame. */
  static function exportY(machine:FlexsplineHobber,dir:String):Void {
    var poses=machine.solvedPoses(),base=poses.get("Ymodule/base");if(base==null)throw "Missing Y base";
    var inverse=F.inverse(base),parts:Array<Dynamic>=[];
    for(role in ["base","carriage"]) {
      var member=machine.components().filter(m->m.id=="Ymodule/"+role)[0],shape=member.component.geometry();
      shape.shape.exportStep(dir+"/Y-"+role+".step");
      parts.push({name:role,volume:shape.volume(),holes:member.component.derivedRouteHoles()});shape.close();
    }
    function point(p:{x:Float,y:Float,z:Float}):Array<Float> {
      var v=F.transformPoint(inverse,p.x,p.y,p.z);return [v.x,v.y,v.z];
    }
    var held:Array<Array<Float>>=[],head:Array<Dynamic>=[],blocks:Array<Dynamic>=[];
    var derived=machine.derivedFasteners();if(derived==null)throw "Y FE needs derived interfaces";
    for(joint in derived.joints) {
      if(joint.clampedMember=="Ymodule/base"&&joint.receivingMember=="swivel/output")for(p in joint.worldHoles)held.push(point(p));
      if(joint.clampedMember=="Ymodule/carriage") {
        var diameter=machinekit.standard.SocketHeadCapScrew.catalog().get(joint.size).diameter;
        if(joint.receivingMember=="spindleSeat"||joint.receivingMember=="outboardCarrier")for(p in joint.worldHoles)head.push({point:point(p),radius:1.5*diameter});
        if(StringTools.startsWith(joint.receivingMember,"Ymodule/block"))for(p in joint.worldHoles)blocks.push({point:point(p),radius:1.5*diameter});
      }
    }
    var hob=F.compose(poses.get("hob"),machine.hob.connector("centre").frame),s=machine.yModule.spec;
    var data={parts:parts,held:held,head:head,blocks:blocks,hob:point({x:hob.x,y:hob.y,z:hob.z}),
      centre:s.centre,railStart:s.railStart(),railLength:s.railLength,wallWidth:s.wallWidth,railSpacing:s.railSpacing,riserCentres:[s.wallY(-1),s.wallY(1)],
      baseThickness:s.baseThickness,carriageBottom:s.carriageBottom()};
    sys.io.File.saveContent(dir+"/y-module.json",haxe.Json.stringify(data,null,"  "));
    Sys.println("exported Y plate-path geometry and joint-derived boundary patches");
  }

  static function main() {
    var args = Sys.args();
    if (args.length < 1 || args.length > 2) throw "usage: hl main.hl <output dir>";
    var dir = args[0];
    sys.FileSystem.createDirectory(dir);
    var machine = new FlexsplineHobber();
    if(args.length==2&&args[1]=="y-module") {exportY(machine,dir);return;}
    var poses = machine.solvedPoses(), back = F.inverse(hobber.HobberFrame.workToMachine()), seatZ = HobberSkeleton.reference().nativeHobCentreZ;
    var partList:Array<Dynamic> = [];
    var out:Dynamic = {parts: partList};
    function at(id:String, x:Float, y:Float, z:Float) return F.transformPoint(F.compose(back, poses.get(id)), x, y, z);
    function member(designation:String):MachineComponent {
      for (entry in machine.components()) if (entry.component.designation == designation) return entry.component;
      throw "no part " + designation;
    }
    for (design in ["GE-X-carriage", "GE-Z-saddle", "GE-spindle-seat", "GE-motor-cradle"]) {
      var saddle = cast(member("HOBBER-" + design), HobberMadePart);
      var geo = saddle.geometry(), entryId = "";
      for (entry in machine.components()) if (entry.component == cast saddle) entryId = entry.id;
      var o = at(entryId, 0, 0, 0), ax = at(entryId, 1, 0, 0), ay = at(entryId, 0, 1, 0), az = at(entryId, 0, 0, 1);
      var pose = {id: entryId, origin: [r(o.x), r(o.y), r(o.z - seatZ)], axes: [[r(ax.x - o.x, 6), r(ax.y - o.y, 6), r(ax.z - o.z, 6)], [r(ay.x - o.x, 6), r(ay.y - o.y, 6), r(ay.z - o.z, 6)],
        [r(az.x - o.x, 6), r(az.y - o.y, 6), r(az.z - o.z, 6)]]};
      geo.shape.exportStep('$dir/$design.step');
      var props = saddle.massProperties();
      var holes = [for (h in saddle.holes()) {id: h.id, kind: h.kind, thread: h.thread, diameter: r(h.diameter), depth: r(h.depth),
        position: [for (v in h.position) r(v)], direction: [for (v in h.direction) r(v, 6)], slotLength: h.slotLength == null ? 0 : r(h.slotLength)}];
      partList.push({name: design, pose: pose, volume: geo.volume(), mass: props.mass, comLocal: [props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z], holes: holes});
      geo.close();
      Sys.println("exported " + design);
    }
    // XC: export the actual separate members, including inherited drilling.
    // The solid FE must use these shapes rather than reconstructing an obsolete
    // integral rib carriage from nominal boxes.
    for (entry in machine.components()) {
      if (!StringTools.startsWith(entry.id, "Xrib-")) continue;
      var component = entry.component;
      var shape = component.geometry();
      shape.shape.exportStep('$dir/${entry.id}.step');
      var props = component.massProperties();
      var origin = at(entry.id, 0, 0, 0);
      var ax = at(entry.id, 1, 0, 0);
      var ay = at(entry.id, 0, 1, 0);
      var az = at(entry.id, 0, 0, 1);
      partList.push({name: entry.id, pose: {id: entry.id,
        origin: [r(origin.x), r(origin.y), r(origin.z - seatZ)],
        axes: [[r(ax.x-origin.x,6),r(ax.y-origin.y,6),r(ax.z-origin.z,6)],
          [r(ay.x-origin.x,6),r(ay.y-origin.y,6),r(ay.z-origin.z,6)],
          [r(az.x-origin.x,6),r(az.y-origin.y,6),r(az.z-origin.z,6)]]},
        volume: shape.volume(), mass: props.mass,
        comLocal: [props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z],
        material: component.materialSpec(), recipeKey: component.bom.valuesKey,
        holes: component.derivedRouteHoles()});
      shape.close();
      Sys.println("exported " + entry.id);
    }
    // Component poses: where each part's authored frame lies in the work frame (to see the offset to the authored frame).
    // Servo group on the cradle: exactly the members and weights HobberFrameChecks.spindleDrive uses.
    var ids = ["spindle/motor", "spindle/motorShaft", "spindle/motorPulley", "spindle/motorEncoder", "motorCradle"], servo = {mass: 0.0, x: 0.0, y: 0.0, z: 0.0}, parts:Array<Dynamic> = [];
    for (m in machine.components()) {
      if (ids.indexOf(m.id) < 0) continue;
      var weight = m.id == "motorCradle" ? 1.0 / 3 : 1.0, props = m.component.massProperties(), p = at(m.id, props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      servo.mass += weight * props.mass;
      servo.x += weight * props.mass * p.x;
      servo.y += weight * props.mass * p.y;
      servo.z += weight * props.mass * (p.z - seatZ);
      parts.push({id: m.id, mass: props.mass, weight: weight, com: [r(p.x), r(p.y), r(p.z - seatZ)]});
    }
    var cradle = member("HOBBER-GE-motor-cradle").massProperties();
    var cc = at("motorCradle", cradle.centreOfMass.x, cradle.centreOfMass.y, cradle.centreOfMass.z);
    out.cradle = {bolts: [for (b in HobberMotorCradle.cradleFloorBolts()) [b.x, b.y]], rest: HobberMotorCradle.cradleRest()};
    out.servo = {mass: servo.mass, com: [r(servo.x / servo.mass), r(servo.y / servo.mass), r(servo.z / servo.mass)], members: parts, motorBody: {face: machine.spindle.motor.rating.bodyDiameter, length: machine.spindle.motor.rating.bodyLength},
      cradleComWork: [r(cc.x), r(cc.y), r(cc.z - seatZ)], cradleComAuthored: [r(cradle.centreOfMass.x), r(cradle.centreOfMass.y), r(cradle.centreOfMass.z)]};
    // Head on the A flange.
    var def = machine.definition(), turning = new Map<String, Bool>();
    turning.set("swivel/output", true);
    var changed = true;
    while (changed) {
      changed = false;
      for (j in def.joints) if (turning.exists(j.parent) && !turning.exists(j.child)) {
        turning.set(j.child, true);
        changed = true;
      }
    }
    var mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
    for (m in machine.components()) if (turning.exists(m.id)) {
      var props = m.component.massProperties(), p = at(m.id, props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      mass += props.mass;
      mx += props.mass * p.x;
      my += props.mass * p.y;
      mz += props.mass * p.z;
    }
    var tool = F.transformPoint(F.compose(back, F.compose(poses.get("hob"), machine.hob.connector("centre").frame)), 0, 0, 0);
    out.head = {mass: mass, com: [r(mx / mass), r(my / mass), r(mz / mass - seatZ)], outputFaceX: HobberSkeleton.reference().swivelNativeX, hob: [r(tool.x), r(tool.y), r(tool.z - seatZ)]};
    out.seatZ = seatZ;
    // Z moving mass (everything on the Z joint) for the saddle's inertial loads.
    var moving = new Map<String, Bool>();
    moving.set("blockZLeft", true);
    changed = true;
    while (changed) {
      changed = false;
      for (j in def.joints) if (moving.exists(j.parent) && !moving.exists(j.child)) {
        moving.set(j.child, true);
        changed = true;
      }
    }
    var zm = 0.0, zx = 0.0, zy = 0.0, zz = 0.0;
    for (m in machine.components()) if (moving.exists(m.id)) {
      var props = m.component.massProperties(), p = at(m.id, props.centreOfMass.x, props.centreOfMass.y, props.centreOfMass.z);
      zm += props.mass;
      zx += props.mass * p.x;
      zy += props.mass * p.y;
      zz += props.mass * p.z;
    }
    out.zMoving = {mass: zm, com: [r(zx / zm), r(zy / zm), r(zz / zm - seatZ)]};
    // Use the actual column steel contacts; no second footprint declaration.
    var footprints:Array<{x0:Float,x1:Float,y0:Float,y1:Float}>=[];
    var xmin=Math.POSITIVE_INFINITY;
    var xmax=Math.NEGATIVE_INFINITY;
    var toDeck=F.inverse(poses.get("Xbridge"));
    for (entry in machine.components()) if (StringTools.startsWith(entry.id,"column/")
      && Std.isOfType(entry.component,machinekit.structural.CastInsert)
      && entry.component.parameterToken("role")=="column-mount") {
      var local=entry.component.geometry();
      var placed=machinekit.assembly.PosedParts.moved(local,F.compose(toDeck,poses.get(entry.id)));
      local.close();
      var box=machinekit.assembly.PosedParts.boxOf(placed);
      placed.close();
      footprints.push({x0:r(box.minX,6),x1:r(box.maxX,6),y0:r(box.minY,6),y1:r(box.maxY,6)});
      xmin=Math.min(xmin,box.minX);xmax=Math.max(xmax,box.maxX);
    }
    if (footprints.length==0) throw "Carriage FE export needs column steel contacts";
    out.columnSoleJoint={contactCentreLocal:[r((xmin+xmax)/2,6),0.0,HobberXCarriage.xCarriageTop()],
      deckThicknessMm:HobberXCarriage.X_CARRIAGE_THICKNESS,contactFootprints:footprints};
    sys.io.File.saveContent('$dir/hobber-fem-export.json', haxe.Json.stringify(out, null, "  "));
    Sys.println("wrote " + dir + "/hobber-fem-export.json");
  }
}

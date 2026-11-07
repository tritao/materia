import machinekit.assembly.JointSlip.JointLoad;
import machinekit.assembly.JointSlip.LoadCase;
import machinekit.motion.ErobActuatorData;
import machinekit.motion.ErobActuatorPart;
import machinekit.motion.ServoMotor;

/**
 * The loads the hobber's bolted joints carry, declared for the slip check of `JointFasteners` (BOM_PLAN F1, `fasteners.slip`). Every figure comes from the
 * machine's own parts and ratings; where a figure is assumed it says so. Joints not listed carry no load the machine knows: the catalogue rails and blocks
 * (their rated loads are the maker's, `HobberFrameChecks.zOffsetMoments`), nut flanges (axial thrust is tension in their screws, not slip), motor flanges
 * (the belt pull is checked in `HobberFrameChecks.spindleDrive`) and the casting's and enclosure's mounts (static weight).
 */
class HobberJointLoads {
  /** Ball screw lead of X, Y and Z, m (the drive sizing's). */
  public static inline final LEAD = 0.005;
  /** The largest hob the spindle takes (the reference hob), radius m, for a jam. */
  public static inline final HOB_RADIUS = 0.016;
  /** The cutting force bound the spindle drive is sized for (`HobberFrameChecks.spindleDrive`), N: ASSUMED upper bound. */
  public static inline final CUTTING_FORCE = 100.0;

  /** The member of `machine` with id `id`. */
  public static function member(machine:FlexsplineHobber, id:String):Dynamic {
    for (entry in machine.components()) if (entry.id == id) return entry.component;
    throw 'The hobber has no member "$id"';
  }

  /** Mass of the members whose id is `id` or starts with `id` + "/", kg. */
  public static function massOf(machine:FlexsplineHobber, ids:Array<String>):Float {
    var total = 0.0;
    for (entry in machine.components()) for (id in ids) if (entry.id == id || StringTools.startsWith(entry.id, id + "/")) total += entry.component.massProperties().mass;
    return total;
  }

  /** Mass of everything that rides on the guide block `first`: its members and all their descendants in the joint graph, kg. */
  public static function movingMass(machine:FlexsplineHobber, first:String):Float {
    var moving = new Map<String, Bool>();
    moving.set(first, true);
    // The definition is derived and copied on every call: take its joints once.
    var joints = machine.definition().joints, changed = true;
    while (changed) {
      changed = false;
      for (j in joints) if (moving.exists(j.parent) && !moving.exists(j.child)) {
        moving.set(j.child, true);
        changed = true;
      }
    }
    var total = 0.0;
    for (entry in machine.components()) if (moving.exists(entry.id)) total += entry.component.massProperties().mass;
    return total;
  }

  public static function of(machine:FlexsplineHobber):Array<JointLoad> {
    var loads:Array<JointLoad> = [], g = 9.80665;
    // Conservative column interface shear: the whole X-moving mass is charged
    // to this joint. Gravity is normal to this horizontal interface, not shear.
    var xCrash = HobberCrashProtection.axis(machine, "X");
    for (pad in machinekit.hobbing.FoundationLayout.columnSolePads()) loads.push({
      label: "Column sole pad " + pad.id + " on the X carriage (whole shear charged to each pad)",
      between: ["column/"+pad.id, "Xbridge"],
      shear: {peak: xCrash.movingKg * 0.5 + CUTTING_FORCE,
        momentary: xCrash.limit.thrust + CUTTING_FORCE}});
    // The A axis: the eRob's output torque passes the output flange, the adapter ring and the swivel plate, and its reaction the housing and the Z saddle.
    // Design peak and momentary (collision or emergency stop) torque are the maker's for the ratio; the flange M4 are tightened to the maker-table torque
    // of 12.9 M4 (4.95 N m, 6790 N, `FastenerTightening`: 80 % of proof) because 70 % leaves only 1.12 x on the momentary torque.
    var row = ErobActuatorData.row(machine.swivel.housing.ratio);
    var torque:LoadCase = {peak: row.peakTorque, momentary: row.momentaryTorque};
    loads.push({label: "A axis: housing reaction on the Z saddle", between: ["Zplate", "swivel/housing"], torque: torque});
    var headKg=movingMass(machine,"Ymodule/base"),headWeight=headKg*g,radius=ErobActuatorData.OUTPUT_PITCH_CIRCLE/2000;
    // Same-radius flange bolts: T + W*r bounds the most loaded bolt when
    // direct gravity shear and torsion act together, without pin credit.
    loads.push({label: "A output combined gravity/torsion, head "+headKg+" kg",between:["Ymodule/base","swivel/output"],torque:{peak:row.peakTorque+headWeight*radius,momentary:row.momentaryTorque+headWeight*radius},utilisation:0.8});
    // The drives' thrust. The design peak is the drive sizing's duty (`HobberCrashProtection`: 0.5 m/s2, 100 N process and 100 N friction, gravity on Z and
    // along the tilted Y, the rotor's inertia and drag) on the axis's moving mass: the sizing's peak torque through the nut's efficiency and the lead. The
    // momentary load (a jam against the stops, a collision) is what the drive's crash protection lets through: the torque limit (`CrashProtection`, 2 x the
    // sizing thrust at most, to be set in the drive) as thrust plus the weight the axis carries along the screw. It passes the fixed support's feet into their
    // pads and brackets, and the nut block's base into its carriage (the nut's flange is perpendicular to the screw: tension, not slip). The nut blocks are
    // located against it by dowels (X and Z) or by a key on the seat (Y), and the seat is held to the Y saddle by friction alone.
    for (axis in [
      {id: "X", pad: "XdrivePad", foot: "Xfixed", carriage: "Xbridge", block: "XnutBlock", seat: null, insert: "bed/driveSeat"},
      {id: "Z", pad: "ZdrivePad", foot: "Zfixed", carriage: "Zplate", block: "ZnutBlock", seat: null, insert: "column/driveSeat"}
    ]) {
      var crash = HobberCrashProtection.axis(machine, axis.id);
      var design = crash.limit.designThrust, jam = crash.limit.thrust + crash.gravityN;
      loads.push({
        label: axis.id + " drive: screw thrust on the fixed support",
        between: [axis.pad, axis.foot],
        shear: {peak: design, momentary: jam}
      });
      // The pad bar passes the same thrust into the cast insert under it (four screws in the groove floor).
      if (axis.insert != null) loads.push({label: axis.id + " drive: screw thrust from the pad bar into the casting insert", between: [axis.pad, axis.insert], shear: {peak: design, momentary: jam}});
      var base = axis.seat == null ? axis.carriage : axis.seat;
      loads.push({label: axis.id + " drive: screw thrust through the nut block", between: [base, axis.block], shear: {peak: design, momentary: jam}});
      if (axis.seat != null) loads.push({label: axis.id + " drive: screw thrust from the nut seat into the saddle", between: [axis.seat, axis.carriage], shear: {peak: design, momentary: jam}});
    }
    var yCrash=HobberCrashProtection.axis(machine,"Y"),yg=yCrash.movingKg*g,ys=machine.yModule.spec;
    var normal=yCrash.limit.designThrust,crash=yCrash.limit.thrust+yCrash.gravityN;
    loads.push({label:"Y fixed support",between:["Ymodule/base","Ymodule/motorMount"],shear:{peak:normal,momentary:crash}});
    loads.push({label:"Y nut carrier",between:["Ymodule/carriage","Ymodule/nutCarrier"],shear:{peak:normal,momentary:crash}});
    loads.push({label:"Y motor pilot",between:["Ymodule/motor","Ymodule/motorMount"],torque:{peak:ys.drive.motor.rating.ratedTorque,momentary:ys.drive.motor.rating.peakTorque}});
    // Rails transmit the A couple and transverse payload loads. Screw thrust
    // passes through the nut/fixed support; a Y stop takes a Y-axis crash.
    // Charge the whole moving weight and the assumed100 N guide drag to EACH
    // rail, then add the100 N cutting bound in its worst direction.
    var angle=machinekit.hobbing.HobberSwivelLayout.RANGE,along=yg*Math.sin(angle)+100,across=yg*Math.cos(angle);
    var fPeak=row.peakTorque/(ys.railSpacing/1000)+along,fCrash=row.momentaryTorque/(ys.railSpacing/1000)+along;
    var railPeak=Math.sqrt(fPeak*fPeak+across*across)+CUTTING_FORCE,railCrash=Math.sqrt(fCrash*fCrash+across*across)+CUTTING_FORCE;
    var railLoad:LoadCase={peak:railPeak,momentary:railCrash};
    for(side in ["Left","Right"]) {
      loads.push({label:"Y riser "+side,between:["Ymodule/riser"+side,"Ymodule/base"],shear:railLoad});
      loads.push({label:"Y rail "+side,between:["Ymodule/rail"+side,"Ymodule/riser"+side],shear:railLoad});
      for(end in ["Front","Rear"])loads.push({label:"Y guide block "+side+end,between:["Ymodule/carriage","Ymodule/block"+side+end],shear:{peak:railPeak/2,momentary:railCrash/2}});
    }
    for(end in ["Front","Rear"]) {
      for(side in ["Left","Right"])loads.push({label:"Y stop "+end+side,between:["Ymodule/stop"+end+side,"Ymodule/base"],shear:{peak:normal,momentary:crash}});
      loads.push({label:"Y guard "+end,between:["Ymodule/cover"+end,"Ymodule/base"],shear:{peak:30.0,momentary:100.0}});
      loads.push({label:"Y retainer "+end,between:["Ymodule/retainer"+end,"Ymodule/cover"+end],shear:{peak:10.0,momentary:20.0}});
    }
    for(pair in [["scaleSeat","base"],["scaleHousing","scaleSeat"],["scaleHead","scaleHeadCarrier"],["carriage","scaleHeadCarrier"]])loads.push({label:"Y scale "+pair[0],between:["Ymodule/"+pair[0],"Ymodule/"+pair[1]],shear:{peak:20.0,momentary:40.0}}); // ASSUMED scale/cable drag.
    // The spindle seat on the Y saddle's plate carries the weight of the seat, cradle and spindle (with the Y axis's 0.5 m/s2) and the cutting force; a jammed
    // hob loads it with the tangential force the servo's peak torque makes at the hob's radius.
    var mass = massOf(machine, ["spindleSeat", "motorCradle", "spindle"]);
    var jam = machine.spindle.jointTorque(machine.spindle.motor.rating.peakTorque) / HOB_RADIUS;
    var weight = mass * (g + 0.5);
    loads.push({label: "Spindle seat on the Y saddle plate", between: ["Ymodule/carriage", "spindleSeat"], shear: {peak: weight + CUTTING_FORCE, momentary: weight + jam}});
    // The cradle on the seat carries the belt's pull on the servo at its peak torque (twice the pretension, `HobberFrameChecks.spindleDrive`) and the servo group's weight.
    var pull = 2 * machinekit.hobbing.HobSpindle.pretension(machine.spindle.motor.rating.peakTorque, machine.spindle.motorPulley.pitchDiameter);
    loads.push({
      label: "Motor cradle on the spindle seat",
      between: ["motorCradle", "spindleSeat"],
      shear: {peak: pull + massOf(machine, ["motorCradle", "spindle/motor"]) * g}
    });
    var outboardWeight = massOf(machine, FlexsplineHobber.outboardMembers().concat(["outboardInnerSpacer", "outboardLocknut"])) * (g + 0.5);
    loads.push({label: "Outboard carrier on the Y saddle", between: ["outboardCarrier", "Ymodule/carriage"],
      shear: {peak: CUTTING_FORCE + outboardWeight, momentary: jam + outboardWeight}});
    loads.push({label: "Indicated housing on its carrier", between: ["outboard", "outboardCarrier"],
      shear: {peak: CUTTING_FORCE + outboardWeight, momentary: jam + outboardWeight}});
    loads.push({label: "Floating sleeve rear stop cap", between: ["outboard", "outboardCap"],
      shear: {peak: massOf(machine, ["outboardCap"]) * (g + 0.5)}});
    // Chip guards carry their own mass with the axis acceleration and an assumed
    // 2 N debris/handling force, not cutting thrust. Apply the whole fixed X
    // assembly weight to each foot conservatively; all loads remain joint-derived.
    var fixedGuardWeight=massOf(machine,machinekit.hobbing.HobberChipCover.fixedMembers())*(g+0.5)+2;
    for(r in machinekit.hobbing.HobberChipCover.installations()) {
      var shear=r.parent=="XdrivePad"?fixedGuardWeight:massOf(machine,[r.id])*(g+0.5)+2;
      loads.push({label:"Chip guard "+r.id,between:[r.id,r.parent],shear:{peak:shear}});
    }
    // Each lid also transfers weight through its other side, whose fixed tree
    // parent is the common X pad rather than the lid.
    loads.push({label:"Front lid on right sheet",between:["XfrontCover","XcoverFrontRight"],shear:{peak:fixedGuardWeight}});
    loads.push({label:"Rear lid on right sheet",between:["XrearCover","XcoverRearRight"],shear:{peak:fixedGuardWeight}});
    return loads;
  }
}
